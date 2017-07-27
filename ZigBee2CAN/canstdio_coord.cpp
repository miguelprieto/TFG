#include "defines.h"

#include <xc.h>
#include <libpic30++.h>
#include <string.h>

#include "bus_interface.h"
#include "canstdio_coord.h"
#include "circqueue.h"
#include "ecan_lib.h"
#include "xbee.h"

/**
 * DESCRIZIONE DEL PROTOCOLLO "canstdio"
 *
 * Terminologia
 *   Nodo          Scheda collegata a bus CAN, identificata da ID compreso tra 0 e 15
 *   Coordinatore  Il nodo ZigBee2CAN (ie questo firmware), ha ID == 0
 *   Endpoint      Un nodo diverso da ZigBee2CAN (ie gli altri firmware), ha ID != 0
 *   Client        Dispositivo che effettua I/O tramite XBee (ie il computer)
 *
 * Descrizione generale
 *   Il ruolo principale del coordinatore è di mantenere dei buffer intermedi
 *   tra ciascun client e il endpoint a cui tale client è connesso.
 *   Il coordinatore tiene inoltre traccia dei client connessi e manda dei
 *   messaggi CAN di attivazione e disattivazione ai endpoint. Ad esempio,
 *   quando un client cerca di connettersi a un endpoint, il coordinatore
 *   informa tale endpoint tramite un messaggio di attivazione. Se il client si
 *   disconnette, viceversa, viene inviato un messaggio di disattivazione. Se un
 *   endpoint è disattivato, esso non deve inviare messaggi CAN (sarebbero
 *   inutili, dato che non c'è nessun client in ascolto).
 *   Vengono bufferizzati sia i dati in uscita (endpoint -> client) che quelli
 *   in ingresso (client -> endpoint), ma queste due tipologie di buffer sono
 *   gestite in modo diverso.
 *
 * Nota sull'affidabilità
 *   Il protocollo cerca di garantire la consegna dei dati inoltrati dai
 *   endpoint ai client (ovvero l'output delle schede), ma non viceversa.
 *   Sia i messaggi CAN sia i messaggi ZigBee contengono dei numeri di sequenza.
 *   I messaggi CAN hanno un seqnum a 8 bit (legato al numero di pacchetti
 *   trasmessi), i messaggi ZigBee a 16 bit (legato al numero di caratteri
 *   trasmessi). Tali campi non sono correlati tra loro.
 *   Nel caso di CAN, che è affidabile, non vengono effettuate ritrasmissioni e
 *   il numero di sequenza viene utilizzato solo per limitare il rate di invio.
 *   Nel caso di ZigBee, che è inaffidabile, i messaggi di cui non si riceve
 *   conferma entro un timeout vengono ritrasmessi.
 *
 * Formato dei messaggi ZigBee
 *   Tutti i messaggi ZigBee elencati di seguito iniziano con due byte 0xFF
 *   0xFF, in modo da poterli distinguere dai messaggi ZigBee che incapsulano
 *   messaggi CAN (0xFF 0xFF non è un tag CAN valido).
 *   Le comunicazioni ZigBee avvengono tra coordinatore e client con
 *   indirizzamento unicast.
 *
 *   KEEPALIVE (client -> coordinatore)
 *     0xFF | 0xFF | 0xFF | #nodes | node0 | node1 | ... | node_{#nodes-1}
 *     Questo pacchetto viene inviato periodicamente dal client e contiene
 *     l'elenco dei dispositivi ai quali tale client è interessato (senza
 *     padding). Per ciascun nodo vengono trasmessi 3 byte:
 *       id-nodo | seqnumH | seqnumL   (seqnum è un intero big-endian a 16 bit)
 *
 *   INPUTDATA (client -> coordinatore)
 *     0xFF | 0xFF | (0x80 | #endpoint-id) | #bytes | byte0 | byte1 | ... | byte_{#bytes-1} | seqnumH | seqnumL
 *     Questo pacchetto viene inviato dal client per trasmettere uno o più
 *     caratteri destinati all'endpoint #endpoint-id
 *
 *   INPUTACK (coordinatore -> client)
 *     0xFF | 0xFF | (0x20 | #endpoint-id) | seqnumH | seqnumL
 *     Invia ACK cumulativo per dati ricevuti tramite INPUTDATA
 *
 *   OUTPUTDATA (coordinatore -> client)
 *     0xFF | 0xFF | #endpoint-id | byte0 | byte0 | ... | byteN | seqnumH | seqnumL
 *     Questo pacchetto viene inviato al client per trasmettere uno o più
 *     caratteri emessi dall'endpoint #endpoint-id. Il campo seqnum è un intero
 *     big-endian a 16-bit, che viene incrementato di uno per ogni carattere
 *     trasmesso. Le ritrasmissioni, invece, *non* contribuiscono a tali
 *     incrementi.
 *
 * Formato dei messaggi CAN (per il formato dei dati v. bus_objects.h)
 *   La comunicazione avviene sempre tra il coordinatore e un endpoint. Non
 *   possono esistere due coordinatori e non sono ammessi messaggi tra endpoint.
 *
 *   REFRESH (coordinatore -> endpoint)
 *     Informa l'endpoint sul suo stato di attivazione (ovvero se un client è
 *     connesso o meno), se il buffer intermedio can_to_xbee è pieno (in tal
 *     caso l'endpoint deve sospendere l'invio di ulteriori caratteri) e include
 *     il seqnum dell'ultimo carattere ricevuto dal endpoint.
 *     Può includere inoltre fino a 6 caratteri di input destinati al endpoint.
 *
 *   OUTPUTDATA (endpoint -> coordinatore)
 *     Invia al coordinatore fino a 6 caratteri di output, accompagnati da un
 *     numero di sequenza. Il coordinatore risponde immediatamente con un
 *     REFRESH che riporta lo stesso numero di sequenza. Fino alla ricezione di
 *     tale REFRESH, l'endpoint non deve inviare ulteriori messaggi al
 *     coordinatore.
 */

// Tempistica messaggi CAN
#define REFRESH_INTERVAL	2  // intervallo tra due pacchetti di refresh (0.1 s)

// Tempistica messaggi ZigBee
#define RETRANSMIT_INTERVAL	5  // intervallo di ritrasmissione - .25 secondi
#define KEEPALIVE_VALIDITY	30 // tempo di validità di un keepalive - 1.5 secondi

// Informazioni relative a ciascun nodo
static struct
{
	// 0 = nodo disattivato, 1 o più = tempo rimanente in cui tenere attivo lo stdio per il nodo
	int keepalive_time;

	// indirizzo xbee del client connesso
	uint8_t client_addr[2];

	// tempo rimanente prima dell'invio del prossimo pacchetto di REFRESH al nodo
	int refresh_time;

	// ultimo numero di sequenza ricevuto dall'endpoint
	char seqnum_can;

	// buffer per inoltro da xbee a can
	circqueue_t xbee_to_can;

	// tempo rimanente prima della prossima trasmissione al client
	int transmit_time; // -1 se disattivato

	// buffer per inoltro da can a xbee
	circqueue_t can_to_xbee;
	bool can_to_xbee_full;

	// numero di sequenza da inviare al client
	unsigned int seqnum_out;

	// ultimo numero di sequenza ricevuto da client
	unsigned int seqnum_in;
	bool seqnum_in_valid;
} nodes[REMOTE_STDIO_MAX_NODES];

void canstdio_coord_init()
{
	int i;

	for (i = 0; i < REMOTE_STDIO_MAX_NODES; i++)
	{
		nodes[i].keepalive_time = 0;
		circqueue_clear(&nodes[i].can_to_xbee);
		circqueue_clear(&nodes[i].xbee_to_can);
		nodes[i].can_to_xbee_full = false;
		nodes[i].transmit_time = -1;
		nodes[i].refresh_time = 0;
		nodes[i].seqnum_out = 0;
	}
}

// invia messaggio ZigBee di tipo OUTPUTDATA
static void send_xbee_inputack(int node_id)
{
	uint8_t buffer[2 + 1 + 2];
	uint8_t dest_addr[2];

	buffer[0] = 0xFF;
	buffer[1] = 0xFF;
	buffer[2] = 0x20 | node_id;
	buffer[3] = (nodes[node_id].seqnum_in >> 8) & 0xff;
	buffer[4] = nodes[node_id].seqnum_in & 0xff;

	memcpy(dest_addr, nodes[node_id].client_addr, 2);
	xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, buffer, 5);
}

// invia messaggio ZigBee di tipo OUTPUTDATA
static void send_xbee_output(int node_id)
{
	uint8_t buffer[2 + 1 + CIRCQUEUE_LENGTH + 2];
	uint8_t dest_addr[2];

	buffer[0] = 0xFF;
	buffer[1] = 0xFF;
	buffer[2] = node_id;

	int current_cpu_ipl;
	SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7); // disattiva interrupt

	// Copia contenuto coda circolare in buffer
	int packet_length = 3 + circqueue_copy((char*)&buffer[3], &nodes[node_id].can_to_xbee);
	buffer[packet_length++] = (nodes[node_id].seqnum_out >> 8) & 0xff;
	buffer[packet_length++] = nodes[node_id].seqnum_out & 0xff;

	memcpy(dest_addr, nodes[node_id].client_addr, 2);
	nodes[node_id].transmit_time = RETRANSMIT_INTERVAL;

	RESTORE_CPU_IPL(current_cpu_ipl); // riattiva interrupt

	xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, buffer, packet_length);
}

// invia messaggio CAN di tipo REFRESH
static void send_can_refresh(int node_id)
{
	t_can_remote_stdio_coord2endpoint m;

	nodes[node_id].refresh_time = REFRESH_INTERVAL;
	m.seqnum_ack = nodes[node_id].seqnum_can;

	if (nodes[node_id].keepalive_time == 0)
	{
		// nodo disattivato
		m.data_len = -1;
	}
	else
	{
		unsigned int data_len = 0;
		while (!circqueue_isempty(&nodes[node_id].xbee_to_can) && data_len < sizeof(m.data))
			m.data[data_len++] = circqueue_dequeue(&nodes[node_id].xbee_to_can);
		m.data_len = data_len | (nodes[node_id].can_to_xbee_full ? 0x80 : 0);
	}

	ecan_send(REMOTE_STDIO_CAN_ID(node_id), (unsigned char*)&m, 8, 0);
}

// N.B.: Questa funzione viene eseguita in contesto di interrupt!
void canstdio_coord_process_can_frame(const uint8_t *data, unsigned int len, void *user_ptr)
{
	const t_can_remote_stdio_endpoint2coord *m = (const t_can_remote_stdio_endpoint2coord*)data;

	const int node_id = (m->data_len >> 4) & 0xf; // estrai node_id da 4 bit più alti
	const int msg_len = m->data_len & 0x7; // estrai lunghezza dati da 3 bit più bassi
	int i;

	nodes[node_id].seqnum_can = m->seqnum;
	nodes[node_id].refresh_time = 0;

	// se il nodo non è attivo, ignoriamo i dati ricevuti
	if (nodes[node_id].keepalive_time == 0)
		return;

	// accoda dati ricevuti a can_to_xbee
	for (i = 0; i < msg_len && !circqueue_isfull(&nodes[node_id].can_to_xbee); i++)
	{
		circqueue_enqueue(&nodes[node_id].can_to_xbee, m->data[i]);
		nodes[node_id].seqnum_out++;
	}

	// avviamo timer di trasmissione
	if (nodes[node_id].transmit_time == -1)
		nodes[node_id].transmit_time = 0;

	// se il buffer è quasi pieno, richiediamo un flush immediato
	if (circqueue_isfull(&nodes[node_id].can_to_xbee))
	{
		nodes[node_id].can_to_xbee_full = true;
		nodes[node_id].transmit_time = 0;
	}
}

void canstdio_coord_process_xbee_frame(const uint8_t *sender_addr, const uint8_t *data)
{
	if (data[0] == 0xff) // KEEPALIVE
	{
		int current_cpu_ipl;
		SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7); // disattiva interrupt

		const uint8_t *nodelist = data + 2;
		int nodelist_len = data[1];
		while (nodelist_len-- != 0)
		{
			int node_id = nodelist[nodelist_len * 3];
			unsigned int seqnum_out = ((unsigned int)nodelist[nodelist_len * 3 + 1] << 8) | (nodelist[nodelist_len * 3 + 2] & 0xFF);

			// se il nodo era disattivato oppure è cambiato il client,
			// invalidiamo l'info sull'ultimo seqnum ricevuto
			if (nodes[node_id].keepalive_time == 0 || memcmp(nodes[node_id].client_addr, sender_addr, 2) != 0)
				nodes[node_id].seqnum_in_valid = false;

			// se il nodo era disattivato, pianifichiamo un refresh
			// immediato e avviamo il timer di trasmissione
			if (nodes[node_id].keepalive_time == 0)
			{
				nodes[node_id].refresh_time = 0;
				nodes[node_id].transmit_time = RETRANSMIT_INTERVAL;
			}

			memcpy(nodes[node_id].client_addr, sender_addr, 2);
			nodes[node_id].keepalive_time = KEEPALIVE_VALIDITY;

			// Abbiamo ricevuto un ACK per tutti i dati in can_to_xbee,
			// quindi smettiamo di ritrasmettere
			if (seqnum_out == nodes[node_id].seqnum_out)
			{
				circqueue_clear(&nodes[node_id].can_to_xbee);
				nodes[node_id].can_to_xbee_full = false;
				nodes[node_id].transmit_time = -1;
			}

			if (!circqueue_isempty(&nodes[node_id].can_to_xbee))
				nodes[node_id].transmit_time = 0;
		}

		RESTORE_CPU_IPL(current_cpu_ipl); // riattiva interrupt
	}
	else if (data[0] >= 0x80 && data[0] < 0x80 + REMOTE_STDIO_MAX_NODES) // INPUT INJECTION
	{
		int node_id = data[0] & 0xf;
		unsigned int i, data_len = data[1];
		unsigned int seqnum_in = ((unsigned int)data[2 + data_len] << 8) | (data[2 + data_len + 1] & 0xFF);

		if (nodes[node_id].keepalive_time == 0) // ignoriamo i dati se il nodo è disattivato
			return;

		unsigned int useful_data_len; // quanti dati sono effettivamente nuovi?
		if (nodes[node_id].seqnum_in_valid)
		{
			// Dei dati ricevuti, solo gli ultimi
			//  seqnum_in - nodes[node_id].seqnum_in
			// sono effettivamente caratteri nuovi
			useful_data_len = seqnum_in - nodes[node_id].seqnum_in;

			if (useful_data_len > 16)
				return; // abbiamo ricevuto un pacchetto vecchio

			if (useful_data_len > data_len)
				useful_data_len = data_len;
		}
		else
		{
			// Il vecchio seqnum_in non era valido, quindi inoltriamo
			// tutti i caratteri ricevuti
			useful_data_len = data_len;
		}

		nodes[node_id].seqnum_in_valid = true;
		nodes[node_id].seqnum_in = seqnum_in;

		for (i = data_len - useful_data_len; i < data_len; i++)
			circqueue_enqueue(&nodes[node_id].xbee_to_can, data[2 + i]);

		send_xbee_inputack(node_id);
	}
}

void canstdio_coord_send_request_rtsp(int node_id)
{
	t_can_remote_stdio_coord2endpoint m;

	m.seqnum_ack = nodes[node_id].seqnum_can;
	m.data_len = -2;

	ecan_send(REMOTE_STDIO_CAN_ID(node_id), (unsigned char*)&m, 8, 0);
}

void canstdio_coord_idle()
{
	int i;
	for (i = 1; i < REMOTE_STDIO_MAX_NODES; i++) // iniziamo da 1 (skip coordinatore)
	{
		if (nodes[i].transmit_time == 0)
			send_xbee_output(i);
		if (nodes[i].refresh_time == 0)
			send_can_refresh(i);
	}
}

void canstdio_coord_interval()
{
	int i, current_cpu_ipl;
	SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7); // disattiva interrupt

	for (i = 0; i < REMOTE_STDIO_MAX_NODES; i++)
	{
		// è scaduta la validità del keepalive?
		if (nodes[i].keepalive_time != 0 && --nodes[i].keepalive_time == 0)
		{
			// scarta eventuale contenuto dei buffer
			circqueue_clear(&nodes[i].can_to_xbee);
			circqueue_clear(&nodes[i].xbee_to_can);

			// annulla azioni pianificate
			nodes[i].can_to_xbee_full = false;
			nodes[i].transmit_time = -1;

			// invia REFRESH immediatamente
			nodes[i].refresh_time = 0;
		}

		if (nodes[i].refresh_time > 0)
			nodes[i].refresh_time--;
		if (nodes[i].transmit_time > 0)
			nodes[i].transmit_time--;

		// se è arrivato il momento di trasmettere ma non c'è nulla da
		// trasmettere teniamo il timeout a 1, in modo da essere pronti
		// immediatamente la prossima volta che ci saranno dati
		if (circqueue_isempty(&nodes[i].can_to_xbee) && nodes[i].transmit_time == 0)
			nodes[i].transmit_time = 1;
	}

	RESTORE_CPU_IPL(current_cpu_ipl); // riattiva interrupt
}
