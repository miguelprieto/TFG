#include "defines.h"

#include <xc.h>
#include <libpic30++.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "bus_interface.h"
#include "bus_objects.h"
#include "canstdio_coord.h"
#include "circqueue.h"
#include "ecan_lib.h"
#include "minilzo.h"
#include "rtsp_coord.h"
#include "xbee.h"

#define ROW_SIZE (64 * 3)
#define MAX_PAGE_SIZE (ROW_SIZE * 16) // dsPIC33EP512MC502 ha 16 righe per pagina

// Controllo della comunicazione XBee
#define KEEPALIVE_VALIDITY	90 // tempo di validità di un keepalive - 4.5 secondi
static uint8_t connected_client[2];
static int connected_client_keepalive; // -1 se nessuno è connesso
static uint8_t connected_client_last_seqnum; // ultimo seqnum visto
static uint8_t connected_client_last_reply[100]; // ultima risposta inviata
static unsigned int connected_client_last_reply_len;
static unsigned int page_size;
static uint8_t page_buffer[MAX_PAGE_SIZE] __attribute__((far)); // buffer di accumulazione word da programmare
static uint8_t temp_buffer[MAX_PAGE_SIZE] __attribute__((far)); // buffer di supporto
static int page_buffer_count; // -1 in caso di overflow
static uint32_t page_target_address;
static uint8_t selected_endpoint_id;

// Controllo della comunicazione CAN
static bool can_got_reply;
static uint16_t can_reply_data, can_reply_data2;
static uint8_t can_reply_payload[6];
static char can_last_seqnum;

// Costanti per CRC16
#define CRC16_INITIAL	0xFFFF
#define CRC16_POLY	0xA001

// Calcola CRC16 (val. iniziale crc = CRC16_INITIAL)
static uint16_t crc16(const uint8_t *buffer, unsigned int len)
{
	uint16_t crc = CRC16_INITIAL;
	int i;

	while (len-- > 0)
	{
		crc ^= *buffer++;

		for (i = 0; i < 8; ++i)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ CRC16_POLY;
			else
				crc = (crc >> 1);
		}
	}

	return crc;
}

void rtsp_coord_init()
{
	connected_client_keepalive = -1;
}

void rtsp_coord_process_can_frame(const uint8_t *data, unsigned int len, void *user_ptr)
{
	const t_can_remote_rtsp_reply *m = (const t_can_remote_rtsp_reply*)data;
	if (m->seqnum_echo == can_last_seqnum)
	{
		can_reply_data = m->data;
		can_reply_data2 = m->data2;
		memcpy(can_reply_payload, m->payload, sizeof(m->payload));
		can_got_reply = true;
	}
}

static bool send_can_and_wait_reply(t_can_remote_rtsp_command *m)
{
	int i;

	m->seqnum = ++can_last_seqnum;
	can_last_seqnum &= 0xf; // Il campo seqnum è a 4 bit

	can_got_reply = false;
	ecan_send(REMOTE_RTSP_CAN_ID(selected_endpoint_id), (unsigned char*)m, 8, 0);

	// 50 ms timeout
	for (i = 0; i < 500; i++)
	{
		ecan_update_object(REMOTE_RTSP_RX_OBJECT);
		if (can_got_reply)
			return true;
		__delay_us(100);
	}

	return false;
}

static bool write_row(const uint8_t *row_data, uint32_t row_address)
{
	t_can_remote_rtsp_command m;
	unsigned int i;

	m.cmd = REMOTE_RTSP_COMMAND_ROW_SELECT;
	m.target_address = row_address;
	if (!send_can_and_wait_reply(&m))
		return false;

	for (i = 0; i < ROW_SIZE; i += sizeof(m.payload))
	{
		unsigned int len = ROW_SIZE - i;
		if (len > sizeof(m.payload))
			len = sizeof(m.payload);

		m.cmd = REMOTE_RTSP_COMMAND_ROW_PUTDATA;
		memcpy(m.payload, &row_data[i], len);
		if (!send_can_and_wait_reply(&m))
			return false;
	}

	m.cmd = REMOTE_RTSP_COMMAND_ROW_FLUSH;
	m.data = crc16(row_data, ROW_SIZE);
	if (!send_can_and_wait_reply(&m))
		return false;

	return can_reply_data == 1;
}

static bool read_row(uint8_t *row_data, uint32_t row_address)
{
	t_can_remote_rtsp_command m;
	unsigned int i;

	m.cmd = REMOTE_RTSP_COMMAND_ROW_SELECT;
	m.target_address = row_address;
	if (!send_can_and_wait_reply(&m))
		return false;

	for (i = 0; i < ROW_SIZE; i += sizeof(can_reply_payload))
	{
		unsigned int len = ROW_SIZE - i;
		if (len > sizeof(can_reply_payload))
			len = sizeof(can_reply_payload);

		m.cmd = REMOTE_RTSP_COMMAND_ROW_GETDATA;
		if (!send_can_and_wait_reply(&m))
			return false;
		memcpy(&row_data[i], can_reply_payload, len);
	}

	return true;
}

static bool write_page()
{
	t_can_remote_rtsp_command m;
	unsigned int i;

	m.cmd = REMOTE_RTSP_COMMAND_PAGE_ERASE;
	m.target_address = page_target_address;
	if (!send_can_and_wait_reply(&m))
		return false;

	for (i = 0; i < page_size; i += ROW_SIZE)
	{
		if (!write_row(page_buffer + i, page_target_address + i * 2 / 3))
			return false;
	}

	return true;
}

static bool read_page_to_temp_buffer()
{
	unsigned int i;

	for (i = 0; i < page_size; i += ROW_SIZE)
	{
		if (!read_row(temp_buffer + i, page_target_address + i * 2 / 3))
			return false;
	}

	return true;
}

static void send_xbee_reply(const uint8_t *data, unsigned int datalen)
{
	memcpy(connected_client_last_reply, data, datalen);
	connected_client_last_reply_len = datalen;
	xbee_tx_request(connected_client, XBEE_ADDR16_LENGTH, connected_client_last_reply, connected_client_last_reply_len);
}

void rtsp_coord_process_xbee_frame(const uint8_t *sender_addr, const uint8_t *data)
{
	static uint8_t replybuffer[100];
	t_can_remote_rtsp_command m;

	replybuffer[0] = 0xFF;
	replybuffer[1] = 0xFE;
	replybuffer[2] = data[2]; // echo seqnum

	// Se abbiamo ricevuto una richiesta di connessione ma c'è già un client
	// connesso, rispondiamo che siamo occupati...
	if (data[3] == 0x80 && connected_client_keepalive != -1)
	{
		if (memcmp(connected_client, sender_addr, 2) == 0)
		{
			// ... a meno che non sia lo stesso client connesso che
			// sta cercando di connettersi di nuovo. In tal caso
			// dimentichiamo la vecchia connessione
			connected_client_keepalive = -1;
		}
		else
		{
			replybuffer[3] = 0;
			xbee_tx_request(sender_addr, XBEE_ADDR16_LENGTH, replybuffer, 4);
			return;
		}
	}

	// Ignora pacchetti non inviati dal client connesso (a meno che non si
	// tratti di una richiesta di connessione)
	if (data[3] != 0x80 && (connected_client_keepalive == -1 || memcmp(connected_client, sender_addr, 2) != 0))
		return;

	// Se abbiamo ricevuto un pacchetto duplicato, inviamo di nuovo ciò che
	// avevamo già inviato in risposta al pacchetto originale
	if (connected_client_keepalive != -1 && data[2] == connected_client_last_seqnum)
	{
		// Rinnoviamo il keepalive della connessione attiva
		connected_client_keepalive = KEEPALIVE_VALIDITY;

		// Ritrasmissione della vecchia risposta
		xbee_tx_request(connected_client, XBEE_ADDR16_LENGTH, connected_client_last_reply, connected_client_last_reply_len);
		return;
	}

	// Rinnoviamo il keepalive della connessione attiva e memorizziamo il
	// nuovo seqnum, per poter individuare eventuali pacchetti duplicati futuri
	connected_client_keepalive = KEEPALIVE_VALIDITY;
	connected_client_last_seqnum = data[2];

	switch (data[3])
	{
		case 0x80: // Richiesta di connessione accettata
		{
			memcpy(connected_client, sender_addr, 2);
			selected_endpoint_id = data[4];

			page_size = data[6];
			page_size |= (uint32_t)data[5] << 8;
			page_size *= 3;

			canstdio_coord_send_request_rtsp(selected_endpoint_id);
			__delay_ms(5); // attendiamo che venga eseguita la richiesta di switch a RTSP

			m.cmd = REMOTE_RTSP_COMMAND_GET_INFO;
			if (send_can_and_wait_reply(&m))
			{
				replybuffer[3] = (uint8_t)(can_reply_data >> 8);
				replybuffer[4] = (uint8_t)(can_reply_data >> 0);
				replybuffer[5] = (uint8_t)(can_reply_data2 >> 8);
				replybuffer[6] = (uint8_t)(can_reply_data2 >> 0);
				send_xbee_reply(replybuffer, 7);
			}
			else
			{
				replybuffer[3] = 1;
				send_xbee_reply(replybuffer, 4);
			}
			break;
		}
		case 0x81: // Richiesta di checksum di un insieme di pagine
		{
			int i;

			for (i = 0; i < data[4]; i++)
			{
				m.cmd = REMOTE_RTSP_COMMAND_PAGE_CRC;
				m.target_address = data[5+4*i+3];
				m.target_address |= (uint32_t)data[5+4*i+2] << 8;
				m.target_address |= (uint32_t)data[5+4*i+1] << 16;
				m.target_address |= (uint32_t)data[5+4*i+0] << 24;

				if (!send_can_and_wait_reply(&m))
					break; // Interrompi in caso di timeout

				replybuffer[3 + 2*i] = (uint8_t)(can_reply_data >> 8);
				replybuffer[3 + 2*i + 1] = (uint8_t)(can_reply_data >> 0);
			}
			send_xbee_reply(replybuffer, 3 + 2*i);
			break;
		}
		case 0x82: // Inizio di pagina da programmare
		{
			const unsigned int addr_offset = 5 + data[4];
			page_target_address = data[addr_offset+3];
			page_target_address |= (uint32_t)data[addr_offset+2] << 8;
			page_target_address |= (uint32_t)data[addr_offset+1] << 16;
			page_target_address |= (uint32_t)data[addr_offset+0] << 24;
			page_buffer_count = 0;
			// niente break, eseguiamo anche il codice di 0x83
		}
		case 0x83: // Porzione intermedia di pagina da programmare
		case 0x84: // Porzione finale di pagina da programmare
		{
			int i, j;

			for (i = 0; i < data[4] && page_buffer_count != -1; i++)
			{
				if (page_buffer_count == sizeof(page_buffer))
					page_buffer_count = -1;
				else
					page_buffer[page_buffer_count++] = data[5 + i];
			}

			if (data[3] != 0x84) // Porzione iniziale o intermedia
			{
				const uint16_t crc = crc16(page_buffer, page_buffer_count);
				replybuffer[3] = (uint8_t)(page_buffer_count >> 8);
				replybuffer[4] = (uint8_t)(page_buffer_count >> 0);
				replybuffer[5] = (uint8_t)(crc >> 8);
				replybuffer[6] = (uint8_t)(crc >> 0);
				send_xbee_reply(replybuffer, 7);
			}
			else // Porzione finale
			{
				const unsigned int expected_crc_offset = 5 + data[4];
				uint16_t expected_crc = data[expected_crc_offset + 1];
				expected_crc |= (uint32_t)data[expected_crc_offset] << 8;
				uint16_t compression_info = data[expected_crc_offset + 3];
				compression_info |= (uint32_t)data[expected_crc_offset + 2] << 8;

				bool is_compressed;
				int diff_rotation;
				//printf("rtsp: page_target_address = 0x%06lx with compression_info = %u\n", (unsigned long)page_target_address, compression_info);
				switch (compression_info)
				{
					case 0 ... (MAX_PAGE_SIZE - 1): // diff compresso
						is_compressed = true;
						diff_rotation = compression_info;
						break;
					case 4000: // diretto compresso
						is_compressed = true;
						diff_rotation = -1;
						break;
					case 4001: // diretto non compresso
						is_compressed = false;
						diff_rotation = -1;
						break;
					default: // valore non valido
						page_buffer_count = -1; // invalida buffer
						break;
				}

				if (is_compressed && page_buffer_count > 0)
				{
					// Sposta dati compressi da page_buffer a temp_buffer
					memcpy(temp_buffer, page_buffer, page_buffer_count);

					// Decomprimi in page_buffer
					lzo_uint out_len = sizeof(page_buffer);
					if (lzo1x_decompress_safe(temp_buffer, page_buffer_count,
						page_buffer, &out_len, NULL) == LZO_E_OK)
					{
						// Decompressione avvenuta con successo
						page_buffer_count = out_len;
					}
					else
					{
						// Decompressione fallita
						page_buffer_count = -1; // Invalida buffer
					}
				}

				// Se abbiamo ricevuto un diff, applichiamolo adesso
				if (page_buffer_count == (int)page_size && diff_rotation != -1)
				{
					if (!read_page_to_temp_buffer())
					{
						page_buffer_count = -1;
					}
					else
					{
						for (i = 0, j = diff_rotation + (int)page_size;
							i < (int)page_size; i++, j++)
						{
							if (j >= (int)page_size)
								j -= (int)page_size;

							page_buffer[i] += temp_buffer[j];
						}
					}
				}

				if (page_buffer_count == (int)page_size
					&& crc16(page_buffer, page_size) == expected_crc
					&& write_page())
				{
					replybuffer[3] = 1; // Successo
				}
				else
				{
					replybuffer[3] = 0; // Errore
				}

				send_xbee_reply(replybuffer, 4);
			}
			break;
		}
		case 0x85: // Reset
		{
			m.cmd = REMOTE_RTSP_COMMAND_RESET;
			replybuffer[3] = send_can_and_wait_reply(&m) ? 1 : 0;
			send_xbee_reply(replybuffer, 4);
			break;
		}
		default:
		{
			break;
		}
	}
}

void rtsp_coord_interval()
{
	if (connected_client_keepalive-- == -1)
		connected_client_keepalive = -1;
}
