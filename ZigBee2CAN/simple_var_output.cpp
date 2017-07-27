#include "bus_objects.h"
#include "simple_var_output.h"
#include "xbee.h"

#include <xc.h>
#include <libpic30++.h>
#include <string.h>

struct buffer
{
	char datastream[80];
	int datalength;
	int nextctr;
	bool overflow;
};

static struct buffer wr_buffer;
static bool wr_buffer_full = false;
static uint8_t tempbuffer[sizeof(wr_buffer.datastream) + 4];

static void add_to_buffer(int type, const void *dataptr)
{
	int datalen, strlen;

	if (wr_buffer.overflow) // se siamo già in overflow non aggiungere nulla
		return;

	switch (type)
	{
		case SIMPLE_VAR_TYPE_NULL:
			datalen = 0;
			break;
		case SIMPLE_VAR_TYPE_CHAR:
			datalen = 1;
			break;
		case SIMPLE_VAR_TYPE_INT8:
			datalen = 1;
			break;
		case SIMPLE_VAR_TYPE_UINT8:
			datalen = 1;
			break;
		case SIMPLE_VAR_TYPE_BOOL:
			datalen = 1;
			break;
		case SIMPLE_VAR_TYPE_INT16:
			datalen = 2;
			break;
		case SIMPLE_VAR_TYPE_UINT16:
			datalen = 2;
			break;
		case SIMPLE_VAR_TYPE_INT32:
			datalen = 4;
			break;
		case SIMPLE_VAR_TYPE_UINT32:
			datalen = 4;
			break;
		case SIMPLE_VAR_TYPE_FLOAT:
			datalen = 4;
			break;
		case SIMPLE_VAR_TYPE_STRING:
		{
			strlen = 0;
			while (strlen < 6 && ((const char*)dataptr)[strlen] != '\0')
				strlen++;
			datalen = 1 + strlen;
			break;
		}
	}

	if (wr_buffer.datalength + 1 + datalen > (int)sizeof(wr_buffer.datastream))
	{
		wr_buffer.overflow = true;
		return;
	}

	wr_buffer.datastream[wr_buffer.datalength] = type;
	if (type == SIMPLE_VAR_TYPE_STRING)
	{
		wr_buffer.datastream[wr_buffer.datalength + 1] = strlen;
		memcpy(&wr_buffer.datastream[wr_buffer.datalength + 2], dataptr, strlen);
	}
	else if (datalen > 0)
	{
		memcpy(&wr_buffer.datastream[wr_buffer.datalength + 1], dataptr, datalen);
	}

	wr_buffer.datalength += 1 + datalen;
	wr_buffer.nextctr++;
}

void simple_var_output_process_can_frame(const uint8_t *data, unsigned int len, void *user_ptr)
{
	const t_can_simple_var_packet *m = (const t_can_simple_var_packet*)data;

	// se il buffer precedente non è stato svuotato in tempo, reset forzato del buffer
	if (wr_buffer_full)
	{
		wr_buffer_full = false;
		wr_buffer.datalength = 0;
		wr_buffer.nextctr = 0;
		wr_buffer.overflow = false;
	}

	// se abbiamo perso messaggi, aggiungiamo i segnaposto
	while (wr_buffer.nextctr != m->ctr && !wr_buffer.overflow)
		add_to_buffer(SIMPLE_VAR_TYPE_NULL, NULL);

	if (m->type == SIMPLE_VAR_TYPE_FLUSH)
	{
		wr_buffer_full = true;
		return;
	}

	add_to_buffer(m->type, m->data);
}

void simple_var_output_relax()
{
	int data_to_send = -1;

	int current_cpu_ipl;
	SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7); // disattiva interrupt

	if (wr_buffer_full)
	{
		data_to_send = wr_buffer.datalength + 4;
		tempbuffer[0] = 0xff;
		tempbuffer[1] = 0xfd;
		tempbuffer[2] = wr_buffer.overflow;
		tempbuffer[3] = wr_buffer.datalength;
		if (wr_buffer.datalength > 0)
			memcpy(&tempbuffer[4], wr_buffer.datastream, wr_buffer.datalength);

		wr_buffer_full = false;
		wr_buffer.datalength = 0;
		wr_buffer.nextctr = 0;
		wr_buffer.overflow = false;
	}

	RESTORE_CPU_IPL(current_cpu_ipl); // riattiva interrupt

	if (data_to_send > 0)
	{
		uint8_t dest_addr[2] = { 0xFF, 0xFF };
		xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, tempbuffer, data_to_send);
	}
}
