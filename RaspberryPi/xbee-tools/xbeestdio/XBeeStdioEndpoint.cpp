#include "XBeeStdioClientHandler.h"
#include "XBeeStdioEndpoint.h"
#include "XBeeStdioMessage.h"
#include "../xbeemux-core/Utils.h"

#include <numeric>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <err.h>

XBeeStdioEndpoint::XBeeStdioEndpoint(XBee *xbee, const XBeeAddress &robotAddress, uint8_t endpoint_id, unsigned int telnetport)
: m_xbee(xbee), m_robotAddress(robotAddress), m_server(telnetport),
  r_device_id(endpoint_id), last_input_seq_num(0), next_buffer_seqnum(0),
  send_timeout_counter(0)
{
}

XBeeStdioEndpoint::~XBeeStdioEndpoint()
{
}

void XBeeStdioEndpoint::sendByteToRobot(uint8_t byte)
{
	output_buffer.push_back(byte);
	next_buffer_seqnum++;
	send_timeout_counter = 0;
	sendOutputBuffer();
}

void XBeeStdioEndpoint::handleNewConnection()
{
	m_clients.insert(new XBeeStdioClientHandler(this, m_server.accept()));
}

void XBeeStdioEndpoint::handleClientEvent(XBeeStdioClientHandler *client)
{
	char buff;
	int r = read(client->fd(), &buff, 1);

	if (r == 0)
	{
		// Connessione terminata
		delete client;
		m_clients.erase(client);
	}
	else if (r == 1)
	{
		// Dati in arrivo
		if (!client->receiveTelnetByte(buff))
		{
			// Richiesta di chiusura della connessione
			delete client;
			m_clients.erase(client);
		}
	}
	else
	{
		perror("read()");
	}
}

void XBeeStdioEndpoint::receivePacketFromRobot(const uint8_t *data, size_t datalen)
{
	uint16_t seq_num = (data[datalen - 2] << 8) | data[datalen - 1];

	uint16_t seq_num_diff = seq_num - last_input_seq_num;
	if(seq_num_diff > datalen - 2)
	{
		warnx("Dati persi: da %u a %u\n", last_input_seq_num, seq_num);
		sendStringToTelnetClients(data, datalen - 2);
	}
	else
	{
		sendStringToTelnetClients(data + datalen - 2 - seq_num_diff, seq_num_diff);
	}

	last_input_seq_num = seq_num;

	XBeeStdioMessage::sendAck(m_xbee, m_robotAddress, r_device_id, last_input_seq_num);
}

void XBeeStdioEndpoint::sendStringToTelnetClients(const uint8_t *str, unsigned int len)
{
	for (std::set<XBeeStdioClientHandler*>::iterator it = m_clients.begin();
		it != m_clients.end(); ++it)
	{
		for (unsigned int i = 0; i < len; i++)
			(*it)->sendTelnetByte(str[i]);
	}
}

void XBeeStdioEndpoint::sendOutputBuffer()
{
	const size_t max_data_len = 16;
	const size_t data_len = std::min(max_data_len, output_buffer.size());
	const uint16_t seq_num = next_buffer_seqnum - (output_buffer.size() - data_len);
	XBeeStdioMessage::sendto(m_xbee, m_robotAddress, r_device_id, &output_buffer[0], data_len, seq_num);
}

void XBeeStdioEndpoint::clockInterval()
{
	if (!output_buffer.empty())
	{
		if (send_timeout_counter++ == 8)
			output_buffer.clear(); // timeout, smettiamo di ritrasmettere
		else
			sendOutputBuffer(); // ritrasmissione
	}
}

void XBeeStdioEndpoint::receiveAckFromRobot(uint16_t seqnum)
{
	const uint16_t buff_start_seqnum = next_buffer_seqnum - output_buffer.size();
	const uint16_t acked_offset = seqnum - buff_start_seqnum;

	if (acked_offset > output_buffer.size())
	{
		// ACK fuori sequenza, lo ignoriamo e svuotiamo il buffer di
		// output (poiché probabilmente è stato riavviato il robot)
		output_buffer.clear();
	}
	else
	{
		output_buffer.erase(output_buffer.begin(), output_buffer.begin() + acked_offset);
	}
}
