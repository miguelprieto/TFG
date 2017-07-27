#include "XBeeStdioMessage.h"

#include <cstring>

void XBeeStdioMessage::sendKeepAlive(XBee *xbee, const XBeeAddress &address, const std::vector<XBeeStdioEndpoint*> &endpoints)
{
	char keepAlive_packet[200];

	keepAlive_packet[0] = 0xFF;
	keepAlive_packet[1] = 0xFF;
	keepAlive_packet[2] = 0xFF;

	unsigned int num_devices = 0;
	for (unsigned int i = 0; i < endpoints.size(); i++)
	{
		if (endpoints[i]->is_connected())
		{
			keepAlive_packet[4 + 3*num_devices] = endpoints[i]->get_r_device_id();
			keepAlive_packet[4 + 3*num_devices + 1] = (endpoints[i]->get_last_seq_num() >> 8);
			keepAlive_packet[4 + 3*num_devices + 2] = endpoints[i]->get_last_seq_num();
			num_devices++;
		}
	}

	keepAlive_packet[3] = num_devices;

	if (num_devices != 0)
		xbee->sendto(keepAlive_packet, 4 + 3*num_devices, address);
}

void XBeeStdioMessage::recvfrom(XBee *xbee, const XBeeAddress &address, const std::vector<XBeeStdioEndpoint*> &endpoints)
{
	uint8_t data[128];
	XBeeAddress srcaddr;
	ssize_t msg_len = xbee->recvfrom(data, sizeof(data), &srcaddr);

	if (srcaddr == address && msg_len > 3 && data[0] == 0xFF && data[1] == 0xFF)
	{
		for (unsigned int i = 0; i < endpoints.size(); i++)
		{
			if (endpoints[i]->get_r_device_id() == data[2])
			{
				endpoints[i]->receivePacketFromRobot(data + 3, msg_len - 3);
				break;
			}

			if ((0x20 | endpoints[i]->get_r_device_id()) == data[2])
			{
				endpoints[i]->receiveAckFromRobot(((uint16_t)data[3] << 8) | data[4]);
				break;
			}
		}
	}
}

void XBeeStdioMessage::sendto(XBee *xbee, const XBeeAddress &address, const uint8_t id, const uint8_t *msg, const size_t msg_len, uint16_t seq_num)
{
	char buf[200];

	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = (id | 0x80);
	buf[3] = msg_len;
	memcpy(&buf[4], msg, msg_len);
	buf[4 + msg_len] = (seq_num >> 8);
	buf[4 + msg_len + 1] = seq_num;

	xbee->sendto(buf, 4 + msg_len + 2, address);
}

void XBeeStdioMessage::sendAck(XBee *xbee, const XBeeAddress &address, const uint8_t id, const uint16_t seq_num)
{
	char buf[7];
	
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFF;
	buf[3] = 1;
	buf[4] = id;
	buf[5] = (char) (seq_num >> 8);
	buf[6] = (char) seq_num;
	
	xbee->sendto(buf, 7, address);
}
