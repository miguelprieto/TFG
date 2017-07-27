#include "XBeeTapClientHandler.h"
#include "../xbeemux-core/Utils.h"

#include <numeric>
#include <cstdio>
#include <cstdlib>
#include <cstring>

XBeeTapClientHandler::XBeeTapClientHandler(XBee *xbee, int sk)
: m_xbee(xbee), m_sk(sk)
{
	setTcpKeepAlive(sk, 10, 5, 3);
	m_decodeState = DELIMITER;
}

XBeeTapClientHandler::~XBeeTapClientHandler()
{
	close(m_sk);
}

void XBeeTapClientHandler::receiveByte(uint8_t data)
{
	switch (m_decodeState)
	{
		case DELIMITER:
			if (data == 0x7e)
				m_decodeState = LENGTH_MSB;
			break;
		case LENGTH_MSB:
			m_decodeLength = (uint16_t)data << 8;
			m_decodeState = LENGTH_LSB;
			break;
		case LENGTH_LSB:
			m_decodeLength |= data;
			m_decodeData.resize(0);
			m_decodeData.reserve(m_decodeLength);
			if (m_decodeLength != 0)
				m_decodeState = DATA;
			else
				m_decodeState = CHECKSUM;
			break;
		case DATA:
			m_decodeData.push_back(data);
			if (m_decodeLength == m_decodeData.size())
				m_decodeState = CHECKSUM;
			break;
		case CHECKSUM:
		{
			if (m_decodeLength >= 1 &&
					0 == (uint8_t)(std::accumulate(m_decodeData.begin(), m_decodeData.end(), 1 + data)))
				handleFrame(m_decodeData[0], &m_decodeData[1], m_decodeLength - 1);
			m_decodeState = DELIMITER;
			break;
		}
	}
}

void XBeeTapClientHandler::handleFrame(uint8_t cmdId, const uint8_t *cmdData, unsigned int cmdLen)
{
	switch (cmdId)
	{
		case 0x00:
		{
			unsigned int messageLen = cmdLen - 10;
			std::vector<uint8_t> message(cmdData + 10, cmdData + 10 + messageLen);
			XBeeAddress dest = XBeeAddress::fromByteArray(cmdData + 1, 8);
			m_xbee->sendto(&message[0], messageLen, dest);

			if (cmdData[0] != 0)
				injectTxStatus(cmdData[0]);
		}
		case 0x01:
		{
			unsigned int messageLen = cmdLen - 4;
			std::vector<uint8_t> message(cmdData + 4, cmdData + 4 + messageLen);
			XBeeAddress dest = XBeeAddress::fromByteArray(cmdData + 1, 2);
			m_xbee->sendto(&message[0], messageLen, dest);

			if (cmdData[0] != 0)
				injectTxStatus(cmdData[0]);
		}
		default:
			break;
	}
}

void XBeeTapClientHandler::injectFrame(uint8_t cmdId, const uint8_t *cmdData, unsigned int cmdLen)
{
	unsigned int fullLen = 1 + cmdLen;
	uint8_t *packet = new uint8_t[3 + fullLen + 1];

	packet[0] = 0x7e;
	packet[1] = (uint8_t)(fullLen >> 8);
	packet[2] = (uint8_t)fullLen;
	packet[3] = cmdId;
	memcpy(&packet[4], cmdData, cmdLen);
	packet[3 + fullLen] = ~std::accumulate(packet + 3, packet + 3 + fullLen, 0);

	if (write(m_sk, packet, 3 + fullLen + 1) != 3 + fullLen + 1)
		perror("write()");

	delete packet;
}

void XBeeTapClientHandler::injectTxStatus(uint8_t frameId)
{
	uint8_t payload[] = { frameId, 0 /* success */ };
	injectFrame(0x89, payload, 2);
}

void XBeeTapClientHandler::sendMessage(const XBeeAddress &sender, const uint8_t *data, size_t datalen)
{
	if (sender.is64bit())
	{
		uint8_t *frame = new uint8_t[10 + datalen];
		memcpy(frame, sender.getAddress(), 8);
		frame[8] = 0; // rssi
		frame[9] = 0; // flags
		memcpy(frame + 10, data, datalen);
		injectFrame(0x80, frame, 10 + datalen);
		delete frame;
	}
	else
	{
		uint8_t *frame = new uint8_t[4 + datalen];
		memcpy(frame, sender.getAddress(), 2);
		frame[2] = 0; // rssi
		frame[3] = 0; // flags
		memcpy(frame + 4, data, datalen);
		injectFrame(0x81, frame, 4 + datalen);
		delete frame;
	}
}
