#ifndef XBEESTDIO_MESSAGE_H
#define XBEESTDIO_MESSAGE_H

#include <vector>

#include "XBeeStdioEndpoint.h"
#include "../xbeemux-core/XBee.h"

namespace XBeeStdioMessage
{
	void sendKeepAlive(XBee *xbee, const XBeeAddress &address,  const std::vector<XBeeStdioEndpoint*> &endpoints);
	void recvfrom(XBee *xbee, const XBeeAddress &address, const std::vector<XBeeStdioEndpoint*> &endpoints);
	void sendto(XBee *xbee, const XBeeAddress &address, const uint8_t id, const uint8_t *msg, const size_t msg_len, uint16_t seqnum);
	void sendAck(XBee *xbee, const XBeeAddress &address, const uint8_t id, const uint16_t seq_num);
};

#endif // XBEESTDIO_MESSAGE_H
