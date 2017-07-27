#ifndef XBEEMUXCORE_XBEE_H
#define XBEEMUXCORE_XBEE_H

#include "XBeeAddress.h"

#include <stddef.h>
#include <unistd.h>

class XBee
{
	public:
		XBee();
		~XBee();

		virtual void sendto(const void *buffer, size_t length, const XBeeAddress &dstaddr) = 0;
		virtual ssize_t recvfrom(void *buffer, size_t maxlength, XBeeAddress *out_srcaddr = NULL) = 0;
		virtual int fd() const = 0;
};

#endif // XBEEMUXCORE_XBEE_H
