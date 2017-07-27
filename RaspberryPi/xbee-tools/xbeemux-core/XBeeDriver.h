#ifndef XBEEMUXCORE_XBEEDRIVER_H
#define XBEEMUXCORE_XBEEDRIVER_H

#include "XBee.h"

class XBeeDriver : public XBee
{
	public:
		XBeeDriver(const char *tty_dev);
		XBeeDriver(const char *tcp_host, unsigned int port);
		~XBeeDriver();

		void sendto(const void *buffer, size_t length, const XBeeAddress &dstaddr);
		ssize_t recvfrom(void *buffer, size_t maxlength, XBeeAddress *out_srcaddr = NULL);
		int fd() const { return xbee_fd; }

	private:
		bool readall(uint8_t *buffer, size_t length);

		int xbee_fd;

		// Invio
		uint8_t next_frame_id;
};

#endif // XBEEMUXCORE_XBEEDRIVER_H
