#ifndef XBEEMUXCORE_MESSAGEQUEUE_H
#define XBEEMUXCORE_MESSAGEQUEUE_H

#include "XBeeAddress.h"

#include <vector>

// Coda thread-safe di messaggi XBee
class MessageQueue
{
	public:
		MessageQueue();
		~MessageQueue();

		int fd() const;

		void enqueueMessage(const XBeeAddress &addr, const std::vector<uint8_t> &data);
		void dequeueMessage(XBeeAddress *addr, std::vector<uint8_t> *data);

	private:
		int pipe_in, pipe_out;
};

#endif // XBEEMUXCORE_MESSAGEQUEUE_P_H
