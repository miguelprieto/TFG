#include "MessageQueue.h"
#include "Utils.h"

#include <cstdio>
#include <cstdlib>
#include <err.h>
#include <unistd.h>

MessageQueue::MessageQueue()
{
	int p[2];

	if (pipe(p) != 0)
		err(EXIT_FAILURE, "pipe()");

	pipe_out = p[0];
	pipe_in = p[1];
}

MessageQueue::~MessageQueue()
{
	close(pipe_in);

	// Scarta elementi nel buffer non consumati
	while (true)
	{
		void *buff[2];
		int r;

		if ((r = read(pipe_out, buff, sizeof(buff))) != sizeof(buff))
		{
			if (r == 0)
				return; // raggiunta fine del buffer
			else
				err(EXIT_FAILURE, "~MessageQueue - read()");
		}

		delete (XBeeAddress*)buff[0];
		delete (std::vector<uint8_t>*)buff[1];
	}

	close(pipe_out);
}

int MessageQueue::fd() const
{
	return pipe_out;
}

void MessageQueue::enqueueMessage(const XBeeAddress &addr, const std::vector<uint8_t> &data)
{
	XBeeAddress *addrCopy = new XBeeAddress(addr);
	std::vector<uint8_t> *dataCopy = new std::vector<uint8_t>(data);
	void *buff[2] = { (void*)addrCopy, (void*)dataCopy };

	if (write(pipe_in, buff, sizeof(buff)) != sizeof(buff))
		err(EXIT_FAILURE, "enqueueMessage - write()");
}

void MessageQueue::dequeueMessage(XBeeAddress *addr, std::vector<uint8_t> *data)
{
	void *buff[2];

	if (read(pipe_out, buff, sizeof(buff)) != sizeof(buff))
		err(EXIT_FAILURE, "dequeueMessage - read()");

	*addr = *(XBeeAddress*)buff[0];
	*data = *(std::vector<uint8_t>*)buff[1];

	delete (XBeeAddress*)buff[0];
	delete (std::vector<uint8_t>*)buff[1];
}
