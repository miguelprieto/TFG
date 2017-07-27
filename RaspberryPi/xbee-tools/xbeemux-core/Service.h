#ifndef XBEEMUXCORE_SERVICE_H
#define XBEEMUXCORE_SERVICE_H

#include "MessageQueue.h"
#include "XBee.h"

#include <stddef.h>
#include <unistd.h>
#include <vector>

class Service
{
	friend int main(int argc, char * const *argv);

	public:
		Service();

		// Loop principale del servizio, viene eseguito su thread dedicato
		virtual void run() = 0;

		// Offriamo al servizio le funzionalità di XBee
		XBee *xbee() const { return (XBee*)&m_xbeeProxy; };

	private:
		void enqueueReceivedXBeeMessage(const XBeeAddress &addr, const std::vector<uint8_t> &data);
		MessageQueue m_incomingMessages;
		static MessageQueue m_outgoingMessages;

		struct XBeeProxy : public XBee
		{
			void sendto(const void *buffer, size_t length, const XBeeAddress &dstaddr);
			ssize_t recvfrom(void *buffer, size_t maxlength, XBeeAddress *out_srcaddr = NULL);
			int fd() const { return m_incomingMessages->fd(); }
			MessageQueue *m_incomingMessages, *m_outgoingMessages;
		} m_xbeeProxy;
};

class ServiceFactory
{
	public:
		ServiceFactory();
		virtual Service *createInstance() const = 0;

		static std::vector<Service*> startAllServices();

	private:
		ServiceFactory *m_nextFactory;
};

#endif // XBEEMUXCORE_SERVICE_H
