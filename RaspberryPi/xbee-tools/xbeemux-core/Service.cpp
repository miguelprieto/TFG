#include "Service.h"

#include <cstdlib>
#include <cstring>
#include <err.h>
#include <pthread.h>
#include <stddef.h>

static ServiceFactory *registeredFactories = NULL;

static void *serviceRunner(void *service)
{
	Service *s = (Service*)service;
	s->run();
	return NULL;
}

// Coda globale dei messaggi XBee in uscita
MessageQueue Service::m_outgoingMessages;

Service::Service()
{
	m_xbeeProxy.m_incomingMessages = &m_incomingMessages;
	m_xbeeProxy.m_outgoingMessages = &m_outgoingMessages;
}

void Service::enqueueReceivedXBeeMessage(const XBeeAddress &addr, const std::vector<uint8_t> &data)
{
	m_incomingMessages.enqueueMessage(addr, data);
}

void Service::XBeeProxy::sendto(const void *buffer, size_t length, const XBeeAddress &dest)
{
	std::vector<uint8_t> temp(length);
	memcpy(&temp[0], buffer, length);
	m_outgoingMessages->enqueueMessage(dest, temp);
}

ssize_t Service::XBeeProxy::recvfrom(void *buffer, size_t maxlength, XBeeAddress *out_srcaddr)
{
	XBeeAddress temp1;
	std::vector<uint8_t> temp2;
	m_incomingMessages->dequeueMessage(&temp1, &temp2);

	if (out_srcaddr) *out_srcaddr = temp1;
	memcpy(buffer, &temp2[0], std::min(temp2.size(), maxlength));

	return temp2.size();
}

ServiceFactory::ServiceFactory()
: m_nextFactory(registeredFactories)
{
	registeredFactories = this;
}

std::vector<Service*> ServiceFactory::startAllServices()
{
	std::vector<Service*> result;
	pthread_t dummy;
	pthread_attr_t attr;

	if (pthread_attr_init(&attr) != 0)
		err(EXIT_FAILURE, "pthread_attr_init() failed");
	if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0)
		err(EXIT_FAILURE, "pthread_attr_setdetachstate() failed");

	for (ServiceFactory *f = registeredFactories; f != NULL; f = f->m_nextFactory)
	{
		Service *s = f->createInstance();
		result.push_back(s);

		if (pthread_create(&dummy, &attr, serviceRunner, (void*)s) != 0)
			err(EXIT_FAILURE, "pthread_create() failed");
	}

	pthread_attr_destroy(&attr);

	return result;
}
