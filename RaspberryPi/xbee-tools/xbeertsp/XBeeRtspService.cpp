#include "../config.h"
#include "XBeeRtspService.h"
#include "XBeeRtspClientHandler.h"
#include "XBeeRtspEndpointInfo.h"
#include "../xbeemux-core/Utils.h"

#include <cstdio>
#include <sys/select.h>

#define FACTORY_FOR_ROBOT(robotname) \
namespace { struct Factory ## robotname : public ServiceFactory \
{ \
	Service *createInstance() const \
	{ \
		return new XBeeRtspService(CONFIG_XBEEADDRESS_ ## robotname, \
			CONFIG_XBEERTSP_ ## robotname ## _PORT); \
	} \
}; \
Factory ## robotname factory ## robotname; }

FACTORY_FOR_ROBOT(GRANDE)
FACTORY_FOR_ROBOT(PICCOLO)

XBeeRtspService::XBeeRtspService(const XBeeAddress &robotAddress, unsigned int port)
: m_xbee(*xbee()), m_robotAddress(robotAddress), m_server(port)
{
}

void XBeeRtspService::run()
{
	fd_set select_fdset;

	// Loop principale che accetta nuove connessioni, effettua read()
	// su connessioni già stabilite e inoltra i pacchetti xbee all'eventuale
	// job in esecuzione
	while (true)
	{
		FD_ZERO(&select_fdset);

		int maxfd = std::max(m_xbee.fd(), m_server.fd());
		FD_SET(m_xbee.fd(), &select_fdset);
		FD_SET(m_server.fd(), &select_fdset);
		FD_SET(m_jobManager.fd(), &select_fdset);

		for (std::map<int, XBeeRtspClientHandler*>::iterator it = m_clients.begin();
			it != m_clients.end(); ++it)
		{
			FD_SET(it->first, &select_fdset);
			maxfd = std::max(maxfd, it->first);
		}

		int select_res = select(maxfd + 1, &select_fdset, NULL, NULL, NULL);
		if (select_res == -1)
		{
			perror("XBeeRtspService - select");
			continue;
		}
		else if (select_res == 0)
		{
			continue;
		}

		if (FD_ISSET(m_jobManager.fd(), &select_fdset))
		{
			m_jobManager.handleJobCompleted();
		}

		if (FD_ISSET(m_xbee.fd(), &select_fdset))
		{
			// Pacchetto xbee ricevuto
			uint8_t data[128];
			XBeeAddress srcaddr;
			ssize_t msglen = m_xbee.recvfrom(data, sizeof(data), &srcaddr);

			if (msglen > 0 && m_jobManager.m_runningJob != NULL)
			{
				// Inoltra messaggio al job in esecuzione
				const std::vector<uint8_t> contents(data, data + msglen);
				m_jobManager.m_runningJob->m_incomingMessages.enqueueMessage(srcaddr, contents);
			}
		}

		if (FD_ISSET(m_server.fd(), &select_fdset))
		{
			// Nuova connessione
			XBeeRtspClientHandler *c = new XBeeRtspClientHandler(this, m_server.accept());
			m_clients.insert(std::pair<int, XBeeRtspClientHandler*>(c->fd(), c));
		}

		std::vector<int> closedConnections;
		for (std::map<int, XBeeRtspClientHandler*>::iterator it = m_clients.begin();
			it != m_clients.end(); ++it)
		{
			if (!FD_ISSET(it->first, &select_fdset))
				continue;

			char buff;
			int r = read(it->first, &buff, 1);

			if (r == 0)
			{
				// Connessione terminata
				closedConnections.push_back(it->first);
			}
			else if (r == 1)
			{
				// Dati in arrivo
				if (!it->second->receiveByte(buff))
					closedConnections.push_back(it->first);
			}
			else
			{
				perror("read()");
			}
		}

		// Chiudi connessioni (deve essere fatto fuori dal loop
		// precedente per non modificare m_clients durante l'iterazione
		// su di esso)
		for (std::vector<int>::iterator it = closedConnections.begin();
			it != closedConnections.end(); ++it)
		{
			delete m_clients[*it];
			m_clients.erase(*it);
		}
	}
}

XBeeRtspJobManager *XBeeRtspService::getJobManager() const
{
	return (XBeeRtspJobManager*)&m_jobManager;
}

XBeeRtspEndpointInfo *XBeeRtspService::getEndpointInfo(unsigned int nodeId)
{
	std::map<int, XBeeRtspEndpointInfo*>::iterator it = m_endpoints.find(nodeId);

	if (it == m_endpoints.end())
	{
		XBeeRtspEndpointInfo *r = new XBeeRtspEndpointInfo(m_robotAddress, nodeId);
		m_endpoints.insert(std::pair<int, XBeeRtspEndpointInfo*>(nodeId, r));
		return r;
	}
	else
	{
		return it->second;
	}
}
