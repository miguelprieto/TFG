#include "../config.h"

#include "XBeeTapClientHandler.h"
#include "../xbeemux-core/Service.h"
#include "../xbeemux-core/TcpServer.h"
#include "../xbeemux-core/Utils.h"

#include <cstdio>
#include <map>
#include <sys/select.h>

class XBeeTapService : public Service
{
	public:
		XBeeTapService();
		void run();

	private:
		XBee &m_xbee;
		TcpServer m_server;
		std::map<int, XBeeTapClientHandler*> m_clients;
};

class XBeeTapServiceFactory : public ServiceFactory
{
	public:
		Service *createInstance() const
		{
			return new XBeeTapService();
		}
};
static XBeeTapServiceFactory factory;

XBeeTapService::XBeeTapService()
: m_xbee(*xbee()), m_server(CONFIG_XBEETAP_PORT)
{
}

void XBeeTapService::run()
{
	fd_set select_fdset;

	// Loop principale che accetta nuove connessioni, effettua read()
	// su connessioni già stabilite e inoltra i pacchetti xbee a tutti i
	// client connessi
	while (true)
	{
		FD_ZERO(&select_fdset);

		int maxfd = std::max(m_xbee.fd(), m_server.fd());
		FD_SET(m_xbee.fd(), &select_fdset);
		FD_SET(m_server.fd(), &select_fdset);

		for (std::map<int, XBeeTapClientHandler*>::iterator it = m_clients.begin();
			it != m_clients.end(); ++it)
		{
			FD_SET(it->first, &select_fdset);
			maxfd = std::max(maxfd, it->first);
		}

		int select_res = select(maxfd + 1, &select_fdset, NULL, NULL, NULL);
		if (select_res == -1)
		{
			perror("XBeeTapService - select");
			continue;
		}
		else if (select_res == 0)
		{
			continue;
		}

		if (FD_ISSET(m_xbee.fd(), &select_fdset))
		{
			// Pacchetto xbee ricevuto
			uint8_t data[128];
			XBeeAddress srcaddr;
			ssize_t msglen = m_xbee.recvfrom(data, sizeof(data), &srcaddr);

			if (msglen > 0)
			{
				for (std::map<int, XBeeTapClientHandler*>::iterator it = m_clients.begin();
					it != m_clients.end(); ++it)
				{
					it->second->sendMessage(srcaddr, data, msglen);
				}
			}
		}

		if (FD_ISSET(m_server.fd(), &select_fdset))
		{
			// Nuova connessione
			XBeeTapClientHandler *c = new XBeeTapClientHandler(&m_xbee, m_server.accept());
			m_clients.insert(std::pair<int, XBeeTapClientHandler*>(c->fd(), c));
		}

		std::vector<int> closedConnections;
		for (std::map<int, XBeeTapClientHandler*>::iterator it = m_clients.begin();
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
				it->second->receiveByte(buff);
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
