#include "../config.h"

#include "XBeeStdioClientHandler.h"
#include "XBeeStdioEndpoint.h"
#include "XBeeStdioMessage.h"
#include "../xbeemux-core/Service.h"
#include "../xbeemux-core/Utils.h"

#include <cstdio>
#include <map>
#include <sys/select.h>

class XBeeStdioService : public Service
{
	public:
		XBeeStdioService(const XBeeAddress &robotAddress, unsigned int baseport,
			const uint8_t *endpoints, unsigned int endpoints_count);
		void run();

	private:
		XBee &m_xbee;
		XBeeAddress m_robotAddress;
		std::vector<XBeeStdioEndpoint*> m_endpoints;
		int tickDivider;
};

#define FACTORY_FOR_ROBOT(robotname) \
namespace { struct Factory ## robotname : public ServiceFactory \
{ \
	Service *createInstance() const \
	{ \
		const uint8_t endpoints[] = CONFIG_STDIO_ ## robotname ## _ENDPOINTS; \
		return new XBeeStdioService(CONFIG_XBEEADDRESS_ ## robotname, \
			CONFIG_STDIO_ ## robotname ## _BASEPORT, \
			endpoints, sizeof(endpoints)); \
	} \
}; \
Factory ## robotname factory ## robotname; }

FACTORY_FOR_ROBOT(GRANDE)
FACTORY_FOR_ROBOT(PICCOLO)

XBeeStdioService::XBeeStdioService(const XBeeAddress &robotAddress, unsigned int baseport,
	const uint8_t *endpoints, unsigned int endpoints_count)
: m_xbee(*xbee()), m_robotAddress(robotAddress), tickDivider(0)
{
	for (unsigned int i = 0; i < endpoints_count; i++)
	{
		m_endpoints.push_back(new XBeeStdioEndpoint(&m_xbee, robotAddress,
			endpoints[i], baseport + endpoints[i]));
	}
}

void XBeeStdioService::run()
{
	fd_set select_fdset;

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 100000;

	// Loop principale che accetta nuove connessioni, effettua read()
	// su connessioni già stabilite e inoltra i pacchetti xbee a tutti i
	// client connessi
	while (true)
	{
		FD_ZERO(&select_fdset);

		int maxfd = m_xbee.fd();
		FD_SET(m_xbee.fd(), &select_fdset);

		for (std::vector<XBeeStdioEndpoint*>::iterator it_e = m_endpoints.begin();
			it_e != m_endpoints.end(); ++it_e)
		{
			std::set<XBeeStdioClientHandler*> clients = (*it_e)->get_clients();
			for (std::set<XBeeStdioClientHandler*>::iterator it_c = clients.begin();
				it_c != clients.end(); ++it_c)
			{
				FD_SET((*it_c)->fd(), &select_fdset);
				maxfd = std::max(maxfd, (*it_c)->fd());
			}

			FD_SET((*it_e)->fd(), &select_fdset);
			maxfd = std::max(maxfd, (*it_e)->fd());
		}

		int select_res = select(maxfd + 1, &select_fdset, NULL, NULL, &timeout);
		if (select_res == -1)
		{
			perror("XBeeStdioService - select");
			continue;
		}
		else if (select_res == 0) // timeout
		{
			timeout.tv_sec = 0;
			timeout.tv_usec = 100000;

			switch (tickDivider++)
			{
				case 1:
					XBeeStdioMessage::sendKeepAlive(&m_xbee, m_robotAddress, m_endpoints);
					break;
				case 0:
				case 2:
				case 4:
				case 6:
				case 8:
					for (std::vector<XBeeStdioEndpoint*>::iterator it = m_endpoints.begin();
						it != m_endpoints.end(); ++it)
					{
						(*it)->clockInterval();
					}
					break;
			}

			if (tickDivider == 10)
				tickDivider = 0;
			continue;
		}

		if (FD_ISSET(m_xbee.fd(), &select_fdset))
		{
			// Pacchetto xbee ricevuto
			XBeeStdioMessage::recvfrom(&m_xbee, m_robotAddress, m_endpoints);
		}

		for (std::vector<XBeeStdioEndpoint*>::iterator it_e = m_endpoints.begin();
			it_e != m_endpoints.end(); ++it_e)
		{
			// Dati ricevuti dai client telnet
			std::set<XBeeStdioClientHandler*> clients = (*it_e)->get_clients();
			for (std::set<XBeeStdioClientHandler*>::iterator it_c = clients.begin();
				it_c != clients.end(); ++it_c)
			{
				if (FD_ISSET((*it_c)->fd(), &select_fdset))
					(*it_e)->handleClientEvent(*it_c);
			}

			// Nuove connessioni in ingresso
			if (FD_ISSET((*it_e)->fd(), &select_fdset))
				(*it_e)->handleNewConnection();
		}
	}
}
