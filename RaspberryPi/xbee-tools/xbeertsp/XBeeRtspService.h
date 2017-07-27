#ifndef XBEERTSP_SERVICE_H
#define XBEERTSP_SERVICE_H

#include "XBeeRtspJobManager.h"
#include "../xbeemux-core/Service.h"
#include "../xbeemux-core/TcpServer.h"

#include <map>

class XBeeRtspClientHandler;
class XBeeRtspEndpointInfo;

class XBeeRtspService : public Service
{
	public:
		XBeeRtspService(const XBeeAddress &robotAddress, unsigned int port);
		void run();

		XBeeRtspJobManager *getJobManager() const;
		XBee *getXBee() const { return (XBee*)&m_xbee; }
		XBeeRtspEndpointInfo *getEndpointInfo(unsigned int nodeId);

	private:
		XBee &m_xbee;
		const XBeeAddress m_robotAddress;
		TcpServer m_server;
		std::map<int, XBeeRtspClientHandler*> m_clients;
		XBeeRtspJobManager m_jobManager;
		std::map<int, XBeeRtspEndpointInfo*> m_endpoints;
};

#endif // XBEERTSP_SERVICE_H
