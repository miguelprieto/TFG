#ifndef XBEESTDIO_CLIENTHANDLER_H
#define XBEESTDIO_CLIENTHANDLER_H

#include <stdint.h>

class XBeeStdioEndpoint;

class XBeeStdioClientHandler
{
	public:
		XBeeStdioClientHandler(XBeeStdioEndpoint *endpoint, int sockfd);
		~XBeeStdioClientHandler();

		int fd() const { return m_connsock; }
		bool receiveTelnetByte(char data);
		void sendTelnetByte(char data);

	private:
		XBeeStdioEndpoint *m_endpoint;
		int m_connsock;
		int m_connSkipBytes;
		bool m_inSubnegotiation;
		bool m_skipIfNulOrLf;
};

#endif // XBEESTDIO_CLIENTHANDLER_H
