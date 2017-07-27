#include "XBeeStdioClientHandler.h"
#include "XBeeStdioEndpoint.h"
#include "../xbeemux-core/Utils.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <arpa/inet.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

XBeeStdioClientHandler::XBeeStdioClientHandler(XBeeStdioEndpoint *endpoint, int sockfd)
: m_endpoint(endpoint), m_connsock(sockfd), m_connSkipBytes(0),
  m_inSubnegotiation(false), m_skipIfNulOrLf(false)
{
	setTcpKeepAlive(m_connsock, 10, 5, 3);
	setTcpNoDelayFlag(m_connsock);

	// IAC WONT LINEMODE IAC WILL ECHO
	write(m_connsock, "\377\375\042\377\373\001", 6);
}

XBeeStdioClientHandler::~XBeeStdioClientHandler()
{
	close(m_connsock);
}

bool XBeeStdioClientHandler::receiveTelnetByte(char data)
{
	if (m_inSubnegotiation)
	{
		// tutto ciò che viene trasmesso durante una
		// subnegotiation non va inoltrato al PIC.
		// IAC SE è la sequenza di terminazione della
		// subnegotiation
		switch (data)
		{
			case '\377': // IAC
				m_connSkipBytes = 1;
				break;
			case '\360': // SE
				if (m_connSkipBytes == 1)
					m_inSubnegotiation = false;
				m_connSkipBytes = 0;
				break;
			default:
				m_connSkipBytes = 0;
				break;
		}
	}
	else if (m_connSkipBytes == 0)
	{
		switch (data)
		{
			case '\377': // IAC
				m_connSkipBytes = 2;
				break;
			case '\177': // DEL (0x7f)
				m_endpoint->sendByteToRobot('\b');
				break;
			case '\003': // Ctrl-C
				return false;
			case '\r': // "CR NUL" or "CR LF" (see RFC 854, page 11)
				m_skipIfNulOrLf = true;
				m_endpoint->sendByteToRobot('\r');
				break;
			default:
				if (m_skipIfNulOrLf)
				{
					if (data != '\0' && data != '\n')
						m_endpoint->sendByteToRobot(data);
					m_skipIfNulOrLf = false;
				}
				else
				{
					m_endpoint->sendByteToRobot(data);
				}
				break;
		}
	}
	else
	{
		// Se il byte precedente era IAC e riceviamo
		// SB entriamo in subnegotiation; se il byte
		// precedente era IAC e non riceviamo SB,
		// scartiamo anche il byte successivo
		if (m_connSkipBytes-- == 2 && data == '\372') // SB
		{
			m_connSkipBytes = 0;
			m_inSubnegotiation = true;
		}
	}

	return true;
}

void XBeeStdioClientHandler::sendTelnetByte(char data)
{
	if (write(m_connsock, &data, 1) != 1)
		perror("write()");
}
