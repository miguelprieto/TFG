#include "TcpServer.h"

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

TcpServer::TcpServer(unsigned int port)
{
	m_sk = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_sk == -1)
		err(EXIT_FAILURE, "TcpServer - socket");

	const int reuseaddr_val = 1;
	if (setsockopt(m_sk, SOL_SOCKET, SO_REUSEADDR, (const int*)&reuseaddr_val, sizeof(int)) == -1)
		err(EXIT_FAILURE, "TcpServer - setsockopt(SO_REUSEADDR)");

	struct sockaddr_in localaddr;
	memset(&localaddr, 0, sizeof(struct sockaddr_in));
	localaddr.sin_family = AF_INET;
	localaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localaddr.sin_port = htons(port);

	if (bind(m_sk, (struct sockaddr*)&localaddr, sizeof(struct sockaddr_in)) == -1)
		err(EXIT_FAILURE, "TcpServer - bind(%u)", port);

	if (listen(m_sk, 1) == -1)
		err(EXIT_FAILURE, "TcpServer - listen");
}

TcpServer::~TcpServer()
{
	close(m_sk);
}

int TcpServer::accept()
{
	int new_socket;

	while ((new_socket = ::accept(m_sk, NULL, NULL)) == -1)
	{
		if (errno == EAGAIN && errno == EWOULDBLOCK)
			err(EXIT_FAILURE, "TcpServer - accept");
	}

	return new_socket;
}
