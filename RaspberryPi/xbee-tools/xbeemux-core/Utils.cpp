#include "Utils.h"

#include <cstdlib>
#include <cstring>
#include <arpa/inet.h>
#include <err.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

void setNonBlockingFlag(int fd, bool newValue)
{
	const int old_fl = fcntl(fd, F_GETFL);
	if (old_fl == -1)
		err(EXIT_FAILURE, "fcntl(F_GETFL)");

	int new_fl;
	if (newValue)
		new_fl = old_fl | O_NONBLOCK;
	else
		new_fl = old_fl & ~O_NONBLOCK;

	if (fcntl(fd, F_SETFL, new_fl) == -1)
		err(EXIT_FAILURE, "fcntl(F_SETFL, O_NONBLOCK)");
}

void setTcpNoDelayFlag(int sockfd, bool newValue)
{
	int optval = newValue ? 1 : 0;
	socklen_t optlen = sizeof(optval);
	if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &optval, optlen) == -1)
		err(EXIT_FAILURE, "setsockopt(IPPROTO_TCP, TCP_NODELAY)");
}

void setTcpKeepAlive(int sockfd, int maxProbes, int secondsBeforeFirstProbe, int secondsBetweenProbes)
{
	int optval = 1;
	socklen_t optlen = sizeof(optval);
	if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0)
		err(EXIT_FAILURE, "setsockopt(SOL_SOCKET, SO_KEEPALIVE)");

	optval = maxProbes;
	if (setsockopt(sockfd, SOL_TCP, TCP_KEEPCNT, &optval, optlen) < 0)
		err(EXIT_FAILURE, "setsockopt(SOL_TCP, TCP_KEEPCNT)");

	optval = secondsBeforeFirstProbe;
	if (setsockopt(sockfd, SOL_TCP, TCP_KEEPIDLE, &optval, optlen) < 0)
		err(EXIT_FAILURE, "setsockopt(SOL_TCP, TCP_KEEPIDLE)");

	optval = secondsBetweenProbes;
	if (setsockopt(sockfd, SOL_TCP, TCP_KEEPINTVL, &optval, optlen) < 0)
		err(EXIT_FAILURE, "setsockopt(SOL_TCP, TCP_KEEPINTVL)");
}

int connectTcp(const char *host, int port)
{
	int sk = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sk == -1)
		err(EXIT_FAILURE, "socket()");

	struct sockaddr_in remoteaddr;
	memset(&remoteaddr, 0, sizeof(struct sockaddr_in));
	remoteaddr.sin_family = AF_INET;
	if (inet_aton(host, &remoteaddr.sin_addr) == 0)
	{
		close(sk);
		return -1;
	}
	remoteaddr.sin_port = htons(port);

	if (connect(sk, (struct sockaddr*)&remoteaddr, sizeof(struct sockaddr_in)) == -1)
	{
		close(sk);
		return -1;
	}

	return sk;
}
