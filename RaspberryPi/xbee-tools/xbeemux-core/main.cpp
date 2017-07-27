#include "../config.h"

#include "Service.h"
#include "XBeeDriver.h"

#include <cstdio>
#include <cstdlib>
#include <err.h>
#include <signal.h>
#include <sys/select.h>
#include <unistd.h>

int main(int argc, char * const *argv)
{
	const char *xbee_file_or_host;
	int xbee_port = -1;
	int c;

	while ((c = getopt(argc, argv, "h")) != -1)
	{
		switch (c)
		{
			case 'h':
				fprintf(stderr, "Uso: %s (usa device predefinito %s)\n", argv[0], CONFIG_XBEE_PATH);
				fprintf(stderr, "     %s /path/to/xbee/device\n", argv[0]);
				fprintf(stderr, "     %s <emulator-xbee-ip> <emulator-xbee-port>\n", argv[0]);
				// fallback
			case '?':
				return c == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
			default:
				abort();
		}
	}

	if (optind == argc - 1)
	{
		xbee_file_or_host = argv[optind];
	}
	else if (optind == argc - 2)
	{
		xbee_file_or_host = argv[optind];
		xbee_port = atoi(argv[optind + 1]);
	}
	else if (optind == argc)
	{
		xbee_file_or_host = CONFIG_XBEE_PATH;
	}
	else
	{
		errx(EXIT_FAILURE, "Percorso device XBee non valido");
	}

	// Disattiva SIGPIPE in caso di write su socket verso client non più connessi
	signal(SIGPIPE, SIG_IGN);

	XBee *xbee;
	if (xbee_port == -1)
		xbee = new XBeeDriver(xbee_file_or_host);
	else
		xbee = new XBeeDriver(xbee_file_or_host, xbee_port);

	// Avvia servizi, ciascuno su thread dedicato
	std::vector<Service*> services = ServiceFactory::startAllServices();

	fd_set select_fdset;

	// Ciclo infinito che inoltra i messaggi da XBee ai servizi e viceversa
	while (true)
	{
		FD_ZERO(&select_fdset);
		FD_SET(xbee->fd(), &select_fdset);
		FD_SET(Service::m_outgoingMessages.fd(), &select_fdset);

		int select_res = select(std::max(xbee->fd(), Service::m_outgoingMessages.fd()) + 1,
			&select_fdset, NULL, NULL, NULL);
		if (select_res == -1)
		{
			perror("main - select");
			continue;
		}
		else if (select_res == 0)
		{
			continue;
		}
		else if (FD_ISSET(xbee->fd(), &select_fdset)) // Messaggio in ingresso da XBee
		{
			uint8_t data[128];
			XBeeAddress srcaddr;
			ssize_t msglen = xbee->recvfrom(data, sizeof(data), &srcaddr);

			if (msglen > 0)
			{
				// Inoltra messaggio ricevuto a tutti i servizi
				const std::vector<uint8_t> contents(data, data + msglen);
				for (std::vector<Service*>::iterator it = services.begin();
					it != services.end(); ++it)
				{
					(*it)->enqueueReceivedXBeeMessage(srcaddr, contents);
				}
			}
		}
		else if (FD_ISSET(Service::m_outgoingMessages.fd(), &select_fdset))
		{
			XBeeAddress temp1;
			std::vector<uint8_t> temp2;
			Service::m_outgoingMessages.dequeueMessage(&temp1, &temp2);
			xbee->sendto(&temp2[0], temp2.size(), temp1);
		}
	}

	delete xbee;
	return EXIT_SUCCESS;
}
