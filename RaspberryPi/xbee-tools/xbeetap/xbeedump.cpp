#include "../config.h"
#include "../xbeemux-core/XBeeDriver.h"

#include <cstdio>
#include <cstdlib>
#include <err.h>
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

	XBee *xbee;
	if (xbee_port == -1)
		xbee = new XBeeDriver(xbee_file_or_host);
	else
		xbee = new XBeeDriver(xbee_file_or_host, xbee_port);

	while (1)
	{
		uint8_t data[128];
		XBeeAddress srcaddr;
		ssize_t msglen = xbee->recvfrom(data, sizeof(data), &srcaddr);

		if (msglen > 0)
		{
			printf("Message from %s:", srcaddr.addrString().c_str());
			if (msglen == 0)
			{
				printf(" (empty)\n");
			}
			else
			{
				for (unsigned int i = 0; i < msglen; i++)
					printf(" %02x", data[i]);
				printf("\n");
			}
		}
	}

	delete xbee;
	return EXIT_SUCCESS;
}
