#include "../config.h"

#include "HexImage.h"
#include "NotifyStatus.h"
#include "Utils.h"
#include "../xbeemux-core/Utils.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <arpa/inet.h>
#include <err.h>
#include <unistd.h>
#include <sys/select.h>

using namespace std;

static int safeRead(int sock, void *data, size_t datalen);
static uint8_t readByte(int sock);
static void safeWrite(int sock, const void *data, size_t datalen);
static void sendByte(int sock, uint8_t data);
static void sendWord(int sock, uint16_t data);
static void sendDword(int sock, uint32_t data);
static void transmitProgramRequest(int sock, const HexImage &image, bool verifyAfterwards, bool resetAfterwards);
static void transmitVerifyRequest(int sock, const HexImage &image, bool resetAfterwards);
static void transmitResetRequest(int sock);
static bool receiveResponse(int sock);

int main(int argc, char * const *argv)
{
	bool doReprogram = false;
	bool doVerify = false;
	bool doReset = false;
	const char *deviceName = NULL;
	const char *serverHostAndPort = NULL;
	const char *hexFilename = NULL;
	int nodeId = -1;
	int c;

	while ((c = getopt(argc, argv, "d:hn:p:rvw")) != -1)
	{
		switch (c)
		{
			case 'd':
				deviceName = optarg;
				break;
			case 'n':
				nodeId = atoi(optarg);
				break;
			case 'p':
				serverHostAndPort = optarg;
				break;
			case 'r':
				doReset = true;
				break;
			case 'v':
				doVerify = true;
				break;
			case 'w':
				doReprogram = true;
				break;
			case 'h':
				cerr << "Uso: " << argv[0] << " -d DEVICENAME -p ipaddr:port -n node-id [-w] [-v] [-r] firmware.hex" << endl;
				cerr << "    -h                Mostra questo messaggio" << endl;
				cerr << "    -d                Tipo di PIC" << endl;
				cerr << "    -p ip-addr:port   IP e porta del server XBeeRtsp da contattare" << endl;
				cerr << "    -n node-id        ID del nodo da contattare (compreso tra 1 e 15)" << endl;
				cerr << "    -w                Programma firmware nel dispositivo" << endl;
				cerr << "    -v                Verifica firmware del dispositivo" << endl;
				cerr << "    -r                Reset del dispositivo" << endl;
				// fallback
			case '?':
				return c == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
			default:
				abort();
		}
	}

	const DeviceInfo *deviceInfo;
	if (deviceName == NULL)
	{
		errx(EXIT_FAILURE, "Nessun tipo di PIC specificato");
	}
	else if ((deviceInfo = getDeviceInfo(deviceName)) == NULL)
	{
		errx(EXIT_FAILURE, "Tipo di PIC non riconosciuto");
	}

	const bool firmwareRequired = (doVerify || doReprogram);
	if (firmwareRequired)
	{
		if (optind == argc - 1)
		{
			hexFilename = argv[optind];
		}
		else if (optind == argc)
		{
			errx(EXIT_FAILURE, "Nessun firmware specificato");
		}
		else
		{
			errx(EXIT_FAILURE, "Troppi parametri");
		}
	}
	else if (optind != argc)
	{
		errx(EXIT_FAILURE, "Troppi parametri");
	}

	if (!doReprogram && !doVerify && !doReset)
		errx(EXIT_FAILURE, "Nessuna azione selezionata");

	if (serverHostAndPort == NULL)
		errx(EXIT_FAILURE, "Utilizzare l'opzione -p per specificare il server XBeeRtsp da contattare");

	char *host = strdup(serverHostAndPort);
	char *colon = strchr(host, ':');
	if (colon == NULL)
		errx(EXIT_FAILURE, "Formato dell'opzione -p non valido");
	*colon = '\0';
	int port = atoi(colon + 1);

	if (nodeId == -1)
		errx(EXIT_FAILURE, "Utilizzare l'opzione -n per selezionare il nodo da contattare");

	HexImage image(hexFilename ?: "", deviceInfo);
	if ((doReprogram || doVerify) && !image.loadOk())
		errx(EXIT_FAILURE, "%s: Errore di caricamento", hexFilename);

	image.dropUntouchedSections();
	image.dropRtspKernelSections();

	cerr << nsbegin << "Connessione al server XBeeRtsp (" << host << ':' << port << ")..." << nsend;

	bool success = false;
	int sock = -1;

	try
	{
		if ((sock = connectTcp(host, port)) == -1)
			throw communication_error("Connessione non riuscita");

		cerr << nsbegin << "Connesso al server XBeeRtsp." << nsend;

		// Invia firma servizio
		sendByte(sock, 'R'); sendByte(sock, 'T'); sendByte(sock, 'S'); sendByte(sock, 'P');

		// Attendi firma servizio
		if (readByte(sock) != 'R' || readByte(sock) != 'T' || readByte(sock) != 'S' || readByte(sock) != 'P')
			throw communication_error("Rispsta del server XBeeRtsp non valida");

		cerr << nsbegin << "Trasmissione della richiesta..." << nsend;

		// Invia versione protocollo RTSP richiesta
		sendByte(sock, 16);

		// Invia numero di endpoint da programmare
		sendByte(sock, nodeId);

		// Invia nome device (tipo di PIC) da programmare
		safeWrite(sock, deviceName, strlen(deviceName) + 1);

		if (doReprogram)
			transmitProgramRequest(sock, image, doVerify, doReset);
		else if (doVerify)
			transmitVerifyRequest(sock, image, doReset);
		else // if (doReset)
			transmitResetRequest(sock);

		cerr << nsbegin << "Richiesta inviata." << nsend;

		success = receiveResponse(sock);
	}
	catch (const communication_error &ex)
	{
		cerr << endl << ex.what() << endl;
	}

	if (sock != -1)
		close(sock);

	free(host);
	return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int safeRead(int sock, void *data, size_t datalen)
{
	int r = read(sock, data, datalen);

	if (r == 0)
	{
		throw communication_error("Persa connessione al server XBeeRtsp");
	}
	else if (r == -1)
	{
		warn("read()");
		throw communication_error("Errore di ricezione da server XBeeRtsp");
	}

	return r;
}

uint8_t readByte(int sock)
{
	uint8_t temp;
	safeRead(sock, &temp, 1);
	return temp;
}

void safeWrite(int sock, const void *data, size_t datalen)
{
	if (write(sock, data, datalen) != datalen)
	{
		warn("write()");
		throw communication_error("Errore di trasmissione verso server XBeeRtsp");
	}
}

void sendByte(int sock, uint8_t data)
{
	safeWrite(sock, &data, 1);
}

void sendWord(int sock, uint16_t data)
{
	uint16_t databe = htons(data);
	safeWrite(sock, &databe, 2);
}

void sendDword(int sock, uint32_t data)
{
	uint32_t databe = htonl(data);
	safeWrite(sock, &databe, 4);
}

void transmitProgramRequest(int sock, const HexImage &image, bool verifyAfterwards, bool resetAfterwards)
{
	sendByte(sock, 0x10 | (verifyAfterwards ? 2 : 0) | (resetAfterwards ? 1 : 0));

	const vector<const HexImage::ProgramPage*> &programPages = image.getProgramPages();
	const vector<const HexImage::ConfigurationWord*> &configurationWords = image.getConfigurationWords();

	sendByte(sock, programPages.size());
	sendByte(sock, configurationWords.size());

	for (vector<const HexImage::ProgramPage*>::const_iterator it = programPages.begin();
		it != programPages.end(); ++it)
	{
		const vector<uint8_t> contents = (*it)->getPackedContents();

		sendDword(sock, (*it)->getBaseAddress());

		for (unsigned int i = 0; i < contents.size(); i++)
			sendByte(sock, contents[i]);

		sendWord(sock, crc16(contents));
	}

	for (vector<const HexImage::ConfigurationWord*>::const_iterator it = configurationWords.begin();
		it != configurationWords.end(); ++it)
	{
		sendDword(sock, (*it)->getAddress());
		sendWord(sock, (*it)->getValue());
	}
}

void transmitVerifyRequest(int sock, const HexImage &image, bool resetAfterwards)
{
	sendByte(sock, 0x20 | (resetAfterwards ? 1 : 0));

	const vector<const HexImage::ProgramPage*> &programPages = image.getProgramPages();
	const vector<const HexImage::ConfigurationWord*> &configurationWords = image.getConfigurationWords();

	sendByte(sock, programPages.size());
	sendByte(sock, configurationWords.size());

	for (vector<const HexImage::ProgramPage*>::const_iterator it = programPages.begin();
		it != programPages.end(); ++it)
	{
		sendDword(sock, (*it)->getBaseAddress());
		sendWord(sock, crc16((*it)->getPackedContents()));
	}

	for (vector<const HexImage::ConfigurationWord*>::const_iterator it = configurationWords.begin();
		it != configurationWords.end(); ++it)
	{
		sendDword(sock, (*it)->getAddress());
		sendWord(sock, (*it)->getValue());
	}
}

void transmitResetRequest(int sock)
{
	sendByte(sock, 0x30);
}

bool receiveResponse(int sock)
{
	char buffer[256];
	int r;

	int status = -1;
	while (status == -1 && (r = safeRead(sock, buffer, sizeof(buffer))) > 0)
	{
		if (buffer[r-1] == '\x80') // marcatore di fine output (job failure)
		{
			r--;
			status = 0; // failure
		}
		else if (buffer[r-1] == '\x81') // marcatore di fine output (job success)
		{
			r--;
			status = 1; // success
		}

		cerr.write(buffer, r);
	}

	// Invia ACK relativo a ricezione del risultato finale
	sendByte(sock, '!');

	// Il server risponde chiudendo la connessione. Attendiamo per un
	// massimo di tre secondi in modo da dare il tempo al server di
	// chiudere la connessione per primo
	fd_set select_fdset;
	FD_ZERO(&select_fdset);
	FD_SET(sock, &select_fdset);
	struct timeval timeout;
	timeout.tv_sec = 3;
	timeout.tv_usec = 0;
	select(sock + 1, &select_fdset, NULL, NULL, &timeout); // ignora risultato

	return status == 1; // true=success
}
