#include "XBeeRtspInterface.h"
#include "NotifyStatus.h"
#include "Utils.h"
#include "XBeeRtspEndpointInfo.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>

#include <sys/select.h>
#include <unistd.h>

using namespace std;

XBeeRtspInterface::XBeeRtspInterface(ostream &userout, XBee &xbee, XBeeRtspEndpointInfo *endpointInfo, const volatile bool *cancelled)
: userout(userout), xbee(xbee), endpointInfo(endpointInfo), cancelled(cancelled), seqnum(0)
{
}

void XBeeRtspInterface::checkCancelledFlag()
{
	if (*cancelled == true)
		throw communication_error("Operazione annullata");
}

vector<uint8_t> XBeeRtspInterface::sendAndWaitReply(const vector<uint8_t> &data, unsigned int max_retries, unsigned int retry_interval_ms)
{
	const XBeeAddress &xbee_addr = endpointInfo->robotAddress();

	vector<uint8_t> packet(3 + data.size());
	packet[0] = 0xFF;
	packet[1] = 0xFE;
	packet[2] = ++seqnum;
	memcpy(&packet[3], &data[0], data.size());

	fd_set select_fdset;

	struct timeval tv;
	tv.tv_sec = retry_interval_ms / 1000;
	tv.tv_usec = (retry_interval_ms % 1000) * 1000;
	xbee.sendto(&packet[0], packet.size(), xbee_addr);

	while (true)
	{
		FD_ZERO(&select_fdset);
		FD_SET(xbee.fd(), &select_fdset);

		checkCancelledFlag();
		int select_res = select(xbee.fd() + 1, &select_fdset, NULL, NULL, &tv);
		checkCancelledFlag();

		if (select_res == -1)
		{
			perror("select");
		}
		else if (select_res == 0)
		{
			if (max_retries-- != 0)
			{
				// Ritrasmissione
				tv.tv_sec = retry_interval_ms / 1000;
				tv.tv_usec = (retry_interval_ms % 1000) * 1000;
				xbee.sendto(&packet[0], packet.size(), xbee_addr);
			}
			else
			{
				// Timeout
				throw communication_error("Il coordinatore " + xbee_addr.addrString() + " non risponde");
			}
		}
		else if (FD_ISSET(xbee.fd(), &select_fdset))
		{
			uint8_t data[128];
			XBeeAddress srcaddr;
			ssize_t msglen = xbee.recvfrom(data, sizeof(data), &srcaddr);
			//for (int i = 0; i < msglen; i++) printf("<%02x>\n", data[i]);
			if (srcaddr == xbee_addr && data[0] == 0xFF && data[1] == 0xFE && data[2] == seqnum)
				return vector<uint8_t>(data, data + msglen);
		}
	}
}

void XBeeRtspInterface::selectTargetEndpoint(unsigned int deviceInfo_wordsPerPage)
{
	const XBeeAddress &xbee_addr = endpointInfo->robotAddress();
	const uint8_t node_id = endpointInfo->nodeId();

	vector<uint8_t> request;
	request.push_back(0x80);
	request.push_back(node_id);
	request.push_back((uint8_t)(deviceInfo_wordsPerPage >> 8));
	request.push_back((uint8_t)(deviceInfo_wordsPerPage >> 0));

	while (true)
	{
		userout << nsbegin << "Collegamento a coordinatore " << xbee_addr.addrString() << ", nodo " << (unsigned)node_id << "..." << nsend;

		const vector<uint8_t> reply = sendAndWaitReply(request, 10, 500);
		if (reply.size() == 4 && reply[3] == 0)
		{
			userout << nsbegin << "Coordinatore occupato, attendo e riprovo..." << nsend;
			sleep(1);
			checkCancelledFlag();
			sleep(1);
		}
		else if (reply.size() == 4 && reply[3] == 1)
		{
			throw communication_error("Connesso al coordinatore, ma il nodo non risponde");
		}
		else if (reply.size() == 7)
		{
			const uint16_t family = ((uint8_t)reply[3] << 8) | reply[4];
			const uint16_t dev_id = ((uint8_t)reply[5] << 8) | reply[6];

			userout << nsbegin << "Connesso al coordinatore " << xbee_addr.addrString() << ", nodo " << (unsigned)node_id << "." << nsendl;

			if (endpointInfo->setDeviceTypeInfo(family, dev_id))
				throw communication_error("Il tipo di PIC non corrisponde a quello indicato");

			break;
		}
		else
		{
			throw communication_error("Risposta non valida");
		}
	}
}

bool XBeeRtspInterface::verifyCrc(const map<uint32_t, uint16_t> &pageMap, const char *progressText, bool quiet)
{
	std::vector<uint32_t> pagesToVerify;
	std::vector<uint16_t> expectedChecksum;

	// Prepara elenco delle pagine da controllare e relativi
	// checksum attesi
	for (map<uint32_t, uint16_t>::const_iterator it = pageMap.begin();
		it != pageMap.end(); ++it)
	{
		pagesToVerify.push_back(it->first);
		expectedChecksum.push_back(it->second);
	}

	const vector<uint16_t> actualChecksum = queryCrcBigList(&pagesToVerify[0], pagesToVerify.size(), progressText);
	bool success = true;
	const streamsize oldW = userout.width();
	const char oldF = userout.fill('0');
	for (unsigned int i = 0; i < pagesToVerify.size(); i++)
	{
		// Se durante la verifica riscontriamo un contenuto diverso da
		// quello memorizzato in endpointInfo, invalidiamo endpointInfo
		endpointInfo->unsetAllUnlessCrcMatches(pagesToVerify[i], actualChecksum[i]);

		if (actualChecksum[i] != expectedChecksum[i])
		{
			if (!quiet)
			{
				userout << nsbegin << "Checksum mismatch su pagina 0x" << hex << setw(6) << pagesToVerify[i]
					<< ": atteso=0x" << setw(4) << expectedChecksum[i] << ", reale=0x" << setw(4) << actualChecksum[i] << dec << nsendl;
			}
			success = false;
		}
	}
	userout.width(oldW);
	userout.fill(oldF);

	if (success && !quiet)
		userout << nsbegin << "Verifica completata con successo." << nsendl;

	return success;
}

vector<uint16_t> XBeeRtspInterface::queryCrcBigList(const uint32_t *pageList, unsigned int pageCount, const char *progressText)
{
	vector<uint16_t> result;

	// Invia richieste CRC16 a blocchi di N pagine
	const unsigned int N = 20;
	for (unsigned int i = 0; i < pageCount; i += N)
	{
		userout << nsbegin << (i * 100 / pageCount) << "% - " << progressText << nsend;
		const unsigned int count = min(N, pageCount - i);
		vector<uint16_t> response = queryCrcSmallList(pageList + i, count);

		// Se abbiamo ricevuto un pacchetto con meno valori di quelli richiesti,
		// vuol dire che il device ha smesso di rispondere durante l'esecuzione
		// del comando
		if (response.size() != count)
			throw communication_error("Il device non risponde al coordinatore");

		for (unsigned int j = 0; j < response.size(); j++)
			result.push_back(response[j]);
	}

	return result;
}

vector<uint16_t> XBeeRtspInterface::queryCrcSmallList(const uint32_t *pageList, unsigned int pageCount)
{
	vector<uint8_t> request;
	request.push_back(0x81);
	request.push_back(pageCount);
	for (unsigned int i = 0; i < pageCount; i++)
	{
		request.push_back((uint8_t)(pageList[i] >> 24));
		request.push_back((uint8_t)(pageList[i] >> 16));
		request.push_back((uint8_t)(pageList[i] >> 8));
		request.push_back((uint8_t)(pageList[i] >> 0));
	}

	const vector<uint8_t> reply = sendAndWaitReply(request);
	vector<uint16_t> result;
	for (unsigned int i = 3; i < reply.size(); i += 2)
		result.push_back(((uint8_t)reply[i] << 8) | reply[i+1]);

	return result;
}

bool XBeeRtspInterface::programPage(uint32_t target_address, uint16_t compressionInfo,
	const vector<uint8_t> &data, uint16_t expectedCrc16,
	float beginPercent, float endPercent)
{
	// Invia dati in blocchi di al massimo N byte
	const size_t N = 91;
	const unsigned int numBlocks = (data.size() + N - 1) / N; // arrotonda per eccesso

	if (numBlocks == 1)
	{
		// Anche se c'è un solo blocco da trsmettere, dobbiamo comunque
		// trasmettere sia il messaggio begin che quello end
		userout << nsbegin << (int)beginPercent << "% - Programmazione pagina 0x" << hex
			<< target_address << dec << "..." << nsend;
		if (beginProgramPage(target_address, &data[0], data.size())
			!= pair<unsigned int, uint16_t>(data.size(), crc16(data)))
		{
			return false;
		}

		userout << nsbegin << (int)((beginPercent + endPercent) / 2) << "% - Programmazione pagina 0x" << hex
			<< target_address << dec << "..." << nsend;
		return endProgramPage(NULL, 0, expectedCrc16, compressionInfo);
	}
	else
	{
		const float blockIncrement = (endPercent - beginPercent) / numBlocks;

		const uint16_t partialCrc16 = crc16(vector<uint8_t>(data.begin(), data.begin() + N));
		userout << nsbegin << (int)beginPercent << "% - Programmazione pagina 0x" << hex
			<< target_address << dec << "..." << nsend;
		if (beginProgramPage(target_address, &data[0], N) !=
			std::pair<unsigned int, uint16_t>(N, partialCrc16))
		{
			return false;
		}

		// Blocchi intermedi
		for (unsigned int i = 1; i < numBlocks - 1; i++)
		{
			const uint16_t partialCrc16 = crc16(vector<uint8_t>(data.begin(), data.begin() + (i+1) * N));
			userout << nsbegin << (int)(beginPercent + blockIncrement*i) << "% - Programmazione pagina 0x" << hex
				<< target_address << dec << "..." << nsend;
			if (contProgramPage(&data[i * N], N) !=
				std::pair<unsigned int, uint16_t>((i+1) * N, partialCrc16))
			{
				return false;
			}
		}

		// Ultimo blocco
		userout << nsbegin << (int)(endPercent - blockIncrement) << "% - Programmazione pagina 0x" << hex
			<< target_address << dec << "..." << nsend;
		return endProgramPage(&data[(numBlocks - 1) * N], ((data.size() + N - 1) % N) + 1, expectedCrc16, compressionInfo);
	}

	return true;
}

pair<unsigned int, uint16_t> XBeeRtspInterface::beginProgramPage(uint32_t target_address, const uint8_t *data, unsigned int datalen)
{
	vector<uint8_t> request;
	request.push_back(0x82);
	request.push_back(datalen);
	for (unsigned int i = 0; i < datalen; i++)
		request.push_back(data[i]);
	request.push_back((uint8_t)(target_address >> 24));
	request.push_back((uint8_t)(target_address >> 16));
	request.push_back((uint8_t)(target_address >> 8));
	request.push_back((uint8_t)(target_address >> 0));

	const vector<uint8_t> reply = sendAndWaitReply(request);
	vector<uint16_t> result;
	for (unsigned int i = 3; i < 7; i += 2)
		result.push_back(((uint8_t)reply[i] << 8) | reply[i+1]);

	return pair<unsigned int, uint16_t>(result[0], result[1]);
}

pair<unsigned int, uint16_t> XBeeRtspInterface::contProgramPage(const uint8_t *data, unsigned int datalen)
{
	vector<uint8_t> request;
	request.push_back(0x83);
	request.push_back(datalen);
	for (unsigned int i = 0; i < datalen; i++)
		request.push_back(data[i]);

	const vector<uint8_t> reply = sendAndWaitReply(request);
	vector<uint16_t> result;
	for (unsigned int i = 3; i < 7; i += 2)
		result.push_back(((uint8_t)reply[i] << 8) | reply[i+1]);

	return pair<unsigned int, uint16_t>(result[0], result[1]);
}

bool XBeeRtspInterface::endProgramPage(const uint8_t *data, unsigned int datalen, uint16_t expectedCrc16, uint16_t compressionInfo)
{
	vector<uint8_t> request;
	request.push_back(0x84);
	request.push_back(datalen);
	for (unsigned int i = 0; i < datalen; i++)
		request.push_back(data[i]);
	request.push_back((uint8_t)(expectedCrc16 >> 8));
	request.push_back((uint8_t)(expectedCrc16 >> 0));
	request.push_back((uint8_t)(compressionInfo >> 8));
	request.push_back((uint8_t)(compressionInfo >> 0));

	const vector<uint8_t> reply = sendAndWaitReply(request);
	return reply[3];
}

void XBeeRtspInterface::reset()
{
	vector<uint8_t> request;
	request.push_back(0x85);

	userout << nsbegin << "Trasmissione della richiesta di reset..." << nsend;
	sendAndWaitReply(request);
	userout << nsbegin << "Reset completato." << nsendl;
}
