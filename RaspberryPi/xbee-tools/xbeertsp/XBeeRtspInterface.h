#ifndef XBEERTSP_INTERFACE_H
#define XBEERTSP_INTERFACE_H

#include "../xbeemux-core/XBee.h"

#include <map>
#include <ostream>
#include <vector>
#include <stdint.h>

class XBeeRtspEndpointInfo;

#define COMPRESSIONINFO_COMPRESSED_DIFF(rotation)	uint16_t(rotation)
#define COMPRESSIONINFO_COMPRESSED_DIRECT		4000
#define COMPRESSIONINFO_UNCOMPRESSED_DIRECT		4001

class XBeeRtspInterface
{
	public:
		XBeeRtspInterface(std::ostream &userout, XBee &xbee, XBeeRtspEndpointInfo *endpointInfo, const volatile bool *cancelled);

		// Ottiene il controllo esclusivo delle funzionalità di programmazione
		// offerte dal coordinatore e pone il nodo in modalità programmazione
		void selectTargetEndpoint(unsigned int deviceInfo_wordsPerPage);

		// Interroga il device per verificare la corrispondenza dei CRC
		// per ciascuna delle coppie (page_base_address, crc) indicate
		bool verifyCrc(const std::map<uint32_t, uint16_t> &pageMap, const char *progressText = "Verifica CRC...", bool quiet = false);

		// Programma una pagina. compressionInfo può essere uno dei valori
		// COMPRESSIONINFO_*
		bool programPage(uint32_t target_address, uint16_t compressionInfo,
			const std::vector<uint8_t> &data, uint16_t expectedCrc16,
			float beginPercent, float endPercent);

		// Invia reset
		void reset();

	private:
		void checkCancelledFlag();
		std::vector<uint8_t> sendAndWaitReply(const std::vector<uint8_t> &data, unsigned int max_retries = 10, unsigned int retry_interval_ms = 800);

		std::vector<uint16_t> queryCrcBigList(const uint32_t *pageList, unsigned int pageCount, const char *progressText);
		std::vector<uint16_t> queryCrcSmallList(const uint32_t *pageList, unsigned int pageCount);

		// Trasmissione di segmenti di pagine da programmare al coordinatore
		// Le funzioni beginProgramPage e contProgramPage restituiscono
		// la quantità e il CRC dei byte inviati fino a quel momento.
		// La programmazione del device avviene con la endProgramPage.
		std::pair<unsigned int, uint16_t> beginProgramPage(uint32_t target_address, const uint8_t *data, unsigned int datalen);
		std::pair<unsigned int, uint16_t> contProgramPage(const uint8_t *data, unsigned int datalen);
		bool endProgramPage(const uint8_t *data, unsigned int datalen, uint16_t expectedCrc16, uint16_t compressionInfo);

		std::ostream &userout;
		XBee &xbee;
		XBeeRtspEndpointInfo *endpointInfo;
		const volatile bool *cancelled;

		uint8_t seqnum;
};

#endif // XBEERTSP_INTERFACE_H
