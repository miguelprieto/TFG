#include "XBeeRtspJobs.h"
#include "DeviceInfo.h"
#include "NotifyStatus.h"
#include "Utils.h"
#include "XBeeRtspClientHandler.h"
#include "XBeeRtspEndpointInfo.h"
#include "XBeeRtspService.h"

#include <cstdio>
#include <unistd.h>

using namespace std;

XBeeRtspResetJob::XBeeRtspResetJob(XBeeRtspClientHandler *client,
	unsigned int nodeId, const DeviceInfo *deviceInfo)
: XBeeRtspJob(client), m_endpointInfo(client->getOwner()->getEndpointInfo(nodeId)),
  m_deviceInfo(deviceInfo), m_iface(userout, xbee, m_endpointInfo, &cancelled)
{
}

void XBeeRtspResetJob::run()
{
	try
	{
		m_endpointInfo->setDeviceTypeInfo(m_deviceInfo->family_id, m_deviceInfo->device_id);

		m_iface.selectTargetEndpoint(m_deviceInfo->wordsPerPage);

		m_iface.reset();
	}
	catch (const communication_error &ex)
	{
		userout << endl << ex.what() << endl;
		return;
	}

	userout << nsbegin << "Successo!" << nsendl;
	setSuccessFlag();
}

XBeeRtspVerifyJob::XBeeRtspVerifyJob(XBeeRtspClientHandler *client,
	unsigned int nodeId, const DeviceInfo *deviceInfo,
	bool resetAfterwards,
	const map<uint32_t, uint16_t> &pageMap,
	const map<uint32_t, uint16_t> &configMap)
: XBeeRtspJob(client), m_endpointInfo(client->getOwner()->getEndpointInfo(nodeId)),
  m_deviceInfo(deviceInfo), m_iface(userout, xbee, m_endpointInfo, &cancelled),
  m_resetAfterwards(resetAfterwards), m_pageMap(pageMap), m_configMap(configMap)
{
}

void XBeeRtspVerifyJob::run()
{
	try
	{
		m_endpointInfo->setDeviceTypeInfo(m_deviceInfo->family_id, m_deviceInfo->device_id);

		m_iface.selectTargetEndpoint(m_deviceInfo->wordsPerPage);

		if (!m_iface.verifyCrc(m_pageMap))
			return; // crc mismatch

		if (m_resetAfterwards)
			m_iface.reset();
	}
	catch (const communication_error &ex)
	{
		userout << endl << ex.what() << endl;
		return;
	}

	userout << nsbegin << "Successo!" << nsendl;
	setSuccessFlag();
}

XBeeRtspProgramJob::XBeeRtspProgramJob(XBeeRtspClientHandler *client,
	unsigned int nodeId, const DeviceInfo *deviceInfo,
	bool verifyAfterwards, bool resetAfterwards,
	const map<uint32_t, vector<uint8_t> > &pageMap,
	const map<uint32_t, uint16_t> &configMap)
: XBeeRtspJob(client), m_endpointInfo(client->getOwner()->getEndpointInfo(nodeId)),
  m_deviceInfo(deviceInfo), m_iface(userout, xbee, m_endpointInfo, &cancelled),
  m_verifyAfterwards(verifyAfterwards), m_resetAfterwards(resetAfterwards),
  m_pageMap(pageMap), m_configMap(configMap)
{
}

void XBeeRtspProgramJob::run()
{
	try
	{
		m_endpointInfo->setDeviceTypeInfo(m_deviceInfo->family_id, m_deviceInfo->device_id);

		// Dati per verifica: pagina -> crc
		map<uint32_t, uint16_t> crcMap;

		// Dati per programmazione non basata su diff: pagina -> (compressed_flag, data)
		map<uint32_t, pair<bool, vector<uint8_t> > > directMap;

		// Dati per programmazione basata su diff:
		set<uint32_t> unmodifiedMap; // pagine non modificate
		map<uint32_t, pair<unsigned int, vector<uint8_t> > > diffMap; // pagina -> deltaInfo

		userout << nsbegin << "Analisi del firmware in corso..." << nsend;
		unsigned int uncompressedSize = 0;
		for (map<uint32_t, vector<uint8_t> >::iterator it = m_pageMap.begin();
			it != m_pageMap.end(); ++it)
		{
			const vector<uint8_t> comprData = compressLZO(it->second);
			crcMap.insert(pair<uint32_t, uint16_t>(it->first, crc16(it->second)));

			uncompressedSize += it->second.size();

			// Se i dati compressi sono più lunghi di quelli non
			// compressi, usiamo direttamente quelli non compressi
			if (comprData.size() >= it->second.size())
			{
				directMap.insert(pair<uint32_t, pair<bool, vector<uint8_t> > >(it->first,
					pair<bool, vector<uint8_t> >(false, it->second)));
			}
			else
			{
				directMap.insert(pair<uint32_t, pair<bool, vector<uint8_t> > >(it->first,
					pair<bool, vector<uint8_t> >(true, comprData)));
			}

			// Se il contenuto attuale della pagina è noto, calcola diff compresso
			vector<uint8_t> oldContents;
			if (m_endpointInfo->getKnownPageContents(it->first, &oldContents))
			{
				pair<unsigned int, vector<uint8_t> > deltaInfo;
				if (compressDiff(oldContents, it->second, &deltaInfo))
					unmodifiedMap.insert(it->first); // il nuovo contenuto è identico al vecchio
				else if (deltaInfo.second.size() < min(comprData.size(), it->second.size()))
					diffMap.insert(pair<uint32_t, pair<unsigned int, vector<uint8_t> > >(it->first, deltaInfo));
			}
		}

		m_iface.selectTargetEndpoint(m_deviceInfo->wordsPerPage);

		// Se stiamo cercando di sfruttare i diff, dobbiamo prima verificare
		// che la condizione di partenza corrisponda a quella attesa
		if (diffMap.size() != 0 || unmodifiedMap.size() != 0)
		{
			if (!m_iface.verifyCrc(m_endpointInfo->knownPageCRCs(), "Verifica della condizione iniziale...", true))
			{
				// Verifica fallita, non usiamo i delta!
				diffMap.clear();
			}
			else
			{
				// Ci fidiamo dei delta, quindi non
				// riprogrammiamo le pagine identiche
				for (set<uint32_t>::iterator it = unmodifiedMap.begin();
					it != unmodifiedMap.end(); ++it)
				{
					directMap.erase(*it); // eliminiamo pagina da lista della pagine da programmare
				}
			}
		}

		// Calcolo del progresso corrispondente alla fine della
		// programmazione di ciascuna pagina
		unsigned int totalBytes = 0;
		map<uint32_t, unsigned int> progressMap;
		for (map<uint32_t, pair<bool, vector<uint8_t> > >::iterator it = directMap.begin();
			it != directMap.end(); ++it)
		{
			if (diffMap.find(it->first) == diffMap.end())
				totalBytes += it->second.second.size();
			else
				totalBytes += diffMap[it->first].second.size();

			progressMap.insert(pair<uint32_t, unsigned int>(it->first, totalBytes));
		}

		unsigned int progressBytes = 0;
		for (map<uint32_t, pair<bool, vector<uint8_t> > >::iterator it = directMap.begin();
			it != directMap.end(); ++it)
		{
			// Calcolo delle percentuali di progresso corrispondenti
			// all'inizio e alla fine della programmazione della pagina
			const float beginPercent = progressBytes * 100.f / totalBytes;
			progressBytes = progressMap[it->first];
			const float endPercent = progressBytes * 100.f / totalBytes;

			m_endpointInfo->unsetKnownPageContents(it->first);

			while (!m_iface.programPage(it->first,
				// compressionInfo: (ammontare rotazione in caso di diff, altrimenti
				// una delle due costanti in caso di dati diretti)
				diffMap.find(it->first) != diffMap.end() ?
					COMPRESSIONINFO_COMPRESSED_DIFF(diffMap[it->first].first) :
					(it->second.first ? COMPRESSIONINFO_COMPRESSED_DIRECT :
						COMPRESSIONINFO_UNCOMPRESSED_DIRECT),
				// data: (diff oppure dati diretti)
				diffMap.find(it->first) != diffMap.end() ? diffMap[it->first].second : it->second.second,
				crcMap[it->first], beginPercent, endPercent))
			{
				userout << endl << "Errore durante programmazione pagina 0x" << hex << it->first
					<< ", riprovo..." << dec << endl;

				// In caso di fallimento del diff, riproviamo con i dati diretti
				diffMap.erase(it->first);
			}

			m_endpointInfo->setKnownPageContents(it->first, m_pageMap[it->first]);
		}

		userout << nsbegin << "Programmazione completata." << nsendl;


		if (m_verifyAfterwards && !m_iface.verifyCrc(crcMap))
			return; // crc mismatch

		if (m_resetAfterwards)
			m_iface.reset();
	}
	catch (const communication_error &ex)
	{
		userout << endl << ex.what() << endl;
		return;
	}

	userout << nsbegin << "Successo!" << nsendl;
	setSuccessFlag();
}
