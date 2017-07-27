#ifndef XBEERTSP_JOBS_H
#define XBEERTSP_JOBS_H

#include "XBeeRtspInterface.h"
#include "XBeeRtspJobManager.h"

#include <map>
#include <vector>
#include <stdint.h>

class DeviceInfo;
class XBeeRtspEndpointInfo;

class XBeeRtspResetJob : public XBeeRtspJob
{
	public:
		XBeeRtspResetJob(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo);

	private:
		void run();

		XBeeRtspEndpointInfo *m_endpointInfo;
		const DeviceInfo *m_deviceInfo;
		XBeeRtspInterface m_iface;
};

class XBeeRtspVerifyJob : public XBeeRtspJob
{
	public:
		XBeeRtspVerifyJob(XBeeRtspClientHandler *client,
			unsigned int nodeId, const DeviceInfo *deviceInfo,
			bool resetAfterwards,
			const std::map<uint32_t, uint16_t> &pageMap,
			const std::map<uint32_t, uint16_t> &configMap);

	private:
		void run();

		XBeeRtspEndpointInfo *m_endpointInfo;
		const DeviceInfo *m_deviceInfo;
		XBeeRtspInterface m_iface;

		bool m_resetAfterwards;
		std::map<uint32_t, uint16_t> m_pageMap; // indirizzo_pagina -> crc16
		std::map<uint32_t, uint16_t> m_configMap; // indirizzo_config_word -> valore
};

class XBeeRtspProgramJob : public XBeeRtspJob
{
	public:
		XBeeRtspProgramJob(XBeeRtspClientHandler *client,
			unsigned int nodeId, const DeviceInfo *deviceInfo,
			bool verifyAfterwards,
			bool resetAfterwards,
			const std::map<uint32_t, std::vector<uint8_t> > &pageMap,
			const std::map<uint32_t, uint16_t> &configMap);

	private:
		void run();

		XBeeRtspEndpointInfo *m_endpointInfo;
		const DeviceInfo *m_deviceInfo;
		XBeeRtspInterface m_iface;

		bool m_verifyAfterwards;
		bool m_resetAfterwards;
		std::map<uint32_t, std::vector<uint8_t> > m_pageMap; // indirizzo_pagina -> dati
		std::map<uint32_t, uint16_t> m_configMap; // indirizzo_config_word -> valore
};

#endif // XBEERTSP_JOBS_H
