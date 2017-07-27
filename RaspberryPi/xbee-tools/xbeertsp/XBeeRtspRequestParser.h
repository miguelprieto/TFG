#ifndef XBEERTSP_REQUESTPARSER_H
#define XBEERTSP_REQUESTPARSER_H

#include <map>
#include <vector>
#include <stdint.h>

class DeviceInfo;
class XBeeRtspClientHandler;
class XBeeRtspJob;

class XBeeRtspRequestParser
{
	public:
		XBeeRtspRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo);
		virtual ~XBeeRtspRequestParser();

		virtual bool receiveByte(uint8_t data) = 0;
		virtual XBeeRtspJob *testDoneAndCreateJob() const = 0;

	protected:
		XBeeRtspClientHandler * const m_client;
		const unsigned int m_nodeId;
		const DeviceInfo *m_deviceInfo;
};

class XBeeRtspResetRequestParser : public XBeeRtspRequestParser
{
	public:
		XBeeRtspResetRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo);

		bool receiveByte(uint8_t data);
		XBeeRtspJob *testDoneAndCreateJob() const;

};

class XBeeRtspVerifyRequestParser : public XBeeRtspRequestParser
{
	public:
		XBeeRtspVerifyRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo, bool resetAfterwards);

		bool receiveByte(uint8_t data);
		XBeeRtspJob *testDoneAndCreateJob() const;

	private:
		bool m_resetAfterwards;
		std::map<uint32_t, uint16_t> m_pageMap; // indirizzo_pagina -> crc16
		std::map<uint32_t, uint16_t> m_configMap; // indirizzo_config_word -> valore
		unsigned int m_remainingPages, m_remainingConfigWords;

		enum
		{
			NUM_PAGES,
			NUM_CONFIGWORDS,
			PAGE,
			CRC,
			CONFIGWORD,
			VALUE,
			DONE
		} m_status;
		unsigned int m_offset;

		uint32_t m_addr;
		uint16_t m_value;
};

class XBeeRtspProgramRequestParser : public XBeeRtspRequestParser
{
	public:
		XBeeRtspProgramRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo, bool verifyAfterwards, bool resetAfterwards);

		bool receiveByte(uint8_t data);
		XBeeRtspJob *testDoneAndCreateJob() const;

	private:
		bool m_verifyAfterwards, m_resetAfterwards;
		std::map<uint32_t, std::vector<uint8_t> > m_pageMap; // indirizzo_pagina -> dati
		std::map<uint32_t, uint16_t> m_configMap; // indirizzo_config_word -> valore
		unsigned int m_remainingPages, m_remainingConfigWords;

		enum
		{
			NUM_PAGES,
			NUM_CONFIGWORDS,
			PAGE,
			DATA,
			CRC,
			CONFIGWORD,
			VALUE,
			DONE
		} m_status;
		unsigned int m_offset;

		uint32_t m_addr;
		uint16_t m_value;
		std::vector<uint8_t> m_buffer;
};

#endif // XBEERTSP_REQUESTPARSER_H
