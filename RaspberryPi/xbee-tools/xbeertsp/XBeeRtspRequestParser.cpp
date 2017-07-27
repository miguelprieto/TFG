#include "XBeeRtspRequestParser.h"
#include "DeviceInfo.h"
#include "Utils.h"
#include "XBeeRtspJobs.h"

XBeeRtspRequestParser::XBeeRtspRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo)
: m_client(client), m_nodeId(nodeId), m_deviceInfo(deviceInfo)
{
}

XBeeRtspRequestParser::~XBeeRtspRequestParser()
{
}

XBeeRtspResetRequestParser::XBeeRtspResetRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo)
: XBeeRtspRequestParser(client, nodeId, deviceInfo)
{
}

bool XBeeRtspResetRequestParser::receiveByte(uint8_t data)
{
	return false;
}

XBeeRtspJob *XBeeRtspResetRequestParser::testDoneAndCreateJob() const
{
	return new XBeeRtspResetJob(m_client, m_nodeId, m_deviceInfo);
}

XBeeRtspVerifyRequestParser::XBeeRtspVerifyRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo, bool resetAfterwards)
: XBeeRtspRequestParser(client, nodeId, deviceInfo), m_resetAfterwards(resetAfterwards), m_status(NUM_PAGES)
{
}

bool XBeeRtspVerifyRequestParser::receiveByte(uint8_t data)
{
	switch (m_status)
	{
		case NUM_PAGES:
			m_remainingPages = data;
			m_status = NUM_CONFIGWORDS;
			break;
		case NUM_CONFIGWORDS:
			m_remainingConfigWords = data;
			if (m_remainingPages-- != 0)
			{
				m_status = PAGE;
				m_addr = 0;
				m_offset = 3;
			}
			else
			{
				m_status = DONE;
			}
			break;
		case PAGE:
			m_addr |= ((uint32_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				m_status = CRC;
				m_value = 0;
				m_offset = 1;
			}
			break;
		case CRC:
			m_value |= ((uint16_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				m_pageMap.insert(std::pair<uint32_t, uint16_t>(m_addr, m_value));

				if (m_remainingPages-- != 0)
				{
					m_status = PAGE;
					m_addr = 0;
					m_offset = 3;
				}
				else if (m_remainingConfigWords-- != 0)
				{
					m_status = CONFIGWORD;
					m_addr = 0;
					m_offset = 3;
				}
				else
				{
					m_status = DONE;
				}
			}
			break;
		case CONFIGWORD:
			m_addr |= ((uint32_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				m_status = VALUE;
				m_value = 0;
				m_offset = 1;
			}
			break;
		case VALUE:
			m_value |= ((uint16_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				m_configMap.insert(std::pair<uint32_t, uint16_t>(m_addr, m_value));

				if (m_remainingConfigWords-- != 0)
				{
					m_status = CONFIGWORD;
					m_addr = 0;
					m_offset = 3;
				}
				else
				{
					m_status = DONE;
				}
			}
			break;
		case DONE:
			return false;
	}

	return true;
}

XBeeRtspJob *XBeeRtspVerifyRequestParser::testDoneAndCreateJob() const
{
	if (m_status != DONE)
		return NULL;

	return new XBeeRtspVerifyJob(m_client, m_nodeId, m_deviceInfo, m_resetAfterwards, m_pageMap, m_configMap);
}

XBeeRtspProgramRequestParser::XBeeRtspProgramRequestParser(XBeeRtspClientHandler *client, unsigned int nodeId, const DeviceInfo *deviceInfo, bool verifyAfterwards, bool resetAfterwards)
: XBeeRtspRequestParser(client, nodeId, deviceInfo), m_verifyAfterwards(verifyAfterwards), m_resetAfterwards(resetAfterwards), m_status(NUM_PAGES)
{
}

bool XBeeRtspProgramRequestParser::receiveByte(uint8_t data)
{
	switch (m_status)
	{
		case NUM_PAGES:
			m_remainingPages = data;
			m_status = NUM_CONFIGWORDS;
			break;
		case NUM_CONFIGWORDS:
			m_remainingConfigWords = data;
			if (m_remainingPages-- != 0)
			{
				m_status = PAGE;
				m_addr = 0;
				m_offset = 3;
			}
			else
			{
				m_status = DONE;
			}
			break;
		case PAGE:
			m_addr |= ((uint32_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				m_status = DATA;
				m_buffer.clear();
				m_offset = m_deviceInfo->wordsPerPage * 3 - 1;
			}
			break;
		case DATA:
			m_buffer.push_back(data);
			if (m_offset-- == 0)
			{
				m_status = CRC;
				m_value = 0;
				m_offset = 1;
			}
			break;
		case CRC:
			m_value |= ((uint16_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				if (crc16(m_buffer) != m_value)
					return false;
				m_pageMap.insert(std::pair<uint32_t, std::vector<uint8_t> >(m_addr, m_buffer));

				if (m_remainingPages-- != 0)
				{
					m_status = PAGE;
					m_addr = 0;
					m_offset = 3;
				}
				else if (m_remainingConfigWords-- != 0)
				{
					m_status = CONFIGWORD;
					m_addr = 0;
					m_offset = 3;
				}
				else
				{
					m_status = DONE;
				}
			}
			break;
		case CONFIGWORD:
			m_addr |= ((uint32_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				m_status = VALUE;
				m_value = 0;
				m_offset = 1;
			}
			break;
		case VALUE:
			m_value |= ((uint16_t)data) << (m_offset * 8);
			if (m_offset-- == 0)
			{
				m_configMap.insert(std::pair<uint32_t, uint16_t>(m_addr, m_value));

				if (m_remainingConfigWords-- != 0)
				{
					m_status = CONFIGWORD;
					m_addr = 0;
					m_offset = 3;
				}
				else
				{
					m_status = DONE;
				}
			}
			break;
		case DONE:
			return false;
	}

	return true;
}

XBeeRtspJob *XBeeRtspProgramRequestParser::testDoneAndCreateJob() const
{
	if (m_status != DONE)
		return NULL;

	return new XBeeRtspProgramJob(m_client, m_nodeId, m_deviceInfo, m_verifyAfterwards, m_resetAfterwards, m_pageMap, m_configMap);
}
