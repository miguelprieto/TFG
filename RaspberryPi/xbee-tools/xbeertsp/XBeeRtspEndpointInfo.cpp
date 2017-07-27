#include "XBeeRtspEndpointInfo.h"
#include "Utils.h"

using namespace std;

XBeeRtspEndpointInfo::XBeeRtspEndpointInfo(const XBeeAddress &robotAddress, unsigned int nodeId)
: m_robotAddress(robotAddress), m_nodeId(nodeId), deviceInfo_present(false)
{
}

bool XBeeRtspEndpointInfo::setDeviceTypeInfo(uint16_t family_id, uint16_t device_id)
{
	if (!deviceInfo_present || deviceInfo_familyId != family_id || deviceInfo_deviceId != device_id)
	{
		deviceInfo_present = true;
		deviceInfo_familyId = family_id;
		deviceInfo_deviceId = device_id;
		m_pageMap.clear();
		return true;
	}
	else
	{
		return false;
	}
}

bool XBeeRtspEndpointInfo::getKnownPageContents(uint32_t page_address, std::vector<uint8_t> *out_contents) const
{
	map<uint32_t, pair<uint16_t, vector<uint8_t> > >::const_iterator it = m_pageMap.find(page_address);

	if (it == m_pageMap.end())
		return false;

	*out_contents = it->second.second;
	return true;
}

void XBeeRtspEndpointInfo::setKnownPageContents(uint32_t page_address, const vector<uint8_t> &contents)
{
	m_pageMap[page_address] = pair<uint16_t, vector<uint8_t> >(crc16(contents), contents);
}

void XBeeRtspEndpointInfo::unsetKnownPageContents(uint32_t page_address)
{
	map<uint32_t, pair<uint16_t, vector<uint8_t> > >::iterator it = m_pageMap.find(page_address);
	if (it != m_pageMap.end())
		m_pageMap.erase(it);
}

void XBeeRtspEndpointInfo::unsetAllUnlessCrcMatches(uint32_t page_address, uint16_t unless_crc)
{
	map<uint32_t, pair<uint16_t, vector<uint8_t> > >::iterator it = m_pageMap.find(page_address);

	if (it == m_pageMap.end())
		return;

	if (it->second.first != unless_crc)
		m_pageMap.clear();
}

map<uint32_t, uint16_t> XBeeRtspEndpointInfo::knownPageCRCs() const
{
	map<uint32_t, uint16_t> result;

	for (map<uint32_t, pair<uint16_t, vector<uint8_t> > >::const_iterator it = m_pageMap.begin();
		it != m_pageMap.end(); ++it)
	{
		result.insert(pair<uint32_t, uint16_t>(it->first, it->second.first));
	}

	return result;
}
