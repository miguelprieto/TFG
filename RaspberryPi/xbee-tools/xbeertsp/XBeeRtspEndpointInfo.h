#ifndef XBEERTSP_ENDPOINTINFO_H
#define XBEERTSP_ENDPOINTINFO_H

#include "../xbeemux-core/XBee.h"

#include <map>
#include <vector>

class XBeeRtspEndpointInfo
{
	public:
		XBeeRtspEndpointInfo(const XBeeAddress &robotAddress, unsigned int nodeId);

		const XBeeAddress &robotAddress() const { return m_robotAddress; }
		unsigned int nodeId() const { return m_nodeId; }

		bool setDeviceTypeInfo(uint16_t family_id, uint16_t device_id);

		bool getKnownPageContents(uint32_t page_address, std::vector<uint8_t> *out_contents) const;
		void setKnownPageContents(uint32_t page_address, const std::vector<uint8_t> &contents);
		void unsetKnownPageContents(uint32_t page_address);
		void unsetAllUnlessCrcMatches(uint32_t page_address, uint16_t unless_crc);
		std::map<uint32_t, uint16_t> knownPageCRCs() const;

	private:
		const XBeeAddress m_robotAddress;
		unsigned int m_nodeId;

		// Elenco delle pagine note e del relativo crc e contenuto
		std::map<uint32_t, std::pair<uint16_t, std::vector<uint8_t> > > m_pageMap;

		// Informazioni sul tipo di device
		bool deviceInfo_present;
		uint16_t deviceInfo_familyId, deviceInfo_deviceId;
};

#endif // XBEERTSP_ENDPOINT_H
