#ifndef XBEEMUXCORE_XBEEADDRESS_H
#define XBEEMUXCORE_XBEEADDRESS_H

#include <stdint.h>
#include <string>

class XBeeAddress
{
	public:
		XBeeAddress()
		: sixtyfour(false)
		{
			addr[0] = addr[1] = 0;
		}

		// 16 bit address
		XBeeAddress(uint8_t b0, uint8_t b1)
		: sixtyfour(false)
		{
			addr[0] = b0; addr[1] = b1;
		}

		// 64 bit address
		XBeeAddress(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
		: sixtyfour(true)
		{
			addr[0] = b0; addr[1] = b1; addr[2] = b2; addr[3] = b3;
			addr[4] = b4; addr[5] = b5; addr[6] = b6; addr[7] = b7;
		}

		bool operator==(const XBeeAddress &other) const;

		static XBeeAddress fromByteArray(const uint8_t *data, size_t len_bytes);

		// getters
		const uint8_t *getAddress() const { return addr; }
		bool is64bit() const { return sixtyfour; }
		std::string addrString() const;

		static const XBeeAddress broadcast;

	private:
		uint8_t addr[8];
		bool sixtyfour;
};

#endif // XBEEMUXCORE_XBEEADDRESS_H
