#ifndef XBEETAP_CLIENTHANDLER_H
#define XBEETAP_CLIENTHANDLER_H

#include "../xbeemux-core/XBee.h"

#include <vector>

// Offriamo a ciascun client tramite TCP un modulo XBee emulato
class XBeeTapClientHandler
{
	public:
		XBeeTapClientHandler(XBee *xbee, int sk);
		~XBeeTapClientHandler();

		void sendMessage(const XBeeAddress &sender, const uint8_t *data, size_t datalen);
		void receiveByte(uint8_t data);
		int fd() const { return m_sk; }

	private:
		XBee *m_xbee;
		int m_sk;

		void handleFrame(uint8_t cmdId, const uint8_t *cmdData, unsigned int cmdLen);
		void injectFrame(uint8_t cmdId, const uint8_t *cmdData, unsigned int cmdLen);
		void injectTxStatus(uint8_t frameId);

		enum
		{
			DELIMITER,
			LENGTH_MSB,
			LENGTH_LSB,
			DATA,
			CHECKSUM
		} m_decodeState;
		uint16_t m_decodeLength;
		std::vector<uint8_t> m_decodeData;
};

#endif // XBEETAP_CLIENTHANDLER_H
