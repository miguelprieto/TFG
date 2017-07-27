#ifndef XBEESTDIO_ENDPOINT_H
#define XBEESTDIO_ENDPOINT_H

#include "../xbeemux-core/TcpServer.h"
#include "../xbeemux-core/XBee.h"

#include <set>
#include <vector>

class XBeeStdioClientHandler;

// Esiste una istanza per ciascuna scheda a cui è possibile collegarsi
class XBeeStdioEndpoint
{
	public:
		XBeeStdioEndpoint(XBee *xbee, const XBeeAddress &robotAddress, uint8_t endpoint_id, unsigned int telnetport);
		~XBeeStdioEndpoint();

		int fd() const { return m_server.fd(); }
		void sendByteToRobot(uint8_t byte);
		void receivePacketFromRobot(const uint8_t *data, size_t datalen);

		void handleNewConnection();
		void handleClientEvent(XBeeStdioClientHandler *client);

		bool is_connected() const { return !m_clients.empty(); }
		std::set<XBeeStdioClientHandler*> get_clients() const { return m_clients; }
		uint16_t get_last_seq_num() const { return last_input_seq_num; }
		uint8_t get_r_device_id() const { return r_device_id; }

		void clockInterval();
		void receiveAckFromRobot(uint16_t seqnum);

	private:
		void sendStringToTelnetClients(const uint8_t *str, unsigned int len);
		void sendOutputBuffer();

		XBee *m_xbee;
		XBeeAddress m_robotAddress;
		TcpServer m_server;
		std::set<XBeeStdioClientHandler*> m_clients;
		uint16_t last_input_seq_num;
		uint8_t r_device_id;

		std::vector<uint8_t> output_buffer;
		uint16_t next_buffer_seqnum;
		unsigned int send_timeout_counter;
};

#endif // XBEESTDIO_ENDPOINT_H
