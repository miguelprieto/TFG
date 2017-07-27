#include "XBeeDriver.h"
#include "Utils.h"
#include "XBeeAddress.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <numeric>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

//#define XBEE_TX_STATUS_ENABLE
#define XBEE_NO_RTSCTS

#define XBEE_START_DELIMITER                    0x7e
#define XBEE_TX64_REQUEST                       0x00
#define XBEE_TX16_REQUEST                       0x01
#define XBEE_RX64_INDICATOR                     0x80
#define XBEE_RX16_INDICATOR                     0x81
#define XBEE_TX_STATUS                          0x89

// --- DELETE ME!
#define XBEE_AT_COMMAND                         0x08
#define XBEE_AT_COMMAND_QUEUE_REGISTER_VALUE    0x09
#define XBEE_REMOTE_AT_COMMAND                  0x17
#define XBEE_DIO_ADC_RX64_INDICATOR             0x82
#define XBEE_DIO_ADC_RX16_INDICATOR             0x83
#define XBEE_AT_COMMAND_RESPONSE                0x88
#define XBEE_MODEM_STATUS                       0x8a
#define XBEE_REMOTE_COMMAND_RESPONSE            0x97
#define XBEE_RX_INDICATOR_MAX_RX_DATA_LENGTH             100
#define XBEE_DIO_ADC_RX_INDICATOR_MAX_RX_DATA_LENGTH     100
#define XBEE_AT_COMMAND_RESPONSE_CMD_DATA_MAX_LENGTH     10
#define XBEE_REMOTE_COMMAND_RESPONSE_CMD_DATA_MAX_LENGTH 10
#define XBEE_ADDR64_LENGTH  8
#define XBEE_ADDR16_LENGTH  2
// --- END

XBeeDriver::XBeeDriver(const char *tty_dev)
: next_frame_id(0)
{
	const int baudrate = 57600;

	xbee_fd = open(tty_dev, O_RDWR | O_NOCTTY | O_SYNC);
	if (xbee_fd < 0)
	{
		perror(tty_dev);
		exit(EXIT_FAILURE);
	}

        struct termios tc;
        speed_t chosenbaud;

	switch (baudrate)
	{
		case 134:
			chosenbaud = B134;
			break;
		case 150:
			chosenbaud = B150;
			break;
		case 200:
			chosenbaud = B200;
			break;
		case 300:
			chosenbaud = B300;
			break;
		case 600:
			chosenbaud = B600;
			break;
		case 1200:
			chosenbaud = B1200;
			break;
		case 1800:
			chosenbaud = B1800;
			break;
		case 2400:
			chosenbaud = B2400;
			break;
		case 4800:
			chosenbaud = B4800;
			break;
		case 9600:
			chosenbaud = B9600;
			break;
		case 19200:
			chosenbaud = B19200;
			break;
		case 38400:
			chosenbaud = B38400;
			break;
		case 57600:
			chosenbaud = B57600;
			break;
		case 115200:
			chosenbaud = B115200;
			break;
		case 230400:
			chosenbaud = B230400;
			break;
		case 460800:
			chosenbaud = B460800;
			break;
		case 500000:
			chosenbaud = B500000;
			break;
		case 576000:
			chosenbaud = B576000;
			break;
		case 921600:
			chosenbaud = B921600;
			break;
		default:
			fprintf(stderr, "Unsupported %d baud rate\n", baudrate);
			exit(EXIT_FAILURE);
	}

        if (tcgetattr(xbee_fd, &tc))
	{
                perror("tcgetattr()");
                exit(EXIT_FAILURE);
        }

	// Input flags
	tc.c_iflag &= ~ IGNBRK;			// enable ignoring break
	tc.c_iflag &= ~(IGNPAR | PARMRK);	// disable parity checks
	tc.c_iflag &= ~ INPCK;			// disable parity checking
	tc.c_iflag &= ~ ISTRIP;			// disable stripping 8th bit
	tc.c_iflag &= ~(INLCR | ICRNL);		// disable translating NL <-> CR
	tc.c_iflag &= ~ IGNCR;			// disable ignoring CR
	tc.c_iflag &= ~(IXON | IXOFF);		// disable XON/XOFF flow control

	// Output flags
	tc.c_oflag &= ~ OPOST;			// disable output processing
	tc.c_oflag &= ~(ONLCR | OCRNL);		// disable translating NL <-> CR
	tc.c_oflag &= ~ OFILL;			// disable fill characters
	tc.c_cflag |=   CLOCAL;			// prevent changing ownership
	tc.c_cflag |=   CREAD;			// enable reciever
	tc.c_cflag &= ~ PARENB;			// disable parity
	if (baudrate >= 115200)
		tc.c_cflag |=   CSTOPB;		// enable 2 stop bits for the high baudrate
	else
		tc.c_cflag &= ~ CSTOPB;		// disable 2 stop bits
	tc.c_cflag &= ~ CSIZE;			// remove size flag...
	tc.c_cflag |=   CS8;			//  ...enable 8 bit characters
	tc.c_cflag |=   HUPCL;			// enable lower control lines on close - hang up
#ifdef XBEE_NO_RTSCTS
	tc.c_cflag &= ~ CRTSCTS;		// disable hardware CTS/RTS flow control
#else
	tc.c_cflag |=   CRTSCTS;		// enable hardware CTS/RTS flow control
#endif

	// Local flags
	tc.c_lflag &= ~ ISIG;			// disable generating signals
	tc.c_lflag &= ~ ICANON;			// disable canonical mode - line by line
	tc.c_lflag &= ~ ECHO;			// disable echoing characters
	tc.c_lflag &= ~ ECHONL;			// ???
	tc.c_lflag &= ~ NOFLSH;			// disable flushing on SIGINT
	tc.c_lflag &= ~ IEXTEN;			// disable input processing

	// Control characters
	memset(tc.c_cc, 0, sizeof(tc.c_cc));

	// Set baud rate
	if (cfsetspeed(&tc, chosenbaud))
	{
		perror("cfsetspeed()");
		exit(EXIT_FAILURE);
	}

	tc.c_cc[VMIN] = 1;			// enable blocking read()
	tc.c_cc[VTIME] = 5;			// but, timeout = .5 second

	if (tcsetattr(xbee_fd, TCSANOW, &tc))
	{
		perror("tcsetattr()");
		exit(EXIT_FAILURE);
	}

	// Enable transmission
	if (tcflow(xbee_fd, TCOON | TCION))
	{
		perror("tcflow()");
		exit(EXIT_FAILURE);
	}
}

XBeeDriver::XBeeDriver(const char *tcp_host, unsigned int tcp_port)
: next_frame_id(0)
{
	xbee_fd = connectTcp(tcp_host, tcp_port);
	if (xbee_fd == -1)
		errx(EXIT_FAILURE, "Connessione a %s:%u fallita", tcp_host, tcp_port);
}

XBeeDriver::~XBeeDriver()
{
	close(xbee_fd);
}

void XBeeDriver::sendto(const void *buffer, size_t length, const XBeeAddress &dest)
{
	// Lunghezza del frame API, eccetto delimiter, length e checksum
	const size_t api_frame_contents_length = 1 /* API identifier */
		+ 1 /* frame ID */ + (dest.is64bit() ? 8 : 2) /* dest address */
		+ 1 /* options byte */ + length /* actual payload */;

#ifdef XBEE_TX_STATUS_ENABLE
	/* Frame ID */
	if (++next_frame_id == 0) // If FrameID = 0 no Tx Status will be received
		next_frame_id = 1;
#endif

	uint8_t *frame = new uint8_t[3 + api_frame_contents_length + 1];
	uint8_t *curr_pos = frame;

	// Delimiter, length MSB, length LSB, API identifier, frame ID
	*curr_pos++ =  XBEE_START_DELIMITER;
	*curr_pos++ = (api_frame_contents_length >> 8) & 0xFF;
	*curr_pos++ =  api_frame_contents_length & 0xFF;
	*curr_pos++ =  dest.is64bit() ? XBEE_TX64_REQUEST : XBEE_TX16_REQUEST;
	*curr_pos++ =  next_frame_id;

	// Dest address
	memcpy(curr_pos, dest.getAddress(), dest.is64bit() ? 8 : 2);
	curr_pos += dest.is64bit() ? 8 : 2;

	// Options byte
	*curr_pos++ = 0x00;

	// Payload
	memcpy(curr_pos, buffer, length);
	curr_pos += length;

	// Checksum (calcolato a partire da API identifier [ie il terzo byte])
	*curr_pos++ = ~std::accumulate(frame + 3, frame + 3 + api_frame_contents_length, 0);
	write(xbee_fd, frame, curr_pos - frame);

	delete[] frame;
}

ssize_t XBeeDriver::recvfrom(void *buffer, size_t maxlength, XBeeAddress *out_srcaddr)
{
	ssize_t lunghezza_dati = -1;

	uint8_t tmp[2];

	// Delimiter
	if (!readall(tmp, 1) || tmp[0] != XBEE_START_DELIMITER)
		return -1;

	// Length LSB+MSB
	if (!readall(tmp, 2) || (tmp[0] == 0 && tmp[1] == 0))
		return -1;

	// Payload
	uint16_t frame_len = tmp[1] | (((uint16_t)tmp[0]) << 8);
	uint8_t *frame_data = new uint8_t[frame_len];
	if (!readall(frame_data, frame_len))
	{
		delete[] frame_data;
		return -1;
	}

	// Checksum
	if (!readall(tmp, 1) || std::accumulate(frame_data, frame_data + frame_len, tmp[0]) != 0xFF)
	{
		delete[] frame_data;
		return -1;
	}

	switch (frame_data[0])
	{
		case XBEE_RX16_INDICATOR:
			if (out_srcaddr)
				*out_srcaddr = XBeeAddress::fromByteArray(frame_data + 1, 2);
			lunghezza_dati = frame_len - 5;
			memcpy(buffer, frame_data + 5, std::min(lunghezza_dati, (ssize_t)maxlength));
			break;
		case XBEE_RX64_INDICATOR:
			if (out_srcaddr)
				*out_srcaddr = XBeeAddress::fromByteArray(frame_data + 1, 8);
			lunghezza_dati = frame_len - 11;
			memcpy(buffer, frame_data + 11, std::min(lunghezza_dati, (ssize_t)maxlength));
			break;
#ifdef XBEE_TX_STATUS_ENABLE
		case XBEE_TX_STATUS:
			// xbee ci informa che un pacchetto è stato inviato. Ignoriamo l'evento
			break;
#endif
		default:
			// Ignora valori API identifier non riconosciuti
			break;
	}
	
	delete[] frame_data;
	return lunghezza_dati;
}

bool XBeeDriver::readall(uint8_t *buffer, size_t length)
{
	while (length != 0)
	{
		int res = read(xbee_fd, buffer, length);

		if (res <= 0) // I/O error
		{
			err(EXIT_FAILURE, "XBeeDriver read()");
		}
		else
		{
			buffer += res;
			length -= res;
		}
	}

	return true;
}
