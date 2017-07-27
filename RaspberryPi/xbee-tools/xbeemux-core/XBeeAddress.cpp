#include "XBeeAddress.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const XBeeAddress XBeeAddress::broadcast(0xff, 0xff);

bool XBeeAddress::operator==(const XBeeAddress &other) const
{
	if (sixtyfour != other.sixtyfour)
		return false;
	else if (sixtyfour)
		return memcmp(addr, other.addr, 8) == 0;
	else
		return memcmp(addr, other.addr, 2) == 0;
}

XBeeAddress XBeeAddress::fromByteArray(const uint8_t *data, size_t len_bytes)
{
	switch (len_bytes)
	{
		case 2:
			return XBeeAddress(data[0], data[1]);
		case 8:
			return XBeeAddress(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		default:
			fprintf(stderr, "%s: Lunghezza indirizzo (%ld) non valida\n", __func__, len_bytes);
			exit(EXIT_FAILURE);
	}
}

std::string XBeeAddress::addrString() const
{
	char buffer[48];

	if (sixtyfour)
		sprintf(buffer, "XBeeAddress(0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x)",
			addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
	else
		sprintf(buffer, "XBeeAddress(0x%02x, 0x%02x)",
			addr[0], addr[1]);

	return std::string(buffer);
}
