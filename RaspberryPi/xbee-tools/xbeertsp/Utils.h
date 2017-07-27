#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <vector>

uint16_t crc16(const std::vector<uint8_t> &data);
std::vector<uint8_t> compressLZO(const std::vector<uint8_t> &data);
bool compressDiff(const std::vector<uint8_t> &dataA, const std::vector<uint8_t> &dataB,
	std::pair<unsigned int, std::vector<uint8_t> > *out_deltaInfo);

#endif // UTILS_H
