#include "Utils.h"

#include <lzo/lzo1x.h>
#include <cstdlib>
#include <cstring>
#include <err.h>
#include <limits.h>

// Memoria di lavoro per compressione LZO
#define HEAP_ALLOC(var,size) lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]
static __thread HEAP_ALLOC(wrkmem, LZO1X_999_MEM_COMPRESS);

// Costanti per CRC16
#define CRC16_INITIAL	0xFFFF
#define CRC16_POLY	0xA001

// Calcola CRC16 (val. iniziale crc = CRC16_INITIAL)
static uint16_t crc16(uint8_t data, uint16_t crc)
{
	crc ^= data;

	for (int i = 0; i < 8; ++i)
	{
		if (crc & 1)
			crc = (crc >> 1) ^ CRC16_POLY;
		else
			crc = (crc >> 1);
	}

	return crc;
}

uint16_t crc16(const std::vector<uint8_t> &data)
{
	uint16_t r = CRC16_INITIAL;

	for (unsigned int i = 0; i < data.size(); i++)
		r = crc16(data[i], r);

	return r;
}

static void initialize_lzo()
{
	static bool done = false;
	if (done)
		return;
	done = true;

	if (lzo_init() != LZO_E_OK)
		errx(EXIT_FAILURE, "lzo_init() failed");
}

std::vector<uint8_t> compressLZO(const std::vector<uint8_t> &data)
{
	int lzoerr;
	initialize_lzo();

	std::vector<uint8_t> result(data.size() * 2);
	lzo_uint out_len = result.size();
	if ((lzoerr = lzo1x_999_compress(&data[0], data.size(), &result[0], &out_len, wrkmem)) != LZO_E_OK)
		errx(EXIT_FAILURE, "lzo1x_999_compress() failed with error code %d", lzoerr);
	result.resize(out_len);

	if (false) // Verifica dei dati compressi
	{
		std::vector<uint8_t> verify_buff(data.size() * 2);
		lzo_uint ver_len = verify_buff.size();
		if ((lzoerr = lzo1x_decompress_safe(&result[0], result.size(), &verify_buff[0], &ver_len, NULL)) != LZO_E_OK)
			errx(EXIT_FAILURE, "lzo1x_decompress() failed with error code %d", lzoerr);
		verify_buff.resize(ver_len);

		if (verify_buff.size() != data.size() || memcmp(&verify_buff[0], &data[0], data.size()) != 0)
			errx(EXIT_FAILURE, "compression error: decompress(compress(data)) != data");
	}

	return result;
}

bool compressDiff(const std::vector<uint8_t> &dataA, const std::vector<uint8_t> &dataB,
	std::pair<unsigned int, std::vector<uint8_t> > *out_deltaInfo)
{
	if (dataA.size() != dataB.size())
		abort();

	unsigned int bestNonZeroCount = UINT_MAX;
	int bestR = 0;

	// prova ogni 3 byte, ovvero ogni word a 24 bit della memoria programma del dspic
	// cerca rotazione "r" che massimizzi il numero di zeri di "diff" (v. sotto)
	for (int r = 0; r < dataA.size(); r += 3)
	{
		unsigned int nonZeroCount = 0;
		for (int i = 0, j = r + dataA.size(); i < dataA.size(); i++, j++)
		{
			if (j >= dataA.size())
				j -= dataA.size();

			if (dataA[j] != dataB[i])
			{
				if (++nonZeroCount >= bestNonZeroCount)
					goto continue_outer_loop;
			}
		}

		if (nonZeroCount == 0 && r == 0)
			return true; // dataA e dataB sono identici

		bestNonZeroCount = nonZeroCount;
		bestR = r;

continue_outer_loop:
		;
	}

	std::vector<uint8_t> diff(dataB.begin(), dataB.end());
	for (int i = 0, j = bestR + dataA.size(); i < dataA.size(); i++, j++)
	{
		if (j >= dataA.size())
			j -= dataA.size();
		diff[i] -= dataA[j];
	}

	// restituisce r e relativo diff compresso
	*out_deltaInfo = std::pair<unsigned int, std::vector<uint8_t> >(bestR, compressLZO(diff));
	return false;
}
