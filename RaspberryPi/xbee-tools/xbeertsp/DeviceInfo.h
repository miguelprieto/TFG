#ifndef DEVICEINFO_H
#define DEVICEINFO_H

#include <map>
#include <stdint.h>

struct DeviceInfo
{
	uint16_t family_id, device_id;
	unsigned int wordsPerPage;	// Number of words in each erase block
	unsigned int programMemWords;	// Total amount of words in program memory
	uint32_t rtspKernelBaseAddress;
	std::map<uint32_t, const char *> configurationWords; // address -> name
};

const DeviceInfo *getDeviceInfo(const char *deviceName);

#endif // DEVICEINFO_H
