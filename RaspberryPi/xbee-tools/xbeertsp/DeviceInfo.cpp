#include "DeviceInfo.h"

#include <string.h>

struct DeviceDatabase
{
	DeviceInfo dsPIC33FJ128MC802;
	DeviceInfo dsPIC33EP512MC502;

	DeviceDatabase()
	{
		dsPIC33FJ128MC802.family_id = 0x33f;
		dsPIC33FJ128MC802.device_id = 0x629;
		dsPIC33FJ128MC802.wordsPerPage = 512;
		dsPIC33FJ128MC802.programMemWords = 0xac00;
		dsPIC33FJ128MC802.rtspKernelBaseAddress = 0x15000;
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80000, "FBS"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80002, "FSS"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80004, "FGS"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80006, "FOSCSEL"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80008, "FOSC"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF8000A, "FWDT"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF8000C, "FPOR"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF8000E, "FICD"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80010, "FUID0"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80012, "FUID1"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80014, "FUID2"));
		dsPIC33FJ128MC802.configurationWords.insert(std::pair<uint32_t, const char *>(0xF80016, "FUID3"));

		dsPIC33EP512MC502.family_id = 0x33e;
		dsPIC33EP512MC502.device_id = 0x1785;
		dsPIC33EP512MC502.wordsPerPage = 1024;
		dsPIC33EP512MC502.programMemWords = 0x2ac00;
		dsPIC33EP512MC502.rtspKernelBaseAddress = 0x54800;
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557EC, "Reserved"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557EE, "Reserved"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557F0, "FICD"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557F2, "FPOR"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557F4, "FWDT"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557F6, "FOSC"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557F8, "FOSCSEL"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557FA, "FGS"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557FC, "Reserved"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x557FE, "Reserved"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x800FF8, "FUID0"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x800FFA, "FUID1"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x800FFC, "FUID2"));
		dsPIC33EP512MC502.configurationWords.insert(std::pair<uint32_t, const char *>(0x800FFE, "FUID3"));
	}
};

const DeviceInfo *getDeviceInfo(const char *deviceName)
{
	static DeviceDatabase db;

	if (strcmp(deviceName, "dsPIC33FJ128MC802") == 0)
		return &db.dsPIC33FJ128MC802;
	else if (strcmp(deviceName, "dsPIC33EP512MC502") == 0)
		return &db.dsPIC33EP512MC502;
	else
		return NULL;
}
