#ifndef HEXIMAGE_H
#define HEXIMAGE_H

#include "DeviceInfo.h"

#include <stddef.h>
#include <stdint.h>
#include <vector>

class HexImage
{
	public:
		HexImage(const char *hexImagePath, const DeviceInfo *deviceInfo);
		~HexImage();

		class ProgramPage
		{
			friend class HexImage;

			public:
				uint32_t getBaseAddress() const;
				uint32_t getEndAddress() const;
				std::vector<uint8_t> getPackedContents() const;
				bool isTouched() const;

			private:
				ProgramPage(uint32_t base_address, unsigned int numwords);
				~ProgramPage();
				void write_byte(uint32_t addr, uint8_t val, unsigned int shift);

				const uint32_t m_base_addr;
				const unsigned int m_numwords;
				bool m_touched;
				std::vector<uint32_t> m_words;
		};

		class ConfigurationWord
		{
			friend class HexImage;

			public:
				uint32_t getAddress() const;
				const char *getName() const;
				uint16_t getValue() const;
				bool isTouched() const;

			private:
				ConfigurationWord(uint32_t address, const char *name);
				~ConfigurationWord();
				void write_byte(uint8_t val, unsigned int shift);

				const uint32_t m_address;
				const char *m_name;
				bool m_touched;
				uint16_t m_value;
		};

		bool loadOk() const;
		void dropUntouchedSections();
		void dropRtspKernelSections();

		const std::vector<const ProgramPage*> &getProgramPages() const;
		const ProgramPage *getProgramPage(uint32_t base_address) const;
		const std::vector<const ConfigurationWord*> &getConfigurationWords() const;

	private:
		bool m_loadOk;
		std::vector<const ProgramPage*> m_programPages;
		std::vector<const ConfigurationWord*> m_configurationWords;
		uint32_t m_rtspKernelBaseAddress;
};

#endif
