#include "HexImage.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <numeric>
#include <iomanip>
#include <cctype>
#include <stdint.h>

using namespace std;

// converte da stringa esadecimale a uint16_t
static uint16_t parse_hex_number(string::iterator begin, string::iterator end)
{
	uint16_t x;

	stringstream ss;
	ss << hex << string(begin, end);
	ss >> x;

	return x;
}

static map<uint32_t, uint8_t> parse_hex_file(const char *path)
{
	ifstream hexfile(path, ios::in | ios::binary);
	if (!hexfile.is_open())
		return map<uint32_t, uint8_t>();

	uint32_t high_address = 0;
	bool eof_marker_found = false;

	map<uint32_t, uint8_t> loaded_bytes;

	while (hexfile.good())
	{
		string line;
		getline(hexfile, line);

		if (line.empty()) // salta righe vuote
			continue;

		if (eof_marker_found)
			return map<uint32_t, uint8_t>(); // ci sono dati dopo il marker eof

		if (line.length() < 11 || line[0] != ':')
			return map<uint32_t, uint8_t>(); // riga non valida

		for (string::iterator it = line.begin() + 1; it != line.end(); ++it)
		{
			if (*it >= '0' && *it <= '9')
				continue;
			else if (*it >= 'a' && *it <= 'f')
				continue;
			else if (*it >= 'A' && *it <= 'F')
				continue;
			else
				return map<uint32_t, uint8_t>(); // carattere non valido (non è una cifra esadecimale)
		}

		const uint8_t byte_count = parse_hex_number(line.begin() + 1, line.begin() + 3);
		const uint16_t low_address = parse_hex_number(line.begin() + 3, line.begin() + 7);
		const uint8_t type = parse_hex_number(line.begin() + 7, line.begin() + 9);
		const uint8_t checksum = parse_hex_number(line.end() - 2, line.end());
		if (line.length() != 11 + byte_count * 2)
			return map<uint32_t, uint8_t>(); // lunghezza effettiva non corrispondente alla lunghezza dichiarata

		vector<uint8_t> data(byte_count);
		for (unsigned int i = 0; i < byte_count; ++i)
			data[i] = parse_hex_number(line.begin() + 9 + 2*i, line.begin() + 9 + 2*i + 2);

		if ((accumulate(data.begin(), data.end(), byte_count
				+ (low_address & 0xff) + ((low_address >> 8) & 0xff)
				+ type + checksum) & 0xff) != 0)
			return map<uint32_t, uint8_t>(); // errore checksum

		switch (type)
		{
			case 0:
			{
				uint32_t target_address = high_address | low_address;
				for (vector<uint8_t>::iterator it = data.begin();
						it != data.end(); ++it, ++target_address)
					loaded_bytes.insert(pair<uint32_t, uint8_t>(target_address, *it));
				break;
			}
			case 1:
			{
				eof_marker_found = true;
				break;
			}
			case 4:
			{
				high_address = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16);
				break;
			}
			default:
			{
				return map<uint32_t, uint8_t>(); // tipo di riga non valido
			}
		}
	}

	if (hexfile.bad() || !eof_marker_found)
		return map<uint32_t, uint8_t>(); // errore di i/o

	return loaded_bytes;
}

HexImage::ProgramPage::ProgramPage(uint32_t base_address, unsigned int numwords)
: m_base_addr(base_address), m_numwords(numwords), m_touched(false), m_words(numwords, 0xFFFFFF)
{
}

HexImage::ProgramPage::~ProgramPage()
{
}

void HexImage::ProgramPage::write_byte(uint32_t addr, uint8_t val, unsigned int shift)
{
	const uint32_t mask = 0xFF << shift;
	const uint32_t value = (uint32_t)val << shift;
	const unsigned int offset = (addr - m_base_addr) / 2;
	m_words[offset] = (m_words[offset] & ~mask) | value;
	m_touched = true;
}

uint32_t HexImage::ProgramPage::getBaseAddress() const
{
	return m_base_addr;
}

uint32_t HexImage::ProgramPage::getEndAddress() const
{
	return m_base_addr + 2 * m_numwords - 1;
}

std::vector<uint8_t> HexImage::ProgramPage::getPackedContents() const
{
	std::vector<uint8_t> result;
	result.reserve(m_numwords * 3);
	for (int i = 0; i < m_numwords; i++)
	{
		result.push_back((uint8_t)(m_words[i] >> 0));
		result.push_back((uint8_t)(m_words[i] >> 8));
		result.push_back((uint8_t)(m_words[i] >> 16));
	}
	return result;
}

bool HexImage::ProgramPage::isTouched() const
{
	return m_touched;
}

HexImage::ConfigurationWord::ConfigurationWord(uint32_t address, const char *name)
: m_address(address), m_name(name), m_touched(false), m_value(0xFFFF)
{
}

HexImage::ConfigurationWord::~ConfigurationWord()
{
}

void HexImage::ConfigurationWord::write_byte(uint8_t val, unsigned int shift)
{
	const uint32_t mask = 0xFF << shift;
	const uint32_t value = (uint32_t)val << shift;
	m_value = (m_value & ~mask) | value;
	m_touched = true;
}

uint32_t HexImage::ConfigurationWord::getAddress() const
{
	return m_address;
}

const char *HexImage::ConfigurationWord::getName() const
{
	return m_name;
}

uint16_t HexImage::ConfigurationWord::getValue() const
{
	return m_value;
}

bool HexImage::ConfigurationWord::isTouched() const
{
	return m_touched;
}

HexImage::HexImage(const char *hexImagePath, const DeviceInfo *deviceInfo)
{
	const map<uint32_t, uint8_t> contents = parse_hex_file(hexImagePath);
	if (contents.empty())
	{
		m_loadOk = false;
		return;
	}
	else
	{
		vector<ProgramPage*> program_memory;
		map<unsigned int, ConfigurationWord*> config_memory;

		// Crea buffer per memoria programma
		for (unsigned int word = 0; word < deviceInfo->programMemWords; word += deviceInfo->wordsPerPage)
		{
			program_memory.push_back(new ProgramPage(word * 2,
				min(deviceInfo->programMemWords - word, deviceInfo->wordsPerPage)));
		}

		for (map<uint32_t, const char *>::const_iterator it = deviceInfo->configurationWords.begin();
			it != deviceInfo->configurationWords.end(); ++it)
		{
			config_memory.insert(pair<unsigned int, ConfigurationWord*>(it->first / 2, new ConfigurationWord(it->first, it->second)));
		}

		for (map<uint32_t, uint8_t>::const_iterator it = contents.begin(); it != contents.end(); ++it)
		{
			const uint32_t bin_addr = it->first;
			const uint8_t value = it->second;

			bool thisByteMakesSense = false;
			if (bin_addr < deviceInfo->programMemWords * 4) // destinazione: memoria programma
			{
				if (bin_addr % 4 == 3)
				{
					if (value == 0)
						thisByteMakesSense = true;
				}
				else
				{
					const uint32_t prog_addr = 2 * (bin_addr / 4);
					const unsigned int offset = (bin_addr % 4) * 8;
					program_memory[prog_addr / (deviceInfo->wordsPerPage * 2)]->write_byte(prog_addr, value, offset);
					thisByteMakesSense = true;
				}
			}

			map<unsigned int, ConfigurationWord*>::iterator cw = config_memory.find(bin_addr / 4);
			if (cw != config_memory.end())
			{
				if (bin_addr % 4 >= 2)
				{
					if (value == 0)
						thisByteMakesSense = true;
				}
				else
				{
					const unsigned int offset = (bin_addr % 4) * 8;
					cw->second->write_byte(value, offset);
					thisByteMakesSense = true;
				}
			}

			if (thisByteMakesSense == false)
			{
				m_loadOk = false;
				return;
			}
		}

		for (vector<ProgramPage*>::const_iterator it = program_memory.begin(); it != program_memory.end(); ++it)
			m_programPages.push_back(*it);

		for (map<unsigned int, ConfigurationWord*>::iterator it = config_memory.begin(); it != config_memory.end(); ++it)
			m_configurationWords.push_back(it->second);
	}

	m_loadOk = true;
	m_rtspKernelBaseAddress = deviceInfo->rtspKernelBaseAddress;
}

HexImage::~HexImage()
{
	for (vector<const ProgramPage*>::const_iterator it = m_programPages.begin(); it != m_programPages.end(); ++it)
		delete *it;

	for (vector<const ConfigurationWord*>::const_iterator it = m_configurationWords.begin(); it != m_configurationWords.end(); ++it)
		delete *it;
}

bool HexImage::loadOk() const
{
	return m_loadOk;
}

void HexImage::dropUntouchedSections()
{
	vector<const ProgramPage*>::iterator it_p = m_programPages.begin();
	while (it_p != m_programPages.end())
	{
		if ((*it_p)->isTouched() == true)
		{
			++it_p;
		}
		else
		{
			delete *it_p;
			it_p = m_programPages.erase(it_p);
		}
	}

	vector<const ConfigurationWord*>::iterator it_c = m_configurationWords.begin();
	while (it_c != m_configurationWords.end())
	{
		if ((*it_c)->isTouched() == true)
		{
			++it_c;
		}
		else
		{
			delete *it_c;
			it_c = m_configurationWords.erase(it_c);
		}
	}
}

// Scarta pagine contenenti il codice del loader rtsp
void HexImage::dropRtspKernelSections()
{
	vector<const ProgramPage*>::iterator it = m_programPages.begin();
	while (it != m_programPages.end())
	{
		if ((*it)->getBaseAddress() < m_rtspKernelBaseAddress)
		{
			++it;
		}
		else
		{
			delete *it;
			it = m_programPages.erase(it);
		}
	}
}

const std::vector<const HexImage::ProgramPage*> &HexImage::getProgramPages() const
{
	return m_programPages;
}

const HexImage::ProgramPage *HexImage::getProgramPage(uint32_t base_address) const
{
	for (vector<const ProgramPage*>::const_iterator it = m_programPages.begin();
		it != m_programPages.end(); ++it)
	{
		if ((*it)->getBaseAddress() == base_address)
			return *it;
	}

	return NULL;
}

const std::vector<const HexImage::ConfigurationWord*> &HexImage::getConfigurationWords() const
{
	return m_configurationWords;
}
