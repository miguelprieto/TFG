#include "DeviceInfo.h"
#include "HexImage.h"
#include "Utils.h"

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <err.h>
#include <unistd.h>

using namespace std;

static void line_hex_dump(uint32_t line_addr, const uint8_t *data)
{
	cout << " " << hex << setfill('0') << setw(6) << line_addr << ":";

	for (unsigned int i = 0; i < 12; ++i)
	{
		if (i % 3 == 0)
			cout << "  ";
		else
			cout << " ";

		cout << hex << setfill('0') << setw(2) << (int)data[i];
	}

	cout << "  ";

	for (unsigned int i = 0; i < 12; ++i)
	{
		if (i % 3 == 0)
			cout << " ";

		if (!isprint(data[i]))
			cout << '.';
		else
			cout << (char)data[i];
	}

	cout << endl;
}

int main(int argc, char * const *argv)
{
	bool hideUntouchedSections = true;
	bool dumpProgramWords = false;
	const char *deviceName = NULL;
	const char *newHexFilename = NULL;
	const char *oldHexFilename = NULL;
	int c;

	while ((c = getopt(argc, argv, "ad:ho:x")) != -1)
	{
		switch (c)
		{
			case 'a':
				hideUntouchedSections = false;
				break;
			case 'd':
				deviceName = optarg;
				break;
			case 'o':
				oldHexFilename = optarg;
				break;
			case 'x':
				dumpProgramWords = true;
				break;
			case 'h':
				cerr << "Uso: " << argv[0] << " [-a] -d DEVICENAME [-x] firmware.hex" << endl;
				cerr << "    -h   Mostra questo messaggio" << endl;
				cerr << "    -a   Elenca anche pagine non utilizzate dal firmware" << endl;
				cerr << "    -d   Tipo di PIC" << endl;
				cerr << "    -x   Mostra contenuto delle word della memoria programma" << endl;
				// fallback
			case '?':
				return c == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
			default:
				abort();
		}
	}

	const DeviceInfo *deviceInfo;
	if (deviceName == NULL)
	{
		errx(EXIT_FAILURE, "Nessun tipo di PIC specificato");
	}
	else if ((deviceInfo = getDeviceInfo(deviceName)) == NULL)
	{
		errx(EXIT_FAILURE, "Tipo di PIC non riconosciuto");
	}

	if (optind == argc - 1)
	{
		newHexFilename = argv[optind];
	}
	else if (optind == argc)
	{
		errx(EXIT_FAILURE, "Nessun firmware specificato");
	}
	else
	{
		errx(EXIT_FAILURE, "Troppi parametri");
	}

	HexImage newImage(newHexFilename, deviceInfo);
	if (!newImage.loadOk())
		errx(EXIT_FAILURE, "%s: Errore di caricamento", newHexFilename);

	HexImage oldImage(oldHexFilename ?: "", deviceInfo);
	if (oldHexFilename != NULL && !oldImage.loadOk())
		errx(EXIT_FAILURE, "%s: Errore di caricamento", newHexFilename);

	if (hideUntouchedSections)
		newImage.dropUntouchedSections();
	oldImage.dropUntouchedSections();

	unsigned int totalUncompressed = 0;
	unsigned int totalCompressed = 0;
	unsigned int totalDelta = 0;

	const vector<const HexImage::ProgramPage*> &programPages = newImage.getProgramPages();
	for (vector<const HexImage::ProgramPage*>::const_iterator it = programPages.begin();
		it != programPages.end(); ++it)
	{
		const unsigned int compressedDataSize = compressLZO((*it)->getPackedContents()).size();

		const float comprRatio = (float)100 * compressedDataSize / (*it)->getPackedContents().size();
		cout << "Page 0x" << hex << (*it)->getBaseAddress() << "-0x" << (*it)->getEndAddress() << ": crc16=0x"
			<< crc16((*it)->getPackedContents()) << dec
			<< ", compressa=" << compressedDataSize << " (" << comprRatio << "%)";

		if (oldHexFilename != NULL)
		{
			std::pair<unsigned int, std::vector<uint8_t> > deltaInfo;
			const HexImage::ProgramPage *oldPage = oldImage.getProgramPage((*it)->getBaseAddress());
			if (oldPage == NULL)
			{
				cout << ", delta=new";
				totalDelta += compressedDataSize;
			}
			else if (compressDiff(oldPage->getPackedContents(),
				(*it)->getPackedContents(), &deltaInfo))
			{
				cout << ", delta=match";
			}
			else
			{
				const unsigned int deltaDataSize = deltaInfo.second.size();
				const float deltaRatio = (float)100 * deltaDataSize / (*it)->getPackedContents().size();
				cout << ", delta=" << deltaDataSize << " (" << deltaRatio << "%, offset="
					<< deltaInfo.first << ")";
				totalDelta += deltaDataSize;
			}
		}

		totalUncompressed += (*it)->getPackedContents().size();
		totalCompressed += compressedDataSize;

		if ((*it)->isTouched() == false)
			cout << " (non utilizzata)";
		cout << endl;

		if (dumpProgramWords)
		{
			const std::vector<uint8_t> packedBytes = (*it)->getPackedContents();
			for (unsigned int i = 0; i < packedBytes.size(); i += 12)
				line_hex_dump((*it)->getBaseAddress() + i*2/3, &packedBytes[i]);
			cout << endl;
		}
	}

	cout << endl;

	const float comprRatio = (float)100 * totalCompressed / totalUncompressed;
	cout << "Dimensione totale dati: non compressa=" << totalUncompressed << ", compressa=" << totalCompressed << " (" << comprRatio << "%)";
	if (oldHexFilename != NULL)
	{
		const float comprRatio = (float)100 * totalDelta / totalUncompressed;
		cout << ", delta=" << totalDelta << " (" << comprRatio << "%)";
	}
	cout << endl;

	cout << endl;

	const vector<const HexImage::ConfigurationWord*> &configurationWords = newImage.getConfigurationWords();
	for (vector<const HexImage::ConfigurationWord*>::const_iterator it = configurationWords.begin();
		it != configurationWords.end(); ++it)
	{
		cout << "Configuration word " << (*it)->getName() << " (0x" << hex << (*it)->getAddress()
			<< "): value=0x" << (*it)->getValue() << dec;

		if ((*it)->isTouched() == false)
			cout << " (non utilizzata)";
		cout << endl;
	}

	return EXIT_SUCCESS;
}
