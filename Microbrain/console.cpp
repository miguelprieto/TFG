#include <string.h>
#include "serial.h"
#include <stdio.h>
#include "canstdio_endpoint.h"
#include "color.h"
#include "console.h"
#include "serial.h"
#include "simulator_interface.h"

#define ESCAPE_SEQUENCE_TIMEOUT 2 /* 2 scatti del game_timer, ovvero 0.5 s */

#define HISTORY_SIZE 5
#define LENGTH_STRING 60

static bool xbee_enabled = true; // attiva output su CAN-over-XBee

static char historyLines[HISTORY_SIZE][LENGTH_STRING];  //array di stringhe precedentemente digitati.
static char* currentLine;         // tutte le stringhe a video sono caricate in questo array.
static bool exist[HISTORY_SIZE];  // 1 se la stringa i-esima esiste (anche se la lunghezza è 0).
static int selected = -1;         // linea i-esama visualizzata.
static int lastLine = 0;	  // linea più vecchia presente nella cronologia.
static bool modified = 0;         // serve per capire se modifico una vecchia linea della cronologia, quindi per scaricare le modifiche.
static char tmpLine[LENGTH_STRING];   //in caso in cui l'utente richieda la cronologia la, stringa appena digitata (ma mai eseguita) viene salvata momentaneamente qui.
static unsigned int cursor;
static bool autocompletion_ambiguous;

static int getFirstLine()   //restitutisce la liena più recente.
{
	return ((HISTORY_SIZE + lastLine) -1) % HISTORY_SIZE;
}

static int getILine() //restitutisce la linea i-esima, dove i è selected.
{
	return ((getFirstLine() + HISTORY_SIZE) - selected) % HISTORY_SIZE;
}

static void leftShiftChar(int i)
{
	for (; i >= 0; i--)
		currentLine[strlen(currentLine) - i -1] = currentLine[strlen(currentLine) - i ];
	
}

static void rightShiftChar(int i)
{
	int unsigned j;
	for ( j = strlen(currentLine) + 1; j >= strlen(currentLine) - cursor; j-- )
		currentLine[j] = currentLine[j - 1];
}

static void saveChar(char c) //salva i char che arrivano dalla read_line
{
	if ( cursor > 0 )
	{
		int unsigned i;
		rightShiftChar(cursor);
		currentLine[strlen(currentLine) - cursor -1] = c;

		for ( i = strlen(currentLine) - cursor -1; i < strlen(currentLine); i++)
			putchar(currentLine[i]);

		for ( i = 0; i < cursor; i++)
			putchar('\b');
	}
	else
	{
		int len = strlen(currentLine);
		currentLine[len] = c;
		currentLine[len + 1] = '\0';
		putchar(c);
	}
	if ( selected != -1 )
		modified = 1;
}

static void saveCurrentLine() // funzione che scarica la linea corrente nella cronologia
{
	if ( (strlen(currentLine) > 0) && (strcmp(historyLines[getFirstLine()], currentLine) != 0) )
	{
		currentLine[strlen(currentLine) + 1] = '\0';
		strcpy(historyLines[lastLine], currentLine);
		exist[lastLine] = 1;
		lastLine = (lastLine + 1) % HISTORY_SIZE;
	}
}

static void deleteCharOnScreen()   //cancella la riga attualmente visualizzata a schermo
{
	int unsigned i;
	
	if ( strlen(currentLine) > 0 )
	{
		if ( cursor > 0 )
		{
			for ( i = 0; i < cursor; i++)
			{
				putchar(0x1b);
				putchar(0x5b);
				putchar(0x43);
			}
			cursor = 0;
		}

		for ( i = 0; i < strlen(currentLine); i++ )
		{
			putchar('\b');
			putchar(' ');
			putchar('\b');
		}
	}
	
	
	
}

static void printCurrentLine()  //stampa la linea corrente a schermo
{
	int unsigned i;
	for( i = 0;i < strlen(currentLine); i++ )
		putchar(currentLine[i]);
}

static void backSpace()
{
	if ( strlen(currentLine) > 0 )
	{
		if ( (cursor > 0) && (cursor < strlen(currentLine)) )
		{
			int unsigned i;
			leftShiftChar(cursor);
			putchar('\b');
			for ( i = strlen(currentLine) - cursor; i < strlen(currentLine); i++)
				putchar(currentLine[i]);
			
			for ( i = 0; i < cursor; i++ )
				putchar(' ');

			for ( i = 0; i < cursor * 2; i++)
				putchar('\b');
		}
		else if ( cursor == 0 )
		{
			currentLine[strlen(currentLine) - 1] = '\0';
			
			putchar('\b');
			putchar(' ');
			putchar('\b');
		}

		if ( selected != -1 )
			modified = 1;
	}
}

static void upArrow()
{
	selected++;

	if ( (exist[getILine()]) && (selected < (HISTORY_SIZE)) )
	{
		deleteCharOnScreen();

		if ( modified ) // in caso in cui ho modificato una linea della cronologia scarico le modifiche
		{
			strcpy(historyLines[getILine()], currentLine);
			modified = 0;
		}
		
		if ( (selected == 0) && (strlen(currentLine) > 0) )  // in caso in cui scrivo qualcosa ma poi richiedo la cronologia salvo quello che avevo scritto
		{
			strcpy(tmpLine, currentLine);
		}

		strcpy(currentLine, historyLines[getILine()]);
		printCurrentLine();
	}
	else
		selected--;
}

static void downArrow()
{
	if ( selected >= 0 )
	{
		deleteCharOnScreen();

		if ( modified )
		{
			strcpy(historyLines[getILine()], currentLine);
			modified = 0;
		}

		selected--;
		
		if ( selected >= 0 )
		{
			strcpy(currentLine, historyLines[getILine()]);
			printCurrentLine();
		}
		else
		{
			if ( strlen(tmpLine) > 0 )
			{
				strcpy(currentLine, tmpLine);
				selected = -1;
				printCurrentLine();
				tmpLine[0] = '\0';
			}
			else
			{
				currentLine[0] = '\0';
				selected = -1;
			}
		}
	}
}

static void leftArrow()
{
	if ( (strlen(currentLine) > 0 ) && (cursor < strlen(currentLine)) )
	{
		putchar('\b');
		cursor++;
	}
}

static void rightArrow()
{
	if ( cursor > 0 )
	{
		putchar(0x1b);
		putchar(0x5b);
		putchar(0x43);
		cursor--;
	}
}

static void getFinalString()
{
	if ( strlen(currentLine) > 0 )
		saveCurrentLine();

	selected = -1;
	modified = 0;
	tmpLine[0] = '\0';
	cursor = 0;
}


void read_line(char * s)
{
    currentLine = s;
    currentLine[0] = '\0';

    while (!continue_read_line()) ;
}



void begin_read_line(char *s)
{
	printf("%s[%c]>", ROBOT_NAME, color == YELLOW ? 'Y' : 'B');

	currentLine = s;
	currentLine[0] = '\0';
}

bool continue_read_line()
{
	if (!kbhit())
		return false;

	const char c = read_char();

	if (c < ' ')
	{
		switch (c)
		{
			case '\r':
				getFinalString();
				putchar('\r');
				putchar('\n');
				return true;
			case '\b':
				backSpace();
				break;
			case '\t':
				if (cursor == 0) // completa solo se il cursore è a fine riga
				{
					int oldLineLength = strlen(currentLine);
					autocompletion_ambiguous = false;
					autocompletion_callback(currentLine);
					if (autocompletion_ambiguous)
					{
						// dopo aver stampato la lista dei completamenti possibili,
						// ristampiamo il prompt
						printf("\n%s[%c]>%s", ROBOT_NAME, color == YELLOW ? 'Y' : 'B', currentLine);
					}
					else
					{
						// stampiamo solo gli eventuali caratteri che sono stati aggiunti
						int newLineLength = strlen(currentLine);
						for (int i = oldLineLength; i < newLineLength; i++)
							putchar(currentLine[i]);
						if (oldLineLength != newLineLength && selected != -1)
							modified = 1;
					}
				}
				break;
			case '\e':
				switch (read_escape_sequence())
				{
					case KEY_UP_ARROW:
						upArrow();
						break;
					case KEY_DOWN_ARROW:
						downArrow();
						break;
					case KEY_LEFT_ARROW:
						leftArrow();
						break;
					case KEY_RIGHT_ARROW:
						rightArrow();
						break;
					default:
						break; // Ignora altre sequenze di escape
				}
				break;
		}
	}
	else
	{
		saveChar(c);
	}

	return false;
}

int kbhit()
{
	return serial2_is_input_available() || canstdio_is_input_available();
}

char read_char()
{
	while (!kbhit())
		simulator_relax();

	if (serial2_is_input_available())
		return serial2_getcharacter(false);
	else
		return canstdio_getcharacter(false);
}

static char read_char_with_timeout(int timeout_start)
{
	while (!kbhit())
	{
		if ((game_timer - timeout_start) >= ESCAPE_SEQUENCE_TIMEOUT)
			return 0;

		simulator_relax();
	}

	if (serial2_is_input_available())
		return serial2_getcharacter(false);
	else
		return canstdio_getcharacter(false);
}

enum escape_sequence read_escape_sequence(void)
{
	const int timeout_start = game_timer;

	switch (read_char_with_timeout(timeout_start))
	{
		case 0x00:
			return KEY_ESC;
		case 0x5b:
			switch (read_char_with_timeout(timeout_start))
			{
				case 0x41:
					return KEY_UP_ARROW;
				case 0x42:
					return KEY_DOWN_ARROW;
				case 0x43:
					return KEY_RIGHT_ARROW;
				case 0x44:
					return KEY_LEFT_ARROW;
				default:
					return UNKNOWN_ESCAPE_SEQUENCE;
			}
		default:
			return UNKNOWN_ESCAPE_SEQUENCE;
	}
}

AutoCompleterRef::AutoCompleterRef()
: proc(NULL), values(NULL)
{
}

AutoCompleterRef::AutoCompleterRef(proc_autocomplete proc)
: proc(proc), values(NULL)
{
}

AutoCompleterRef::AutoCompleterRef(const char * const *values)
: proc(NULL), values(values)
{
}

void AutoCompleterRef::operator()(char *buffer, int argidx) const
{
	if (proc != NULL)
		proc(buffer, argidx);
	else if (values != NULL)
	{
		const char * const *completions = values;
		argidx--;

		// Salta stringhe relative a argomenti precedenti
		for (int nullctr = 0; nullctr < argidx; completions++)
		{
			if (*completions == NULL)
				nullctr++;
		}

		class StringArrayCompletionGenerator : public CompletionGenerator
		{
			public:
				StringArrayCompletionGenerator(const char * const *array)
				: array(array), index(0)
				{
				}

				void rewind()
				{
					index = 0;
				}

				const char* next()
				{
					if (array[index] == NULL)
						return NULL;
					else
						return array[index++];
				}

			private:
				const char * const *array;
				int index;
		};

		StringArrayCompletionGenerator stringIterator(completions);
		autocomplete(buffer, &stringIterator);
	}
}

bool AutoCompleterRef::isValid() const
{
	return proc != NULL || values != NULL;
}

// Classe per la ricerca del prefisso più lungo tra le stringhe che iniziano
// con un prefisso dato (utilizzata da autocomplete)
class MaximalPrefixFinder
{
	public:
		MaximalPrefixFinder(const char *filterPrefix)
		: filterPrefix(filterPrefix),
		  filterPrefixLength(strlen(filterPrefix)), hit_count(0)
		{
			extraChars[0] = '\0';
		}

		void process(const char *string)
		{
			// Ignora stringhe che iniziano con un prefisso diverso
			// da filterPrefix
			if (strncmp(string, filterPrefix, filterPrefixLength) != 0)
				return;

			const char *stringExtraChars = string + filterPrefixLength;
			int i = 0;
			if (hit_count == 0)
			{
				// Conserva testo fino al primo spazio
				extraChars[sizeof(extraChars) - 1] = '\0';
				strncpy(extraChars, stringExtraChars, sizeof(extraChars) - 1);
				while (extraChars[i] != '\0' && extraChars[i] != ' ')
					i++;
			}
			else
			{
				// Conserva porzione di prefisso comune (al massimo fino al primo spazio)
				while (extraChars[i] != '\0' && stringExtraChars[i] != ' '
					&& extraChars[i] == stringExtraChars[i])
					i++;
			}
			extraChars[i] = '\0';
			hit_count++;
		}

		int getNumHits() const { return hit_count; }
		const char* getMassimalPrefixExtraChars() const { return extraChars; }

	private:
		const char *filterPrefix;
		int filterPrefixLength;
		char extraChars[60];
		int hit_count;
};

void autocomplete(char *buffer, CompletionGenerator *generator)
{
	const char *text;

	// primo passaggio: determiniamo se il testo già immesso è ambiguo oppure no
	MaximalPrefixFinder pfxfinder(buffer);
	generator->rewind();
	while ((text = generator->next()) != NULL)
		pfxfinder.process(text);
	if (pfxfinder.getNumHits() == 0) // Nessun completamento possibile
	{
		return;
	}
	else
	{
		strcat(buffer, pfxfinder.getMassimalPrefixExtraChars());
		if (pfxfinder.getNumHits() == 1) // Completamento non ambiguo
		{
			strcat(buffer, " ");
			return;
		}
	}

	// secondo passaggio: se siamo qui vuol dire che il completamento è
	// ambiguo, ovvero c'è più di una possibilità. elenchiamole tutte
	int prefixLen = strlen(buffer);
	generator->rewind();
	while ((text = generator->next()) != NULL)
	{
		if (strncmp(buffer, text, prefixLen) == 0)
		{
			// il primo valore viene separato da \n, i successivi un semplice spazio
			putchar(autocompletion_ambiguous ? ' ' : '\n');
			autocompletion_ambiguous = true;

			// stampiamo l'opzione (fino al primo eventuale spazio)
			for (int i = 0; text[i] != '\0' && text[i] != ' '; i++)
				putchar(text[i]);
		}
	}
}

void enable_xbee_console()
{
	xbee_enabled = true;
	printf("ATTENZIONE: Console su XBee abilitata!\n");
}

void disable_xbee_console()
{
	xbee_enabled = true;
	printf("ATTENZIONE: Console su XBee disabilitata!\n");
	xbee_enabled = false;
}

// http://www.microchip.com/forums/m704703.aspx
extern "C" int __attribute__((__weak__, __section__(".libc")))
write(int handle, void *buffer, unsigned int len)
{
	serial2_send((char*)buffer, len);
	if (xbee_enabled)
		canstdio_send((char*)buffer, len);
	return len;
}
