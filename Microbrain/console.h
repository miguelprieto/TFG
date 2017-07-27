#ifndef __CONSOLE_H
#define __CONSOLE_H

enum escape_sequence
{
	UNKNOWN_ESCAPE_SEQUENCE,
	KEY_ESC,
	KEY_UP_ARROW,
	KEY_DOWN_ARROW,
	KEY_LEFT_ARROW,
	KEY_RIGHT_ARROW
};

// Versione non bloccante della read_line
void begin_read_line(char *s); // da richiamare per ogni riga prima di continue_read_line
bool continue_read_line(); // da richiamare continuamente, restituisce true se è stata completata l'immissione della stringa
void read_line(char * s);

int kbhit();

/* read_char() restituisce il carattere letto. Se viene restituito il carattere
 * di escape '\e' (ovvero 0x1b), richiamare read_escape_sequence() per
 * decodificare la restante parte della sequenza di escape */
char read_char();
enum escape_sequence read_escape_sequence();

/* Autocompletion callback (definita in main.c)
 * Viene richiamata alla pressione del tasto TAB */
void autocompletion_callback(char *linebuffer);

#define AUTOCOMPLETE_LIST(...) AutoCompleterRef((const char *const[]){__VA_ARGS__})
class AutoCompleterRef
{
	public:
		typedef void (*proc_autocomplete)(char *buffer, int argidx);

		// NULL autocompleter [isValid() == false]
		explicit AutoCompleterRef();

		// procedura di completamento (oppure NULL)
		AutoCompleterRef(proc_autocomplete proc);

		// liste NULL-terminate di completamenti per ciascun parametro, concatenate tra loro
		// es: AUTOCOMPLETE_LIST("esempioA_per_arg1", "esempioA_per_arg2", NULL, "esempioA_per_arg2", "esempioA_per_arg2", NULL)
		explicit AutoCompleterRef(const char *const values[]);

		void operator()(char *buffer, int argidx) const;
		bool isValid() const;

	private:
		proc_autocomplete proc;
		const char * const *values;
};

// Classe base per generazione dei possibili completamenti automatici
class CompletionGenerator
{
	public:
		virtual void rewind() = 0;
		virtual const char* next() = 0;
};

/* Dato un insieme di possibili valori, effettua il completamento automatico
 * del buffer. Può essere richiamata solo durante l'autocompletion callback
 * (v. funzione autocompletion_callback) */
void autocomplete(char *buffer, CompletionGenerator *generator);

void enable_xbee_console();
void disable_xbee_console();

#endif
