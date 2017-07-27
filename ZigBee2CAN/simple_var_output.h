#ifndef SIMPLE_VAR_OUTPUT_H
#define SIMPLE_VAR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

/* Inzializzazione strutture dati */
void simple_var_output_init();

/* Gestore frame CAN */
void simple_var_output_process_can_frame(const uint8_t *data, unsigned int len, void *user_ptr);

/* Viene invocato più spesso possibile */
void simple_var_output_relax();

#endif /* SIMPLE_VAR_OUTPUT_H */
