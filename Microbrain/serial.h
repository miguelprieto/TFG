/*
 *  serial.h
 */

#ifndef __SERIAL_H
#define __SERIAL_H

#include "defines.h"

#include <stdbool.h>

#define B115200         20

void serial2_start(int use_interrupt, int baud_rate);
void serial2_stop();

/* Controlla presenza di almeno un carattere da UART2 (non bloccante) */
bool serial2_is_input_available();

/* Riceve carattere da UART2 (bloccante / non bloccante) */
char serial2_getcharacter(bool blocking);

/* Invia uno o più caratteri a UART2 */
void serial2_send(const char *data, int data_len);

#endif
