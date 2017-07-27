#ifndef UART_H
#define UART_H

#include "defines.h"

#ifdef EXTERNAL_CLOCK
#define B115200 20  // ???
#endif

#ifdef INTERNAL_CLOCK
#define B9600   251
#define B19200  125
#define B38400  62
#define B57600  41
#define B115200 20
#endif

void uart1_initialize(uint16_t baud_rate, uint8_t interrupt);
void uart1_start();
void uart1_stop();
void uart1_send(uint8_t data);
void uart2_initialize(uint16_t baud_rate, uint8_t interrupt);
void uart2_start();
void uart2_stop();
void uart2_send(uint8_t data);
#endif