/*
 * led.h
 */

#ifndef __LED_H
#define __LED_H

#include "defines.h"

#define set_led() { TRISAbits.TRISA1 = PORT_OUT; ODCAbits.ODCA1 = 0; }

#define led_on()  LATAbits.LATA1 = 0
#define led_off()  LATAbits.LATA1 = 1

#define led_toggle() LATAbits.LATA1 = !LATAbits.LATA1

#endif
