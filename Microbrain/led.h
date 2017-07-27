/*
 * led.h
 */

#ifndef __LED_H
#define __LED_H

#include "defines.h"

#define set_led() { TRISAbits.TRISA4 = 0; }

#define led_on()  LATAbits.LATA4 = 0
#define led_off()  LATAbits.LATA4 = 1
#define led_toggle()  LATAbits.LATA4 = !LATAbits.LATA4

#endif
