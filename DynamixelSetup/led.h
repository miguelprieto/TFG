/*
 * led.h
 */

#ifndef __LED_H
#define __LED_H

#include "defines.h"

#define set_led() { TRISBbits.TRISB15 = 0; }

#define led_on()  LATBbits.LATB15 = 1
#define led_off()  LATBbits.LATB15 = 0
#define led_toggle()  LATBbits.LATB15 = !LATBbits.LATB15

#endif
