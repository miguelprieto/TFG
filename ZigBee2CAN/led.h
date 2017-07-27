#ifndef LED_H
#define LED_H

#include <xc.h>
#include "defines.h"

#define led_initialize() { TRISBbits.TRISB3 = 0; TRISBbits.TRISB12 = 0; }

#define red_led_on()  LATBbits.LATB3 = 0
#define red_led_off()  LATBbits.LATB3 = 1
#define red_led_toggle()  LATBbits.LATB3 = !LATBbits.LATB3

#define green_led_on()  LATBbits.LATB12 = 0
#define green_led_off()  LATBbits.LATB12 = 1
#define green_led_toggle()  LATBbits.LATB12 = !LATBbits.LATB12
#endif
