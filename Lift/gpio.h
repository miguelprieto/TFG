/*
 * gpio.h
 */

#ifndef __GPIO_H
#define __GPIO_H

#include "defines.h"

#ifdef BOARD_VERSION_2

// LED = RB5, Push-pull output, 0 = LED_ON, 1 = LED_OFF
// BUMP_1 = RB4
// BUMP_2 = RA4

#define init_gpio() {                           \
        TRISBbits.TRISB5 = PORT_OUT;            \
        TRISBbits.TRISB4 = PORT_IN;             \
        TRISAbits.TRISA4 = PORT_IN;             \
                                                \
        ODCBbits.ODCB5 = 0;                     \
                                                \
        LATBbits.LATB5 = 1;                     \
    }


#define led_on()    LATBbits.LATB5 = 0
#define led_off()   LATBbits.LATB5 = 1
#define led_toggle()   LATBbits.LATB5 = !LATBbits.LATB5

#define bumper_pin     PORTAbits.RA4

#else

// GPIO0 = RB4, pin 11, Push-pull output
// GPIO1 = RA4, pin 12, LED, Push-pull output, 0 = LED_ON, 1 = LED_OFF

#define init_gpio() {                           \
        TRISBbits.TRISB4 = PORT_OUT;            \
        TRISAbits.TRISA4 = PORT_OUT;            \
                                                \
        ODCBbits.ODCB4 = 0;                     \
        ODCAbits.ODCA4 = 0;                     \
                                                \
        LATBbits.LATB4 = 0;                     \
        LATAbits.LATA4 = 1;                     \
    }


#define led_on()    LATAbits.LATA4 = 1
#define led_off()   LATAbits.LATA4 = 0
#define led_toggle()   LATAbits.LATA4 = !LATAbits.LATA4

#define gpio_toggle()   LATBbits.LATB4 = 1 - LATBbits.LATB4
#define gpio(v)         LATBbits.LATB4 = v

#endif

#endif
