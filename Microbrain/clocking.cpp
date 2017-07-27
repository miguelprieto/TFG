/*
 * clocking.c
 */

#include <xc.h>

#include "defines.h"
#include "clocking.h"

void init_clock()
{
    OSCTUNbits.TUN = 0;

    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBD = 40;           // M = 42
    CLKDIVbits.PLLPOST=0; // N2 = 2
    CLKDIVbits.PLLPRE=0;   // N1 = 2

    // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001) ;

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) ;

    //
    // clock = 7.37 * (40 + 2) / ( 2 * 2 ) = 77.385
    //

}
