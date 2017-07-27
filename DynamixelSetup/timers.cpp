/*
 * timers.c
 */



#include "defines.h"

#include <p33FJ128MC802.h>

#include "timers.h"

// TIMER 2 IS USED FOR UART TIMEOUT

#define USEC_TO_TIM2(v)   (int)(v / 1.65)

void timer2_init(void)
{

    T2CONbits.TON = 0; // disabilito il timer2

    T2CONbits.TSIDL = 0 ; //non ci fermiamo in idle mode
    T2CONbits.T32 = 0 ; //timer a 16 bit
    T2CONbits.TCS = 0; // clock interno Fosc/2
    T2CONbits.TGATE = 0 ; //disabilito la modalita' gated timer

    T2CONbits.TCKPS = 0b10; // 1:64 prescaler

    // clock increment = (77385000/2)/64, period of 1.65 microsecs (approx)

    TMR2 =  0;
    PR2 = 0xffff; // periodo_timer(s)*((Fosc)/(2*prescaler)) :abbiamo un timer a circa 5ms


    IPC1bits.T2IP = 1;	// priorita' a 1
    IEC0bits.T2IE = 0;    // disabilito Timer2 interrupt
    IFS0bits.T2IF = 0; 	// reset del dell' interrupt flag

}


void timer2_start(int usec_val)
{
    T2CONbits.TON = 0;

    TMR2 = 0;
    PR2 = USEC_TO_TIM2(usec_val);

    IFS0bits.T2IF = 0;

    T2CONbits.TON = 1;
}


bool timer2_elapsed(void)
{
    return IFS0bits.T2IF == 1;
}


// TIMER 4 IS USED FOR SCHEDULING
void timer4_init(void)
{

    T4CONbits.TON = 0; // disabilito il timer4

    T4CONbits.TSIDL = 0 ; //non ci fermiamo in idle mode
    T4CONbits.T32 = 0 ; //timer a 16 bit
    T4CONbits.TCS = 0; // clock interno Fosc/2
    T4CONbits.TGATE = 0 ; //disabilito la modalita' gated timer

    T4CONbits.TCKPS = 0b10; // 1:64 prescaler

    TMR4 =  0;
    PR4 = 3023; // periodo_timer(s)*((Fosc)/(2*prescaler)) :abbiamo un timer a circa 5ms


    IPC6bits.T4IP = 2;	// priorita' a 2
    IEC1bits.T4IE = 0;    // disabilito Timer4 interrupt
    IFS1bits.T4IF = 0; 	// reset del dell' interrupt flag

    T4CONbits.TON = 1; 	// abilito il timer4		

}

bool timer4_elapsed(void)
{
    bool v = (IFS1bits.T4IF == 1);
    if (v) {
        IFS1bits.T4IF = 0;
        return true;
    }
    else
        return false;
}


