/*
 * adc.c
 */

#include "defines.h"

#include <xc.h>
#include <math.h>
#include <dsp.h>
#include <libq.h>

#include "adc.h"
#include "bus_interface.h"

#define uQ16(X) ((uint16_t) (65535*(X)))
#define ALPHA (0.58)

int adc_chan = 0;
uint16_t sensor[3];

uint16_t get_sensor(int channel)
{
	return sensor[channel];
}

void init_timer4()
{
	T4CONbits.TON = 0; // disabilito il timer4
	
	T4CONbits.TSIDL = 0;
	
	T4CONbits.T32 = 0; //32 bit mode off
	T4CONbits.TCS = 0;
	T4CONbits.TGATE = 0;
	
	T4CONbits.TCKPS = 0b10; // 1:64 prescaler
	
	TMR4 = 0;
	
	PR4 = 24192; // periodo_timer(s)*((Fosc)/(2*prescaler))
	
	IPC6bits.T4IP = 1; // priorita' a 1
	IEC1bits.T4IE = 1; // abilito Timer4 interrupt
	IFS1bits.T4IF = 0; // reset del dell' interrupt flag
	
	T4CONbits.TON = 1; // abilito il timer4
}

void init_adc(void)
{
    CORCONbits.IF = 1;
    PMD1bits.AD1MD = 0; // enable ADC

    // Imposta AN0-2 come input analogici
#if defined(MICROBRAIN_BOARD_2013)
    AD1PCFGLbits.PCFG0 = 0;
    AD1PCFGLbits.PCFG1 = 0;
    AD1PCFGLbits.PCFG2 = 0;
#elif defined(MICROBRAIN_BOARD_2015)
    ANSELAbits.ANSA0 = 1;
    ANSELAbits.ANSA1 = 1;
    ANSELBbits.ANSB0 = 1;
#endif
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISBbits.TRISB0 = 1;

    AD1CON1bits.ADON = 0;
    AD1CON1bits.ADDMABM = 1; // buffers written in sequential mode
    AD1CON1bits.AD12B = 1; // 12-bit
//     AD1CON1bits.FORM = 0b10; // fractional (left-just)
    AD1CON1bits.FORM = 0b00; // integer (right-just)
    AD1CON1bits.SSRC = 0b111; // auto-convert
    AD1CON1bits.ASAM = 0; // no auto-sample

    //AD1CON2bits.VCFG = 0b000; // AVdd & AVss
    AD1CON2bits.VCFG = 0b100; // AVdd & AVss
    AD1CON2bits.CSCNA = 0; // do not scan inputs
    AD1CON2bits.CHPS = 0b00; // converts CH0
    AD1CON2bits.SMPI = 0b0000; // interrupt after every conversion
    AD1CON2bits.ALTS = 0;
    AD1CON2bits.BUFM = 0;

    AD1CON3bits.ADRC = 1; // clock from RC clock
    AD1CON3bits.SAMC = 0b11111;
    AD1CON3bits.ADCS = 0b00111111; // slowest clock

    AD1CON4bits.DMABL = 0;

    AD1CSSL = 0b000000000000001; // scan AN0 to AN5

    adc_chan = 0;

    AD1CHS0bits.CH0NA = 0;
    AD1CHS0bits.CH0SA = adc_chan;

    IFS0bits.AD1IF = 0 ; 	// clear the AD interrupt
    IPC3bits.AD1IP = 5 ;     // priority 5
    IEC0bits.AD1IE = 1 ;     // enable the interrupt

    AD1CON1bits.ADON = 1 ;	// turn on the A to D

    AD1CON1bits.SAMP = 1;

    init_timer4();
}


#if defined(MICROBRAIN_BOARD_2013)
extern "C" void __attribute__((__interrupt__,__no_auto_psv__)) _ADC1Interrupt(void)
#elif defined(MICROBRAIN_BOARD_2015)
extern "C" void __attribute__((__interrupt__,__no_auto_psv__)) _AD1Interrupt(void)
#endif
{
	sensor[adc_chan] = (uint16_t) (_Q16mpy(uQ16(ALPHA), sensor[adc_chan]) + _Q16mpy(uQ16(1 - ALPHA), ADC1BUF0));
	
	adc_chan = (adc_chan + 1) % 3;

	AD1CHS0bits.CH0SA = adc_chan;
	IFS0bits.AD1IF = 0 ;		// clear the AD interrupt
	if(adc_chan != 0)
		AD1CON1bits.SAMP = 1;	// restart sampling
}

extern "C" void __attribute__((__interrupt__,__no_auto_psv__)) _T4Interrupt(void)
{
	IFS1bits.T4IF = 0;
	IFS0bits.AD1IF = 0;	// clear the AD interrupt
	AD1CON1bits.SAMP = 1;	// restart sampling
}
