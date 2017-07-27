#include "defines.h"

#include <xc.h>
#include <libpic30++.h>
#include <stdio.h>
#include <math.h>

#include "console.h"
#include "pwm.h"

/* Frequenza a cui opera il modulo PWM. */
#define FPWM			24000 // 24 kHz

void pwm_init()
{
	// Calcola e imposta periodo (assumiamo prescaler 1:1)
	PTPER = (FCY/FPWM - 1) / 2;

	PDC1 = 0;
	IOCON1bits.PMOD = 0b01; // Redundant output mode

	PDC2 = 0;
	IOCON2bits.PMOD = 0b01; // Redundant output mode

	PDC3 = 0;
	IOCON3bits.PMOD = 0b01; // Redundant output mode

	// Enable PWM2L (RB13) output
	TRISBbits.TRISB13 = 0;
	IOCON2bits.POLL = 0; // active-high

	IOCON1bits.PENL = 0; // PWM1L - controlled by I/O module
	IOCON1bits.PENH = 0; // PWM1H - controlled by I/O module
	IOCON2bits.PENL = 1; // PWM2L - controlled by PWM module
	IOCON2bits.PENH = 0; // PWM2H - controlled by I/O module
	IOCON3bits.PENL = 0; // PWM3L - controlled by I/O module
	IOCON3bits.PENH = 0; // PWM3H - controlled by I/O module

	// Enable PWM
	PTCONbits.PTEN = 1;
}

void pwm_set_duty_cycle(int num, float dc)
{
	int val = PTPER * dc;

	switch (num)
	{
		case 1:
			PDC1 = val;
			break;
		case 2:
			PDC2 = val;
			break;
		case 3:
			PDC3 = val;
			break;
	}
}

static void __attribute__((used)) pwm_tone(int freq)
{
	PTCONbits.PTEN = 0;
	PTCONbits.PTEN = 1;
	IOCON2bits.OVRENL = 0;
	PTPER = (FCY/freq - 1) / 2;
	PDC2 = PTPER / 20;
}

static void  __attribute__((used)) pwm_mute()
{
	PTCONbits.PTEN = 0;
	PTCONbits.PTEN = 1;
	IOCON2bits.OVRDAT = 0b00;
	IOCON2bits.OVRENL = 1;
}

#define C4 261
#define D4 293
#define E4 329
#define F4 349
#define G4 392
#define A4 440
#define B4 493
#define C5 523
#define D5 587
#define E5 659
#define F5 698
#define G5 783

// Nota: Premere qualsiasi tasto per terminare immediatamente
void pwm_melody()
{
#define DELAY_OR_FAST_FORWARD(ms) do { __delay_ms(ms); if (kbhit()) { (void)read_char(); goto done; } } while (0)
	pwm_init();

	PTCONbits.EIPU = 1;
	PWMCON1bits.IUE = 1;

	printf("Stai sentendo anche tu, da dove viene? :O\n");

#if 0
	for (int i = 0; i < 2; i++)
	{
		pwm_tone(G4);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(A4);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(B4);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(G4);
		DELAY_OR_FAST_FORWARD(300);
		pwm_mute();
		DELAY_OR_FAST_FORWARD(200);
	}

	for (int i = 0; i < 2; i++)
	{
		pwm_tone(B4);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(C5);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(D5);
		DELAY_OR_FAST_FORWARD(800);
		pwm_mute();
		DELAY_OR_FAST_FORWARD(200);
	}

	for (int i = 0; i < 2; i++)
	{
		pwm_tone(D5);
		DELAY_OR_FAST_FORWARD(250);
		pwm_tone(E5);
		DELAY_OR_FAST_FORWARD(250);
		pwm_tone(D5);
		DELAY_OR_FAST_FORWARD(250);
		pwm_tone(C5);
		DELAY_OR_FAST_FORWARD(250);
		pwm_tone(B4);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(G4);
		DELAY_OR_FAST_FORWARD(500);
	}

	for (int i = 0; i < 2; i++)
	{
		pwm_tone(A4);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(D5);
		DELAY_OR_FAST_FORWARD(500);
		pwm_tone(G4);
		DELAY_OR_FAST_FORWARD(1000);
	}
#else
	pwm_tone(G4);
	DELAY_OR_FAST_FORWARD(200);
	pwm_tone(B4);
	DELAY_OR_FAST_FORWARD(200);
	pwm_tone(D5);
	DELAY_OR_FAST_FORWARD(200);
	pwm_tone(G5);
	DELAY_OR_FAST_FORWARD(400);
	pwm_tone(D5);
	DELAY_OR_FAST_FORWARD(200);
	pwm_tone(G5);
	DELAY_OR_FAST_FORWARD(400);
#endif

done:
	PTCONbits.EIPU = 0;
	PWMCON1bits.IUE = 0;
	IOCON2bits.OVRENL = 0;
	PTCONbits.PTEN = 0;

	printf("Adesso non si sente piu' nulla... :(\n");

	pwm_init();
}
