/*
 * File:   servo.cpp
 *
 * Functions to manage servos.
 *
 */

#include "defines.h"
#include <p33FJ128MC802.h>
#include "servo.h"
#include "servo_pins.h"
#include "led.h"

volatile unsigned char servo_turn = 0;
t_servo_time servo_time[NUM_SERVO];
unsigned int up = 0;

#define PWM_PERIOD_20      12104
// ~20 millisecond period (50 Hz) at 77.385 MHz clock, prescaler = 64

#define PERIOD  (PWM_PERIOD_20 / 8)

volatile unsigned char wait_counter;

int map[8] = {1, 3, 5, 7, 8, 9, 0, 0};

void update_servos(void)
{

    int duration, servo_index;
    unsigned int tm;

    led_toggle();

    IFS0bits.T2IF = 0;
    T2CONbits.TON = 0;

    servo_index = map[servo_turn];

    if (servo_index == 0) {
        tm = 0;
    }
    else {
        if (up == 0) {
            GET_AND_UPDATE_SERVO_TIME(servo_index, tm);
        } else {
            tm = GET_SERVO_TIME(servo_index);
        }
    }

    if (tm != 0) {
        up = !up;
    }
    switch (servo_index) {
        case 1: BIT_SERVO1 = up;
            break;
        case 3: BIT_SERVO3 = up;
            break;
        case 5: BIT_SERVO5 = up;
            break;
        case 7: BIT_SERVO7 = up;
            break;
        case 8: BIT_SERVO8 = up;
            break;
        case 9: BIT_SERVO9 = up;
            break;
    }

    if (up) {
        duration = tm;
        //led_on();
    } else {
        duration = PERIOD - tm;
        servo_turn = (servo_turn + 1) % 8;
        if (servo_turn == 0)
            wait_counter ++;
    }

    PR2 = duration;
    TMR2 = 0x00;
	
    T2CONbits.TON = 1;

}

/* Timer1 ISR */

#ifdef SERVO_INTERRUPT
/**
 * Function that handle the interrupt of the timer
 */
extern "C" void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    update_servos();
}


#endif

void initialize_servo(void)
{
    int i;

    for (i = 0; i < NUM_SERVO; i++) {
        servo_time[i].do_update = 0;
        SET_SERVO_TIME(i, 0);
    }
	
    // ---------------------- PWM 1

    P1TPER = PWM_PERIOD_20;

    P1TCONbits.PTCKPS = 0b11;	// prescaler = 64
    P1TCONbits.PTOPS = 0;	// no postscaler
    P1TCONbits.PTMOD = 0b00;	// free run

    PWM1CON1bits.PMOD1 = 1 ; // independent PWM mode
    PWM1CON1bits.PMOD2 = 1 ;
    PWM1CON1bits.PMOD3 = 1 ;

    PWM1CON1bits.PEN1L = 1 ; // low pins used as PWM
    PWM1CON1bits.PEN2L = 1 ;
    PWM1CON1bits.PEN3L = 1 ;

    PWM1CON1bits.PEN1H = 0 ; // high pins used as digital I/O
    PWM1CON1bits.PEN2H = 0 ;
    PWM1CON1bits.PEN3H = 0 ;

    P1DC1 = P1DC2 = P1DC3 = 0;

    IFS3bits.PWM1IF = 0;
    IEC3bits.PWM1IE = 0;
    //IPC14bits.PWM1IP = 3; // priority 3


    // ---------------------- PWM 2

    P2TPER = PWM_PERIOD_20;

    P2TCONbits.PTCKPS = 0b11;	// prescaler = 64
    P2TCONbits.PTOPS = 0;	// no postscaler
    P2TCONbits.PTMOD = 0b00;	// free run

    PWM2CON1bits.PMOD1 = 1 ; // independent PWM mode

    PWM2CON1bits.PEN1L = 1 ; // low pins used as PWM

    PWM2CON1bits.PEN1H = 0 ; // high pins used as digital I/O

    P2DC1 = 0;

    IFS4bits.PWM2IF = 0;
    IEC4bits.PWM2IE = 0;
    //IPC14bits.PWM1IP = 3; // priority 3

    // -------------- timer 2 for 4th software-based pwm channel

    T2CONbits.TON = 0;
    T2CONbits.TSIDL = 1;
    T2CONbits.TGATE = 0;
    T2CONbits.TCKPS = 0b10; // x 64 prescaler
    T2CONbits.T32 = 0; // 16 bit timer

    TMR2 = 0;
    PR2 = PWM_PERIOD_20;
    IFS0bits.T2IF = 0;
    IPC1bits.T2IP = 3; // priority 3
    IEC0bits.T2IE = 0;

    P1TCONbits.PTEN = 1; 	// turn on the PWM 1
    P2TCONbits.PTEN = 1; 	// turn on the PWM 2

    T2CONbits.TON = 1;

}


void set_servo(int servo_num, int value)
{
    if (value != 0) value += SERVO_MIN_TIME;

    SET_SERVO_TIME(servo_num, value);

    switch (servo_num) {
    case 0: P1DC1 = value*2; break;
    case 2: P1DC2 = value*2; break;
    case 4: P1DC3 = value*2; break;
    case 6: P2DC1 = value*2; break;
    }
}

