/*
 * piccolo1_automation.cpp
 */

#include <stdio.h>
#include "fsm.h"
#include "servo.h"
#include "automation.h"

#define SENSOR_RIGHT_ON()         (PORTBbits.RB2 == 0)
#define SET_SENSOR_RIGHT_DIR()   { TRISBbits.TRISB2 = 1; }

#define SENSOR_LEFT_ON()         (PORTAbits.RA4 == 0)
#define SET_SENSOR_LEFT_DIR()   { TRISAbits.TRISA4 = 1; }

#define O_1()                   (PORTAbits.RA2)
#define O_2()                   (PORTAbits.RA3)
#define O_3()                   (PORTBbits.RB3)

#define SET_GPIO() { \
    TRISAbits.TRISA2 = 1;\
    TRISBbits.TRISB3 = 1;\
    TRISAbits.TRISA3 = 1;\
    }



// ------------------------------------------------------------------


void init_automation(void)
{
    SET_SENSOR_LEFT_DIR();
    SET_SENSOR_RIGHT_DIR();
    SET_GPIO();
}

void send_gpio(unsigned char board_num,
               unsigned char s1,
               unsigned char s2,
               unsigned char o1,
               unsigned char o2,
               unsigned char o3);

void automation(void)
{
    send_gpio(0, SENSOR_LEFT_ON(), SENSOR_RIGHT_ON(), O_1(), O_2(), O_3());
}

// API

void automation_command(const t_servo_position * pos)
{
}
