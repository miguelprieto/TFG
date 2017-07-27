/*
 * File:   servo.h
 *
 * Functions to manage servos.
 *
 */

#ifndef __SERVO_H
#define __SERVO_H

#include <p33FJ128MC802.h>
#include "servo_pins.h"

#define NUM_SERVO 10

#define SERVO_MIN_TIME 100

#define SERVO_MAX_TIME 1400


typedef struct {
    int do_update;
    unsigned int current_time;
    unsigned int desired_time;
} t_servo_time;

#ifdef SERVO_INTERRUPT

#define SET_SERVO_TIME(s,t) {\
    while (servo_time[s].do_update == 1) ; \
    servo_time[s].desired_time = t; \
    servo_time[s].do_update = 1; \
    }

#define GET_SERVO_TIME(s) servo_time[s].current_time

#define GET_AND_UPDATE_SERVO_TIME(s,d) {              \
        t_servo_time * st;                            \
        st = &servo_time[s];                          \
        if (st->do_update) {                          \
            st->current_time = st->desired_time;              \
            st->do_update = 0;                                          \
        }                                                               \
        d = st->current_time;                                           \
    }

#else

#define SET_SERVO_TIME(s,t) {\
    servo_time[s].desired_time = t; \
    servo_time[s].do_update = 1; \
    }

#define GET_SERVO_TIME(s) servo_time[s].current_time

#define GET_AND_UPDATE_SERVO_TIME(s,d) {              \
        t_servo_time * st;                            \
        st = &servo_time[s];                          \
        if (st->do_update) {                          \
            st->current_time = st->desired_time;              \
            st->do_update = 0;                                          \
        }                                                               \
        d = st->current_time;                                           \
    }

#endif

//void initialize_servo(int interrupt_enable);
void update_servos(void);
void initialize_servo(void);
void timer_interrupt(void);
void set_servo(int servo_num, int value);



extern volatile unsigned char wait_counter;

extern volatile unsigned char servo_turn;


#endif




