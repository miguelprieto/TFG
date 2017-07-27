/*
 * position.h
 */
#ifndef __POSITION_H
#define __POSITION_H

#include "controller.h"

void init_position(void);
void machine(int i);
long get_position_1(void);
long get_position_2(void);
void set_target(int axis, float pos);
int get_pid_status(int axis);

extern controller axis[2];
extern float target[2];
extern char pid_status[2];
extern int r_sampling[2];

enum {
    PID_OFF = 0,
    DO_RESET,
    RESETTING,
    PID_ON
};

#define LIMIT_ONE   PORTBbits.RB4
#define LIMIT_TWO   PORTAbits.RA4

// 400 ticks per revolution
// 1 revolution = 2mm
#define TICKS_MM_FACTOR     200


#define TICKS_TO_MM(t)   ((float)(t)/TICKS_MM_FACTOR)
#define MM_TO_TICKS(m)   ((float)(m)*TICKS_MM_FACTOR)



#endif
