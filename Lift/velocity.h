/*
 * velocity.h
 */


#ifndef __VELOCITY_H
#define __VELOCITY_H

#include "pwm_control.h"
#include "controller.h"
#include "timers.h"
#include "math.h"

#define SATURATION 			MAX_PWM*0.96				//soglia di saturazione per il pwm
#define TO_RPM 				9.55						//fattore per passare da rad/s a rpm
#define V_CHECK_TIME		2							//parametro a moltiplicare il PERTMR2 per parametrizzare la temporizzazione del controllo in velocita'
#define V_TIMER_INTERVAL	(PERTMR2*V_CHECK_TIME) 		//tempo controllo in velocita'


typedef struct {
    float current_speed;
    float target_speed;
    float position;
    int ticks;
    int on;
} velocity;


void init_velocity_controllers(void);
void reset_velocity_controllers(void);
void read_velocity(void);
void velocity_control(void);
float get_position(int axis);
void set_position(int axis, float pos);
void set_speed(int axis, float speed);
float get_speed(int axis);
void axis_off(int axis);


#endif
