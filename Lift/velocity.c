/*
 * velocity.c
 */

#include "velocity.h"
#include "position.h"

#include <p33FJ128MC802.h>
#include <libpic30.h>

controller	 vel_pid[2];
velocity         vel[2];

#define KP    80.0
#define KI    100.0
#define KD    0.5

void init_velocity_controllers(void)
{
    pid_init(&vel_pid[0], KP, KI, KD, U_TIMER_INTERVAL, 1, SATURATION);
    pid_init(&vel_pid[1], KP, KI, KD, U_TIMER_INTERVAL, 1, SATURATION);

    reset_velocity_controllers();
}


void reset_velocity_controllers(void)
{
    pid_reset(&vel_pid[0]);
    pid_reset(&vel_pid[1]);
    set_pwm(0, 0);
    set_pwm(0, 1);

    vel[0].target_speed = 0;
    vel[0].ticks = 0;
    vel[0].position = 0;
    vel[0].on = 0;

    vel[1].target_speed = 0;
    vel[1].ticks = 0;
    vel[1].position = 0;
    vel[1].on = 0;

    POS1CNT = 0;
    POS2CNT = 0;
}


void read_velocity(void)
{
    int poscount[2];
    int i;

    poscount[0] = POS1CNT;
    poscount[1] = POS2CNT;

    for (i = 0;i < 2;i++) {
        // update speed & position
        float delta = TICKS_TO_MM((float)(poscount[i] - vel[i].ticks));
        vel[i].current_speed = delta / U_TIMER_INTERVAL;
        vel[i].ticks = poscount[i];
        vel[i].position += delta;
    }
}


void velocity_control(void)
{
    int i;
    for (i = 0;i < 2;i++) {
        if (vel[i].on) {
            float out = pid_evaluation(&vel_pid[i], vel[i].target_speed, vel[i].current_speed);
            set_pwm(i, out);
        }
    }
}


inline float get_position(int axis)
{
    return vel[axis].position;
}

inline void set_position(int axis, float pos)
{
    vel[axis].position = pos;
}

inline float get_speed(int axis)
{
    return vel[axis].current_speed;
}

inline void set_speed(int axis, float speed)
{
    vel[axis].target_speed = speed;
    vel[axis].on = 1;
}

inline void axis_off(int axis)
{
    vel[axis].on = 0;
    set_pwm(axis, 0);
}
