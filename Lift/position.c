/*
 * position.c
 */

#include "defines.h"

#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <stdio.h>

#include "clocking.h"
#include "gpio.h"
#include "pwm_control.h"
#include "controller.h"
#include "timers.h"
#include "position.h"
#include "velocity.h"

controller axis[2];
float target[2];
char pid_status[2];
float next_speed[2];

int r_sampling[2];

#define V_MAX 			50  // mm/s
#define ACCEL         50
#define DECEL         30

#define POSITION_THRESHOLD  3
#define MIN_SPEED           10  // mm/s


float m_decel_distance, m_accel_step;


#define KP    5.0
#define KI    0.0
#define KD    0.0

int get_pid_status(int axis)
{
    return pid_status[axis];
}

/*
 * left motor:
 *   encoder: increase down
 *
 * Right motor:
 *   encoder: increase down
 */

void init_position(void)
{
    pid_init(&axis[0], KP, KI, KD, P_TIMER_INTERVAL, 1, V_MAX);
    pid_init(&axis[1], KP, KI, KD, P_TIMER_INTERVAL, 1, V_MAX);
    pid_status[0] = pid_status[1] = PID_OFF;

    TRISBbits.TRISB4 = 1; // channel 0
    TRISAbits.TRISA4 = 1; // channel 1

    m_accel_step = ACCEL * 0.040; // 40ms
    m_decel_distance = (V_MAX * V_MAX) / (2 * DECEL);

    next_speed[0] = next_speed[1] = 0;
}


#define HOME_SPEED    15

void set_target(int axis, float pos)
{
    pid_status[axis] = PID_ON;
    target[axis] = pos;
    next_speed[axis] = 0;
}

void do_reset(int i)
{
    set_speed(i, HOME_SPEED);

    switch (i) {
    case 0:
        if (LIMIT_ONE == 0)
            pid_status[i] = RESETTING;
        break;
    case 1:
        if (LIMIT_TWO == 0)
            pid_status[i] = RESETTING;
        break;
    }

    /* int pos; */
    /* static int lastpos[2]; */

    /* if (r_sampling[i] != 0) */
    /* { */
    /*     pos = get_position(i); */
    /*     if (lastpos[i] == pos) */
    /*         r_sampling[i]--; */
    /*     else */
    /*         r_sampling[i] = 10; */

    /*     lastpos[i] = pos; */
    /* } */
    /* else */
    /*     pid_status[i] = RESETTING; */
}


void resetting(int i)
{
    pid_status[i] = PID_ON;
    pid_reset(&axis[i]);
    set_position(i, 0);
    set_target(i, -10); // initial target to 10 mm
}

#define sgn(x) ( (x < 0) ? -1 : 1)




void do_control(int i)
{
    float out, current, distance, current_speed;

    //c = &axis[i];
    current = get_position(i);
    current_speed = get_speed(i);

    distance = target[i] - current;

    if (fabs(distance) >= POSITION_THRESHOLD) {

        float s;

        if (distance < 0) {
            s = -1;
            distance = -distance;
        }
        else
            s = 1;

        if (distance < m_decel_distance) {
            // fase 3
            float expected_speed = sqrt(V_MAX * V_MAX - 2 * DECEL * (m_decel_distance - distance));
            if (expected_speed > fabs(current_speed)) {
                // la vel da raggiungere risulta MAGGIORE della vel corrente
                // questo vuol dire che siamo ancora nella fase di accelerazione
                // pertanto acceleriamo
                next_speed[i] += m_accel_step;
                if (next_speed[i] > expected_speed) next_speed[i] = expected_speed;
                if (next_speed[i] > V_MAX) next_speed[i] = V_MAX;
            }
            else {
                next_speed[i] = expected_speed;
            }
        }
        else {
            // Fase 1, fase 2
            if (next_speed[i] < V_MAX) {
                next_speed[i] += m_accel_step;
                if (next_speed[i] > V_MAX) next_speed[i] = V_MAX;
            }
        }

        if (next_speed[i] < MIN_SPEED)
            out = s*MIN_SPEED;
        else
            out = s*next_speed[i];

    }
    else
        out = 0;

    set_speed(i, out);
}


void machine(int i)
{
    switch (pid_status[i]) {
    case PID_OFF:
        break;
    case DO_RESET:
	do_reset(i);
        break;
    case RESETTING:
        resetting(i);
        break;
    case PID_ON:
        do_control(i);
        break;
    }
}
