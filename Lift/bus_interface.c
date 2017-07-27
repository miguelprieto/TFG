 /*
  * bus_interface.c
  */

#include "defines.h"

#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bus_interface.h"
#include "bus_objects.h"
#include "canstdio_endpoint.h"
#include "ecan_lib.h"
#include "gpio.h"
#include "position.h"
#include "velocity.h"
#include "pwm_control.h"


#define M_E(m,e)   (float)m * pow(10,e)

int telemetry_flag = 1;

void update_motion_command(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_motion_command * m = (const t_can_motion_command*)data;

    switch (m->_cmd) {

    case POSITION_PID_1:
        {
            float kp, ki, kd;
            t_command_speed_pid * p = (t_command_speed_pid *)m;
				
            kp = M_E(p->kp_m, p->kp_e);
            ki = M_E(p->ki_m, p->ki_e);
            kd = M_E(p->kd_m, p->kd_e);

            axis[0].Kp = kp;
            axis[0].Ki = ki;
            axis[0].Kd = kd;

            pid_reset(&axis[0]);

        }
        break;

    case POSITION_PID_2:
        {
            float kp, ki, kd;
            t_command_speed_pid * p = (t_command_speed_pid *)m;
				
            kp = M_E(p->kp_m, p->kp_e);
            ki = M_E(p->ki_m, p->ki_e);
            kd = M_E(p->kd_m, p->kd_e);

            axis[1].Kp = kp;
            axis[1].Ki = ki;
            axis[1].Kd = kd;

            pid_reset(&axis[1]);

        }
        break;
    case POSITION_OFF:
        {
            t_command_position_generic * p = (t_command_position_generic *)m;
            if (p->axis == 0) pid_status[0] = PID_OFF;
            if (p->axis == 1) pid_status[1] = PID_OFF;
            axis_off(p->axis);
            set_pwm(p->axis, 0);
            break;
        }				
    case POSITION_HOME:
        {
            t_command_position_generic * p = (t_command_position_generic *)m;
	    if (p->axis == 0) r_sampling[0] = 10, pid_status[0] = DO_RESET;
	    if (p->axis == 1) r_sampling[1] = 10, pid_status[1] = DO_RESET;
            break;
        }				
    case POSITION_GO:
        {
            t_command_position_generic * p = (t_command_position_generic *)m;
            set_target(p->axis, p->position);
            break;
        }				
    case POSITION_TELEMETRY:
        {
            t_command_position_generic * p = (t_command_position_generic *)m;
            telemetry_flag = p->axis;
            break;
        }	
    case POSITION_PWM:
        {
            t_command_position_generic * p = (t_command_position_generic *)m;
            pid_status[p->axis] = PID_OFF;
            axis_off(p->axis);
            set_pwm(p->axis, p->position);
            break;
        }				
    case POSITION_SPEED:
        {
            t_command_position_generic * p = (t_command_position_generic *)m;
            set_speed(p->axis, p->position);
            break;
        }				
    }
}


#define POSITION_COMMAND_OBJECT	0
#define REMOTE_STDIO_RX_OBJECT	1

void send_can_objects(void)
{
    t_can_lift_telemetry_encoder_data d;

    static int turn = 1;

    if (telemetry_flag == 1) {

        if (turn == 1 || 1 /* NOTE-2014: Noi usiamo un solo asse, quindi evitiamo di trasmettere info sul secondo */) {

            led_on();

            d.frame_id = 0x10;
            d.sub_id = 0x01;
            d.bumper = (LIMIT_ONE << 8) | get_pid_status(0);
            d.value = get_position(0);
            ecan_send(LIFT_TELEMETRY_DATA_CAN_ID, (unsigned char *)&d, 8, 0);
            turn = 2;
        }
        else {
            d.frame_id = 0x10;
            d.sub_id = 0x02;
            d.bumper =  (LIMIT_ONE << 8) | get_pid_status(1);
            d.value = get_position(1);
            ecan_send(LIFT_TELEMETRY_DATA_CAN_ID, (unsigned char *)&d, 8, 0);
            turn = 1;
        }
    }
}


void check_can_objects(void)
{
    ecan_update_object(POSITION_COMMAND_OBJECT);
}


void init_bus_objects(void)
{
#ifdef BOARD_VERSION_2
    // RX is RP6, TX is RP7 in open_drain configuration
    // (CAN_H == A, CAN_L == B)
    ecan_initialize(6, 7, true, true);
#else
    // RX is RP10, TX is RP11 in open_drain configuration
    // (CAN_H == A, CAN_L == B)
    ecan_initialize(10, 11, true, true);
#endif

    ecan_set_rx_object(POSITION_COMMAND_OBJECT, POSITION_COMMAND_CAN_ID, update_motion_command, NULL, 0);
    ecan_set_rx_object(REMOTE_STDIO_RX_OBJECT, REMOTE_STDIO_CAN_ID(CAN_STDIO_AND_RTSP_NODE_ID),
        canstdio_endpoint_process_can_frame, NULL, ECAN_RX_FLAG_ASYNC);
}
