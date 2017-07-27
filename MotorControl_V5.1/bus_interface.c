 /*
  * bus_interface.c
  */

#include "defines.h"

#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "canstdio_endpoint.h"
#include "gpio.h"
#include "position.h"
#include "velocity.h"
#include "path.h"
#include "ecan_lib.h"
#include "bus_objects.h"
#include "bus_interface.h"


#define M_E(m,e)   (float)m * pow(10,e)

t_command_distance_controller_1  dist_c_1;
t_command_heading_controller_1  hdg_c_1;
t_command_point_controller_1 point_c_1;
t_command_point_controller_2 point_c_2;
t_command_line_controller_1 line_c_1;
t_command_line_controller_2 line_c_2;
t_command_circular_rotation_controller_1 circ_rot_c_1;
t_command_set_telemetry_mode telemetry_mode;

// If this variable is set, all motion commands are ignored
static bool obstacle_detected = false;

static void update_motion_command(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_motion_command * m = (const t_can_motion_command*)data;

    switch (m->_cmd) {

    case MOTION_COMMAND_SPEED_PID:
        {
            float kp, ki, kd;
            t_command_speed_pid * p = (t_command_speed_pid *)m;
				
            kp = M_E(p->kp_m, p->kp_e);
            ki = M_E(p->ki_m, p->ki_e);
            kd = M_E(p->kd_m, p->kd_e);

            pi_velocity_l.Kp = kp;
            pi_velocity_l.Ki = ki;
            pi_velocity_l.Kd = kd;

            pi_velocity_r.Kp = kp;
            pi_velocity_r.Ki = ki;
            pi_velocity_r.Kd = kd;

            reset_velocity_controllers();

            flagsR.pid_on   = 0;
            flagsR.motor_en = 0;
            flagsR.frenata  = 0;

            flagsL.pid_on   = 0;
            flagsL.motor_en = 0;
            flagsL.frenata  = 0;

        }
        break;

    case MOTION_COMMAND_DISTANCE_CONTROLLER_1:
        dist_c_1 = *(t_command_distance_controller_1 *)m;
        break;

    case MOTION_COMMAND_DISTANCE_CONTROLLER_2:
        {
            t_command_distance_controller_2 * c_2;
            c_2 = (t_command_distance_controller_2 *)m;
            add_set_distance_controller(dist_c_1.max_speed, dist_c_1.accel, c_2->decel, M_E(c_2->kd_m, c_2->kd_e));
        }
        break;

    case MOTION_COMMAND_HEADING_CONTROLLER_1:
        hdg_c_1 = *(t_command_heading_controller_1 *)m;
        break;

    case MOTION_COMMAND_HEADING_CONTROLLER_2:
        {
            t_command_heading_controller_2 * c_2;
            c_2 = (t_command_heading_controller_2 *)m;
            add_set_heading_controller(hdg_c_1.max_speed, hdg_c_1.accel, c_2->decel, M_E(c_2->kd_m, c_2->kd_e));
        }
        break;


    case MOTION_COMMAND_POINT_CONTROLLER_1:
        point_c_1 = *(t_command_point_controller_1 *)m;
        break;

    case MOTION_COMMAND_POINT_CONTROLLER_2:
        point_c_2 = *(t_command_point_controller_2 *)m;
        break;


    case MOTION_COMMAND_POINT_CONTROLLER_3:
        {
            t_command_point_controller_3 * point_c_3 = (t_command_point_controller_3 *)m;
            add_set_point_controller(point_c_1.max_speed, point_c_1.accel, point_c_1.decel,
                                     M_E(point_c_3->kd_m, point_c_3->kd_e),
                                     point_c_2.max_speed_h, point_c_2.decel_h,
                                     M_E(point_c_3->kd_h_m, point_c_3->kd_h_e));
        }
        break;

    case MOTION_COMMAND_LINE_CONTROLLER_1:
        line_c_1 = *(t_command_line_controller_1 *)m;
        break;

    case MOTION_COMMAND_LINE_CONTROLLER_2:
        line_c_2 = *(t_command_line_controller_2 *)m;
        break;


    case MOTION_COMMAND_LINE_CONTROLLER_3:
        {
            t_command_line_controller_3 * line_c_3 = (t_command_line_controller_3 *)m;
            add_set_line_controller(line_c_1.max_speed, line_c_1.accel, line_c_1.decel,
                                    M_E(line_c_2.kd_m, line_c_2.kd_e),
                                    M_E(line_c_2.kp_h_m, line_c_2.kp_h_e),
                                    M_E(line_c_2.kp_line_m, line_c_2.kp_line_e),
                                    line_c_3->change_threshold);
        }
        break;

    case MOTION_COMMAND_CIRCULAR_ROTATION_CONTROLLER_1:
        circ_rot_c_1 = *(t_command_circular_rotation_controller_1 *)m;
        break;

    case MOTION_COMMAND_CIRCULAR_ROTATION_CONTROLLER_2:
        {
            t_command_circular_rotation_controller_2 * c_2;
            c_2 = (t_command_circular_rotation_controller_2 *)m;
            add_set_circular_rotation_controller(circ_rot_c_1.max_speed, circ_rot_c_1.accel, circ_rot_c_1.decel,
                                                 M_E(c_2->kd_m, c_2->kd_e),
                                                 M_E(c_2->kp_dist_m, c_2->kp_dist_e),
                                                 M_E(c_2->kp_h_m, c_2->kp_h_e));
        }
        break;

    case MOTION_COMMAND_CLEAR_POSITION_VALID_FLAG:
        robot_pos.position_valid = 0;
        break;

    case MOTION_COMMAND_SET_POSITION_VALID_FLAG:
        robot_pos.position_valid = 1;
        break;

    // ------------------------------------------------------------------------
    // Removed since 5.1
    /* case MOTION_COMMAND_CLEAR_SAMPLE: */
    /*     clear_sample_buffer(); */
    /*     break; */

    /* case MOTION_COMMAND_START_SAMPLE: */
    /*     sample_on(); */
    /*     break; */

    /* case MOTION_COMMAND_STOP_SAMPLE: */
    /*     sample_off(); */
    /*     break; */

    /* case MOTION_COMMAND_SEND_SAMPLE: */
    /*     start_send_samples(); */
    /*     break; */

    // ------------------------------------------------------------------------
    case MOTION_COMMAND_SET_PMW:
        {
            t_command_set_pwm_speed * p = (t_command_set_pwm_speed *)m;
            flagsR.pid_on   = 0;
            flagsR.motor_en = 1;
            flagsR.frenata  = 0;
		
            flagsL.pid_on   = 0;
            flagsL.motor_en = 1;
            flagsL.frenata  = 0;
					
            robot_pos.position_control_state = CONTROL_OFF;

            set_pwm(p->left, p->right);
        }
        break;

    case MOTION_COMMAND_SET_SPEED:
        if (obstacle_detected == false)
        {
            t_command_set_pwm_speed * p = (t_command_set_pwm_speed *)m;
            flagsR.pid_on   = 1;
            flagsR.motor_en = 1;
            flagsR.frenata  = 0;
		
            flagsL.pid_on   = 1;
            flagsL.motor_en = 1;
            flagsL.frenata  = 0;
					
            vel.left  = p->left;
            vel.right = p->right;

            robot_pos.motor_locked = 0;
            robot_pos.motor_locked_count = 0;

            robot_pos.position_control_state = CONTROL_OFF;
        }
        break;

    case MOTION_COMMAND_SET_SPEED_NO_LOCK:
        {
            t_command_set_pwm_speed * p = (t_command_set_pwm_speed *)m;
            flagsR.pid_on   = 1;
            flagsR.motor_en = 1;
            flagsR.frenata  = 0;

            flagsL.pid_on   = 1;
            flagsL.motor_en = 1;
            flagsL.frenata  = 0;

            vel.left  = p->left;
            vel.right = p->right;

            robot_pos.motor_locked = 0;
            robot_pos.motor_locked_count = 0;

            robot_pos.position_control_state = CONTROL_SKIP;
        }
        break;

    case MOTION_COMMAND_STOP_AND_FREE:
        if (obstacle_detected == false)
        {
            robot_pos.position_control_state = CONTROL_OFF;

            reset_velocity_controllers();

            flagsR.pid_on   = 0;
            flagsR.motor_en = 0;
            flagsR.frenata  = 0;

            flagsL.pid_on   = 0;
            flagsL.motor_en = 0;
            flagsL.frenata  = 0;

            path.elements = 0;
            path.current = 0;
            path.stopped = 1;
            robot_pos.path_done = 1;

            set_pwm(0, 0);

            //clear_smoothing_filter(&filter_left);
            //clear_smoothing_filter(&filter_right);
        }
        break;

    case MOTION_COMMAND_STOP_AND_BRAKE:
        if (obstacle_detected == false)
        {
            robot_pos.position_control_state = CONTROL_STOP;
            path.elements = 0;
            path.current = 0;
            path.stopped = 1;
            robot_pos.path_done = 1;
        }
        break;

    case MOTION_COMMAND_SET_CURRENT_POSITION:
        {
            t_command_set_current_position * p = (t_command_set_current_position *)m;
            robot_pos.x = p->x;
            robot_pos.y = p->y;
            robot_pos.theta = TO_RADIANS(p->deg100 / 100.0);
            robot_pos.cross_coupling_L = 0.0;
            robot_pos.cross_coupling_R = 0.0;
        }
        break;

    case MOTION_COMMAND_FORWARD_TO_DISTANCE:
        if (obstacle_detected == false)
        {
            t_command_forward_to_distance * p =  (t_command_forward_to_distance *)m;
            add_forward_to_distance(p->distance);
        }
        break;

    case MOTION_COMMAND_FORWARD_TO_POINT:
        if (obstacle_detected == false)
        {
            t_command_forward_to_point * p =  (t_command_forward_to_point *)m;
            add_forward_to_point(p->x, p->y);
        }
        break;

    case MOTION_COMMAND_LINE_TO_POINT:
        if (obstacle_detected == false)
        {
            t_command_line_to_point * p =  (t_command_line_to_point *)m;
            add_line_to_point(p->x, p->y, p->backward);
        }
        break;

    case MOTION_COMMAND_ROTATE_RELATIVE:
        if (obstacle_detected == false)
        {
            t_command_rotate * p =  (t_command_rotate *)m;
            add_rotate_relative(TO_RADIANS(p->degrees));
        }
        break;

    case MOTION_COMMAND_ROTATE_ABSOLUTE:
        if (obstacle_detected == false)
        {
            t_command_rotate * p =  (t_command_rotate *)m;
            add_rotate_absolute(TO_RADIANS(p->degrees));
        }
        break;

    case MOTION_COMMAND_ROTATE_CIRCULAR:
        if (obstacle_detected == false)
        {
            t_command_rotate_circular * p =  (t_command_rotate_circular *)m;
            add_rotate_circular((float)TO_RADIANS(p->degrees/10.0), p->x);
        }
        break;

    case MOTION_COMMAND_HEADING_TO:
        if (obstacle_detected == false)
        {
            t_command_heading_to * p = (t_command_heading_to *)m;
            add_heading_to(p->x, p->y, p->with_back);
        }
        break;

    case MOTION_COMMAND_BUMP_AND_SET_X:
        if (obstacle_detected == false)
        {
            t_command_bump_and_set_x * p = (t_command_bump_and_set_x *)m;
            add_bump_set_x(p->x, p->hdg);
        }
        break;

    case MOTION_COMMAND_BUMP_AND_SET_Y:
        if (obstacle_detected == false)
        {
            t_command_bump_and_set_y * p = (t_command_bump_and_set_y *)m;
            add_bump_set_y(p->y, p->hdg);
        }
        break;

    case MOTION_COMMAND_SIMPLE_BUMP:
        if (obstacle_detected == false)
            add_simple_bump();
        break;

    case MOTION_COMMAND_GO_WITH_OFFSET:
        {
            t_command_go_with_offset * p = (t_command_go_with_offset *)m;
            int offs_x = (p->high_offs_x << 4) | (p->lowbits_offs_xy & 0xf);
            int offs_y = (p->high_offs_y << 4) | ((p->lowbits_offs_xy >> 4) & 0xf);
            add_go_with_offset(p->x, p->y, offs_x, offs_y);
        }
        break;

    case MOTION_COMMAND_HEADING_TO_WITH_OFFSET:
        {
            t_command_go_with_offset * p = (t_command_go_with_offset *)m;
            int offs_x = (p->high_offs_x << 4) | (p->lowbits_offs_xy & 0xf);
            int offs_y = (p->high_offs_y << 4) | ((p->lowbits_offs_xy >> 4) & 0xf);
            add_heading_to_with_offset(p->x, p->y, offs_x, offs_y);
        }
        break;

    case MOTION_COMMAND_SET_WHEEL_RADIUS_LEFT:
        {
            t_command_set_wheel_radius * p = (t_command_set_wheel_radius *)m;
            set_wheel_radius_left(p->wheel_radius);
        }
        break;

    case MOTION_COMMAND_SET_WHEEL_RADIUS_RIGHT:
        {
            t_command_set_wheel_radius * p = (t_command_set_wheel_radius *)m;
            set_wheel_radius_right(p->wheel_radius);
        }
        break;

    case MOTION_COMMAND_SET_WHEEL_DISTANCE:
        {
            t_command_set_wheel_distance * p = (t_command_set_wheel_distance *)m;
            set_wheel_distance(p->wheel_distance);
        }
        break;

    case MOTION_COMMAND_SET_TELEMETRY_MODE:
        {
            t_command_set_telemetry_mode * p = (t_command_set_telemetry_mode *)m;
            telemetry_mode = *p;
        }
        break;

    case MOTION_COMMAND_SET_KX:
        {
            t_command_set_K * p = (t_command_set_K *)m;
            robot_pos.K_x = p->K;
        }
        break;

    case MOTION_COMMAND_SET_KY:
        {
            t_command_set_K * p = (t_command_set_K *)m;
            robot_pos.K_y = p->K;
        }
        break;

    case MOTION_COMMAND_SET_MINIMAL_SPEED:
        {
            t_command_set_minimal_speed * p = (t_command_set_minimal_speed *)m;
            //MINIMAL_LINEAR_SPEED = p->linear_speed;
            //MINIMAL_ROTATION_SPEED = TO_RADIANS(p->rotation_10_speed / 10.0);
        }
        break;

    case MOTION_COMMAND_SET_ERROR_TOLERANCE:
        {
            t_command_set_error_tolerance * p = (t_command_set_error_tolerance *)m;
            //DISTANCE_TOLERANCE = p->linear_10_error / 10.0;
            //HEADING_TOLERANCE = TO_RADIANS(p->heading_10_error / 10.0);
        }
        break;

    case MOTION_COMMAND_SET_ANTICIPATION_GAIN:
        {
            t_command_set_anticipation_gain * p = (t_command_set_anticipation_gain *)m;
            //LINEAR_ANTICIPATION_GAIN = p->linear_gain / 100.0;
            //HEADING_ANTICIPATION_GAIN = p->heading_gain / 100.0;
        }
        break;

    case MOTION_COMMAND_SET_ERROR_TO_MINIMAL_SPEED:
        {
            t_command_set_error_tolerance * p = (t_command_set_error_tolerance *)m;
            //DISTANCE_TO_MINIMAL_SPEED_THRESHOLD = p->linear_10_error / 10.0;
            //HEADING_TO_MINIMAL_SPEED_THRESHOLD = TO_RADIANS(p->heading_10_error / 10.0);
        }
        break;

    }
}

static void update_obstacle_avoidance(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_obstacle_avoidance *m = (const t_can_obstacle_avoidance*)data;

    switch (m->msg_type)
    {
        case CAN_OBSTACLE_AVOIDANCE_MSG_OBSTACLEDETECTED:
            // Arresto immediato
            robot_pos.position_control_state = CONTROL_STOP;
            path.elements = 0;
            path.current = 0;
            path.stopped = 1;
            robot_pos.path_done = 1;
            obstacle_detected = true;
            break;
        case CAN_OBSTACLE_AVOIDANCE_MSG_OBSTACLEPROCESSED:
            obstacle_detected = false;
            break;
        case CAN_OBSTACLE_AVOIDANCE_MSG_RESET:
            obstacle_detected = false;
            break;
    }
}

void send_can_objects(void)
{
    t_can_robot_position pos;
    t_can_robot_motion mot;

    if (telemetry_mode.odometry_mode == 0) {
        pos.x = robot_pos.x;
        pos.y = robot_pos.y;
        pos.deg100 = TO_DEGREES(robot_pos.theta) * 100;
        pos.flags = robot_pos.path_done | (robot_pos.motor_locked << 1) | (robot_pos.position_valid << 2);
#if defined(ROBOT_GRANDE)
        pos.bumpers = (!bumper_pin_1) | ((!bumper_pin_2) << 1);
#else
	pos.bumpers = (!front_bumper_pin) | ((!back_bumper_pin) << 1);
#endif
        ecan_send(ROBOT_POSITION_CAN_ID, (unsigned char *)&pos, 8, 0);
    }
    else {
        mot.L = robot_pos.cross_coupling_L;
        mot.R = robot_pos.cross_coupling_R;
        ecan_send(ROBOT_POSITION_CAN_ID, (unsigned char *)&mot, 8, 0);
    }
}


void check_can_objects(void)
{
    ecan_update_object(MOTION_COMMAND_OBJECT);
    ecan_update_object(OBSTACLE_AVOIDANCE_OBJECT);
}


void send_telemetry_data(void)
{
    if (telemetry_mode.speed_telemetry == 1) {
        t_can_robot_wheels_velocity robot_vel;

        robot_vel.left_speed = (short)vel.read_L;
        robot_vel.right_speed = (short)vel.read_R;
        robot_vel.left_pwm = (short)pi_velocity_l.out;
        robot_vel.right_pwm = (short)pi_velocity_r.out;
        ecan_send(ROBOT_WHEELS_VELOCITY_CAN_ID, (unsigned char *)&robot_vel, 8, 0);
    }
}

void send_velocity_data(void)
{
    t_can_robot_velocity data;

    // Invia zero se siamo ruotando attorno a noi stessi (ovvero il centro di
    // rotazione è compreso tra le due ruote)
    if (robot_pos.rotation_radius_inf == 1 || fabs(robot_pos.rotation_radius) > robot_pos.wheel_distance / 2.0)
        data.linear_speed = vel.read_mean_linear;
    else
        data.linear_speed = 0;

    ecan_send(ROBOT_VELOCITY_CAN_ID, (unsigned char *)&data, 8, 0);
}

void init_bus_objects(void)
{
    telemetry_mode.odometry_mode = 0;
    telemetry_mode.speed_telemetry = 0;

#ifdef BOARD_VERSION_2
    // RX is RP6, TX is RP7 in open_drain configuration
    // (CAN_H == A, CAN_L == B)
    ecan_initialize(6, 7, true, true);
#else
    // RX is RP10, TX is RP11 in open_drain configuration
    // (CAN_H == A, CAN_L == B)
    ecan_initialize(10, 11, true, true);
#endif

    ecan_set_rx_object(MOTION_COMMAND_OBJECT, MOTION_COMMAND_CAN_ID, update_motion_command, NULL, 0);
    ecan_set_rx_object(OBSTACLE_AVOIDANCE_OBJECT, OBSTACLE_AVOIDANCE_CAN_ID, update_obstacle_avoidance, NULL, 0);
    ecan_set_rx_object(REMOTE_STDIO_RX_OBJECT, REMOTE_STDIO_CAN_ID(CAN_STDIO_AND_RTSP_NODE_ID),
        canstdio_endpoint_process_can_frame, NULL, ECAN_RX_FLAG_ASYNC);
}
