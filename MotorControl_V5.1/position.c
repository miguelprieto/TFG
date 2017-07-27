/*
 * position.c
 */


#include "defines.h"

#include "gpio.h"
#include "position.h"
#include "path.h"
#include "bus_interface.h"

#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>


#define BUMP_SPEED 120

t_position_controller heading_controller, distance_controller;
t_circular_rotation_controller circular_rotation_controller;
t_bump_and_set bump_and_set_data;
float target_x, target_y, target_side, offset_x, offset_y, critical_radius;
t_trajectory_profile linear_trajectory, rotation_trajectory;
t_line_controller line_profile;


void set_robot_geometry(float radius_left, float radius_right, float distance, float ticks)
{
    robot_pos.wheel_radius_l = radius_left;
    robot_pos.wheel_radius_r = radius_right;
    robot_pos.wheel_distance = distance;

    robot_pos.tick_per_revolution = ticks;

    robot_pos.wheel_factor_l = TWO_PI * radius_left / ticks;
    robot_pos.wheel_factor_r = TWO_PI * radius_right / ticks;

    robot_pos.K_x = 0.0;
    robot_pos.K_y = 0.0;
}


void set_wheel_radius(float radius_left, float radius_right)
{
    robot_pos.wheel_radius_l = radius_left;
    robot_pos.wheel_radius_r = radius_right;

    robot_pos.wheel_factor_l = TWO_PI * radius_left / robot_pos.tick_per_revolution;
    robot_pos.wheel_factor_r = TWO_PI * radius_right / robot_pos.tick_per_revolution;
}


void set_wheel_radius_left(float radius_left)
{
    robot_pos.wheel_radius_l = radius_left;
    robot_pos.wheel_factor_l = TWO_PI * radius_left / robot_pos.tick_per_revolution;
}


void set_wheel_radius_right(float radius_right)
{
    robot_pos.wheel_radius_r = radius_right;
    robot_pos.wheel_factor_r = TWO_PI * radius_right / robot_pos.tick_per_revolution;
}


void set_wheel_distance(float distance)
{
    robot_pos.wheel_distance = distance;
}


void init_position(void)
{	
    /*inizializzazzione struttura robot, informazioni sulla posizione */

    robot_pos.x = 0.0;
    robot_pos.y = 0.0;
    robot_pos.theta = 0.0;

    robot_pos.tic_L = 0;
    robot_pos.tic_R = 0;
    robot_pos.delta_tic_L = 0;
    robot_pos.delta_tic_R = 0;

    robot_pos.delta_linear = 0.0;
    robot_pos.delta_angular = 0.0;	

    robot_pos.linear	= 0.0;
    robot_pos.angular = 0.0;
	
    robot_pos.delta_R = 0.0;
    robot_pos.delta_L = 0.0;
	
    robot_pos.threshold_flag = 0;
    robot_pos.continuous_flag = 0;
	
    robot_pos.cross_coupling_L = 0.0;
    robot_pos.cross_coupling_R = 0.0;

    robot_pos.position_read = 0.0;
	
    pid_init(&heading_controller.c, 0, 0, 0, P_TIMER_INTERVAL, 0, 0);
    pid_init(&distance_controller.c, 0, 0, 0, P_TIMER_INTERVAL, 0, 0);

    robot_pos.position_control_state = CONTROL_OFF;

    robot_pos.target_got = 0;

    robot_pos.motor_locked_count = 0;
    robot_pos.motor_locked = 0;
}


void reset_position_controllers(controller*pos, controller* cc_ctrl)
{	
    /*reset robot*/
	
    robot_pos.linear		= 0.0;
    robot_pos.angular	= 0.0;
	
    robot_pos.threshold_flag  = 0;
    robot_pos.continuous_flag = 0;
	
    robot_pos.position_read	 = 0.0;

    robot_pos.cross_coupling_L 	= 0.0;
    robot_pos.cross_coupling_R 	= 0.0;
	
    /* reset pi loop in posizione */
	
    pos->error      =  0.0;
    pos->error_old  =  0.0;
    pos->windup     =  0.0;
    pos->u_i        =  0.0;
    pos->out        =  0.0;

    /* reset pi cross_coupling */

    cc_ctrl->error      = 0.0;
    cc_ctrl->error_old  = 0.0;
    cc_ctrl->windup     = 0.0;
    cc_ctrl->u_i        = 0.0;
    cc_ctrl->out        = 0.0;
	
}


float to_rad_s(float vel)
{
    return vel / (robot_pos.wheel_distance / 2.0);
}


float to_mm_s(float w)
{
    return w * robot_pos.wheel_distance / 2.0;
}


float normalize_angle(float a)
{
    if (a > PI)
        a = a - TWO_PI;
    if (a < -PI)
        a = a + TWO_PI;
    return a;
}

void localToGlobal(float local_x, float local_y, float * global_x, float * global_y)
{
    float cos_t = cos(robot_pos.theta);
    float sin_t = sin(robot_pos.theta);
    *global_x = robot_pos.x + local_x * cos_t - local_y * sin_t;
    *global_y = robot_pos.y + local_x * sin_t + local_y * cos_t;
}

void globalToLocal(float global_x, float global_y, float * local_x, float * local_y)
{
    float cos_t = cos(robot_pos.theta);
    float sin_t = sin(robot_pos.theta);
    float dx = global_x - robot_pos.x;
    float dy = global_y - robot_pos.y;
    *local_x =   dx * cos_t + dy * sin_t;
    *local_y = - dx * sin_t + dy * cos_t;
}

void do_brake(void)
{
    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
    flagsR.frenata  = 1;

    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;
    flagsL.frenata  = 1;

    vel.left  = 0;
    vel.right = 0;
}

#define SIGNAL_MOTOR_LOCKED()  {                \
    robot_pos.motor_locked = 1;                 \
    path.current = 0;                           \
    path.elements = 0;                          \
    path.stopped = 1;                           \
                                                \
    flagsR.pid_on   = 1;                        \
    flagsR.motor_en = 0;                        \
    flagsR.frenata  = 1;                        \
                                                \
    flagsL.pid_on   = 1;                        \
    flagsL.motor_en = 0;                        \
    flagsL.frenata  = 1;                        \
                                                \
    vel.left  = 0;                              \
    vel.right = 0;                              \
                                                                \
    robot_pos.position_control_state = CONTROL_OFF;             \
    }


#define MINIMAL_LINEAR_SPEED 150
#if defined(ROBOT_GRANDE)
#define MINIMAL_HEADING_SPEED 0.060 // about 40 deg/sec
#else
#define MINIMAL_HEADING_SPEED 0.5 // about 160 deg/sec
#endif

#define HARD_HEADING_THRESHOLD  (TO_RADIANS(60))

// --------------- HEADING CONTROL -----------------

void set_heading_speed(float max_speed, float accel, float decel, float kd)
{
    // convert speed in rad/s
    max_speed = to_rad_s(max_speed);

    // convert accel/decel in rad/s2
    accel = to_rad_s(accel);
    decel = to_rad_s(decel);

    heading_controller.accel = accel;
    heading_controller.decel = decel;
    heading_controller.max_speed = max_speed;
    heading_controller.max_speed_2 = max_speed * max_speed;
    heading_controller.decel_distance = (heading_controller.max_speed_2) / (2 * decel);
    heading_controller.c.Kp = max_speed / heading_controller.decel_distance;
    heading_controller.c.Kd = kd;
}

// --------------- DISTANCE CONTROL -----------------

void set_distance_speed(float max_speed, float accel, float decel, float kd)
{
    distance_controller.accel = accel;
    distance_controller.decel = decel;
    distance_controller.max_speed = max_speed;
    distance_controller.max_speed_2 = max_speed * max_speed;
    distance_controller.decel_distance = (distance_controller.max_speed_2) / (2 * decel);
    distance_controller.c.Kd = kd;
    //distance_controller.c.Kp = kp;
    distance_controller.c.Kp = max_speed / distance_controller.decel_distance;
}



bool point_control(void)
{
    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
    flagsR.frenata  = 0;

    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;
    flagsL.frenata  = 0;

    float dx = target_x - robot_pos.x;
    float dy = target_y - robot_pos.y;

    /* compute distance to target */
    float effective_linear_distance_to_target = sqrt(dx*dx + dy*dy);

    /* compute heading error */

    heading_controller.target = atan2(dy, dx);

    float heading_error = normalize_angle(heading_controller.target - robot_pos.theta);
    float abs_heading_error = fabs(heading_error);

    /*
     * Ranges for heading_error
     *        0      --> no error, the target is in front of the robot
     * (0   ,  PI/2) --> target on the LEFT, FRONT
     * (0   , -PI/2) --> target on the RIGHT, FRONT
     * (PI/2, PI)    --> target on the LEFT, BEHIND
     * (-PI , -PI/2) --> target on the RIGHT, BEHIND
     */

    float v_sign, w_sign;

    if (abs_heading_error >= HALF_PI) {
        // setup new angle if the target is behind the robot (in this case we must go backward)
        heading_error = normalize_angle(PI - heading_error);
        /* new ranges for heading error when the target is behind
         *
         *  OLD HGD ERR      NEW HDG ERR
         * (PI/2, PI)        (PI/2 , 0)     --> target on the LEFT, BEHIND
         * (-PI , -PI/2)     (0    , -PI/2 )--> target on the RIGHT, BEHIND
         */
        abs_heading_error = fabs(heading_error);
        v_sign = -1.0;
    }
    else
        v_sign = 1.0;

    // correct the distance to target by considering only local X offset
    float error = fabs(effective_linear_distance_to_target*cos(abs_heading_error));

    /* BEGIN DEBUG OUTPUT VARIABLES */
    //debug_output_float("x", robot_pos.x);
    //debug_output_float("y", robot_pos.y);
    //debug_output_float("tx", target_x);
    //debug_output_float("ty", target_y);
    //debug_output_float("vl", vel.read_L);
    //debug_output_float("refl", vel.left);
    //debug_output_float("pwml", pi_velocity_l.out);
    //debug_output_float("vr", vel.read_R);
    //debug_output_float("refr", vel.right);
    //debug_output_float("pwmr", pi_velocity_r.out);
    //debug_output_float("error", error*v_sign);
    //debug_output_float("herror", TO_DEGREES(heading_error));
    //debug_output_flush();
    /* END DEBUG OUTPUT VARIABLES */

    if (error < 5 && effective_linear_distance_to_target <= 50) { // do not optimize if we are far than 5cm
        vel.left  = 0;
        vel.right = 0;
        return true;
    }

    float speed;

    if (abs_heading_error > HARD_HEADING_THRESHOLD && effective_linear_distance_to_target > 50) {
        // if the heading is greather than 30 degrees, rotate only and do not perform distance control
        vel.left = 0;
        vel.right = 0;
    }
    else {
        /* CONTROL ON DISTANCE */

        /* update speeds */
        if (error <= 20) {
            // ok, we are very near the target!
            speed = MINIMAL_LINEAR_SPEED;
        }
        else if (error < distance_controller.decel_distance) {
            // we shoud be in deceleration part
            // compute the expected speed for the given distance
            speed = sqrt(distance_controller.max_speed_2 -
                         2 * distance_controller.decel * (distance_controller.decel_distance - error));
            //speed = pid_evaluation_error(&distance_controller.c, error, robot_pos.linear);

            if (speed > linear_trajectory.current_robot_speed) {
                // but the speed we want to set is greater than the current robot speed, so we should accelerate
                linear_trajectory.current_robot_speed += (linear_trajectory.accel * P_TIMER_INTERVAL);
                if (linear_trajectory.current_robot_speed > speed) {
                    // No! do not overcome the speed we would like to set
                    linear_trajectory.current_robot_speed = speed;
                }
                else {
                    // Ok, we are in acceleration phase, set the desired speed according the speed profile
                    speed = linear_trajectory.current_robot_speed;
                }
            }
            else {
                // regular deceleration phase
                linear_trajectory.current_robot_speed = speed;
            }

            if (speed < MINIMAL_LINEAR_SPEED)
                speed = MINIMAL_LINEAR_SPEED;
        }
        else {
            //pid_set_process_var(&distance_controller.c, robot_pos.linear); // USELESS
            if (linear_trajectory.current_robot_speed < linear_trajectory.max_speed) {
                // accelerate
                linear_trajectory.current_robot_speed += (linear_trajectory.accel * P_TIMER_INTERVAL);
                if (linear_trajectory.current_robot_speed > linear_trajectory.max_speed)
                    linear_trajectory.current_robot_speed = linear_trajectory.max_speed;
            }
            speed = linear_trajectory.current_robot_speed;
        }

        speed *= v_sign;

        vel.left = speed;
        vel.right = speed;
    }

    /* CONTROL ON HEADING */

    if (heading_error < 0)
        w_sign = -1.0;
    else
        w_sign = 1.0;

    if (abs_heading_error < TO_RADIANS(0.5) || effective_linear_distance_to_target <= 50) {
        robot_pos.angular = 0;
        return false; // do not corread heading if the error is small or we are too near the target
    }

    /* update rotation speeds */
    if (abs_heading_error < heading_controller.decel_distance) {
        // we shoud be in deceleration part
        // compute the expected speed for the given heading error
        speed = sqrt(heading_controller.max_speed_2 -
                     2 * heading_controller.decel * (heading_controller.decel_distance - abs_heading_error));

        if (speed > rotation_trajectory.current_robot_speed) {
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > speed) {
                // No! do not overcome the speed we would like to set
                rotation_trajectory.current_robot_speed = speed;
            }
            else {
                // Ok, we are in acceleration phase, set the desired speed according the speed profile
                speed = rotation_trajectory.current_robot_speed;
            }
        }
        else {
            // regular deceleration phase
            rotation_trajectory.current_robot_speed = speed;
        }

        if (speed <= MINIMAL_HEADING_SPEED)
            speed = MINIMAL_HEADING_SPEED; // min speed
    }
    else {
        if (rotation_trajectory.current_robot_speed < rotation_trajectory.max_speed) {
            // accelerate
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > rotation_trajectory.max_speed)
                rotation_trajectory.current_robot_speed = rotation_trajectory.max_speed;
        }
        speed = rotation_trajectory.current_robot_speed;
    }

    /*
     * Ranges for heading_error when the target is in front
     *
     *   linear_speed > 0, v_sign > 0
     *
     * (0   ,  PI/2) --> target on the  LEFT, FRONT, w_sign > 0, rotation_speed > 0
     * (0   , -PI/2) --> target on the RIGHT, FRONT, w_sign < 0, rotation_speed < 0
     */

    /* Ranges for heading error when the target is behind
     *
     *   linear_speed < 0, v_sign < 0
     *
     * (PI/2 , 0)     --> target on the  LEFT, BEHIND, w_sign > 0, rotation_speed < 0
     * (0    , -PI/2) --> target on the RIGHT, BEHIND, w_sign < 0, rotation_speed > 0
     */

    speed = v_sign * w_sign * speed;

    speed = speed * (robot_pos.wheel_distance / 2.0);
    vel.left += -speed;
    vel.right += speed;

    /* debug_output_float("w", speed); */
    /* debug_output_float("vl", vel.left); */
    /* debug_output_float("vr", vel.right); */
    /* debug_output_float("error", error);//\*v_sign); */
    /* debug_output_float("herror", TO_DEGREES(heading_error)); */
    /* debug_output_flush(); */

    return false;
}



bool heading_control(void)
{
    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
    flagsR.frenata  = 0;

    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;
    flagsL.frenata  = 0;

    float dx = target_x - robot_pos.x;
    float dy = target_y - robot_pos.y;

    /* compute distance to target */
    float effective_linear_distance_to_target = sqrt(dx*dx + dy*dy);

    /* compute heading error */

    float heading_error = normalize_angle(heading_controller.target - robot_pos.theta);
    float abs_heading_error = fabs(heading_error);

    if (abs_heading_error < TO_RADIANS(0.5)) {
        vel.left  = 0;
        vel.right = 0;
        return true;
    }

    float w_sign, speed;

    if (heading_error < 0)
        w_sign = -1.0;
    else
        w_sign = 1.0;

    if (abs_heading_error < TO_RADIANS(2)) {
        speed = MINIMAL_HEADING_SPEED;
    }
    else if (abs_heading_error < heading_controller.decel_distance) {
        // we shoud be in deceleration part
        // compute the expected speed for the given heading error
        speed = sqrt(heading_controller.max_speed_2 -
                     2 * heading_controller.decel * (heading_controller.decel_distance - abs_heading_error));

        if (speed > rotation_trajectory.current_robot_speed) {
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > speed) {
                // No! do not overcome the speed we would like to set
                rotation_trajectory.current_robot_speed = speed;
            }
            else {
                // Ok, we are in acceleration phase, set the desired speed according the speed profile
                speed = rotation_trajectory.current_robot_speed;
            }
        }
        else {
            // regular deceleration phase
            rotation_trajectory.current_robot_speed = speed;
        }

        if (speed <= MINIMAL_HEADING_SPEED)
            speed = MINIMAL_HEADING_SPEED; // min speed
    }
    else {
        if (rotation_trajectory.current_robot_speed < rotation_trajectory.max_speed) {
            // accelerate
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > rotation_trajectory.max_speed)
                rotation_trajectory.current_robot_speed = rotation_trajectory.max_speed;
        }
        speed = rotation_trajectory.current_robot_speed;
    }

    speed = w_sign * speed * (robot_pos.wheel_distance / 2.0);
    vel.left = -speed;
    vel.right = speed;

    //debug_output_float("vl", vel.left);
    //debug_output_float("vr", vel.right);
    //debug_output_float("herror", TO_DEGREES(heading_error));
    //debug_output_flush();

    return false;
}



bool relative_rotation_control(void)
{
    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
    flagsR.frenata  = 0;

    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;
    flagsL.frenata  = 0;

    float dx = target_x - robot_pos.x;
    float dy = target_y - robot_pos.y;

    /* compute distance to target */
    float effective_linear_distance_to_target = sqrt(dx*dx + dy*dy);

    /* compute heading error */

    float heading_error = heading_controller.target - robot_pos.angular;
    float abs_heading_error = fabs(heading_error);

    if (abs_heading_error < TO_RADIANS(0.5)) {
        vel.left  = 0;
        vel.right = 0;
        return true;
    }

    float w_sign, speed;

    if (heading_error < 0)
        w_sign = -1.0;
    else
        w_sign = 1.0;

    if (abs_heading_error < TO_RADIANS(2)) {
        speed = MINIMAL_HEADING_SPEED;
    }
    else if (abs_heading_error < heading_controller.decel_distance) {
        // we shoud be in deceleration part
        // compute the expected speed for the given heading error
        speed = sqrt(heading_controller.max_speed_2 -
                     2 * heading_controller.decel * (heading_controller.decel_distance - abs_heading_error));

        if (speed > rotation_trajectory.current_robot_speed) {
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > speed) {
                // No! do not overcome the speed we would like to set
                rotation_trajectory.current_robot_speed = speed;
            }
            else {
                // Ok, we are in acceleration phase, set the desired speed according the speed profile
                speed = rotation_trajectory.current_robot_speed;
            }
        }
        else {
            // regular deceleration phase
            rotation_trajectory.current_robot_speed = speed;
        }

        if (speed <= MINIMAL_HEADING_SPEED)
            speed = MINIMAL_HEADING_SPEED; // min speed
    }
    else {
        if (rotation_trajectory.current_robot_speed < rotation_trajectory.max_speed) {
            // accelerate
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > rotation_trajectory.max_speed)
                rotation_trajectory.current_robot_speed = rotation_trajectory.max_speed;
        }
        speed = rotation_trajectory.current_robot_speed;
    }

    speed = w_sign * speed * (robot_pos.wheel_distance / 2.0);
    vel.left = -speed;
    vel.right = speed;

    return false;
}



bool heading_with_offset_control(void)
{
    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
    flagsR.frenata  = 0;

    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;
    flagsL.frenata  = 0;

    float dx = robot_pos.x - target_x;
    float dy = robot_pos.y - target_y;

    float dist = dx*dx + dy*dy; // distance (robot <--> target)^2

    // FIXME! Quando il punto e' dentro il cerchio di rotazione, il robot trema tutto!
    if (dist < critical_radius) {
        SIGNAL_MOTOR_LOCKED();
        return false;
    }

    float global_x, global_y;

    localToGlobal(offset_x, offset_y, &global_x, &global_y);

    dx = target_x - global_x;
    dy = target_y - global_y;

    heading_controller.target = atan2(dy, dx);

    if (offset_x < 0)
        heading_controller.target = normalize_angle(heading_controller.target + PI);

    return heading_control();
}




// --------------- POINT CONTROL WITH OFFSET -----------------

bool point_control_with_offset(void)
{
    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
    flagsR.frenata  = 0;

    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;
    flagsL.frenata  = 0;


    float dx = robot_pos.x - target_x;
    float dy = robot_pos.y - target_y;

    float dist = dx*dx + dy*dy; // distance (robot <--> target)^2

    if (dist < critical_radius) {
        vel.left  = 0;
        vel.right = 0;
        return true;
    }

    float global_x, global_y;

    // first compute the heading and error
    localToGlobal(offset_x, offset_y, &global_x, &global_y);

    dx = target_x - global_x;
    dy = target_y - global_y;

    heading_controller.target = atan2(dy, dx);

    float v_sign, w_sign;

    /* if (offset_x < 0) { */
    /*     heading_controller.target = normalize_angle(heading_controller.target + PI); */
    /*     v_sign = -1.0; */
    /* } */
    /* else */
    /*     v_sign = 1.0; */

    float heading_error = normalize_angle(heading_controller.target - robot_pos.theta);
    float abs_heading_error = fabs(heading_error);

    /*
     * Ranges for heading_error
     *        0      --> no error, the target is in front of the robot
     * (0   ,  PI/2) --> target on the LEFT, FRONT
     * (0   , -PI/2) --> target on the RIGHT, FRONT
     * (PI/2, PI)    --> target on the LEFT, BEHIND
     * (-PI , -PI/2) --> target on the RIGHT, BEHIND
     */

    if (abs_heading_error >= HALF_PI) {
        // setup new angle if the target is behind the robot (in this case we must go backward)
        heading_error = normalize_angle(PI - heading_error);
        /* new ranges for heading error when the target is behind
         *
         *  OLD HGD ERR      NEW HDG ERR
         * (PI/2, PI)        (PI/2 , 0)     --> target on the LEFT, BEHIND
         * (-PI , -PI/2)     (0    , -PI/2 )--> target on the RIGHT, BEHIND
         */
        abs_heading_error = fabs(heading_error);
        v_sign = -1.0;
    }
    else
        v_sign = 1.0;

    float local_target_x, local_target_y;

    globalToLocal(target_x, target_y, &local_target_x, &local_target_y);

    float error = fabs(local_target_x - offset_x);
    float effective_linear_distance_to_target = error;

    /* BEGIN DEBUG OUTPUT VARIABLES */
    //debug_output_float("x", robot_pos.x);
    //debug_output_float("y", robot_pos.y);
    //debug_output_float("gx", global_x);
    //debug_output_float("gy", global_y);
    //debug_output_float("tx", target_x);
    //debug_output_float("ty", target_y);
    //debug_output_float("vl", vel.read_L);
    //debug_output_float("vrefl", vel.left);
    //debug_output_float("pwml", pi_velocity_l.out);
    //debug_output_float("vr", vel.read_R);
    //debug_output_float("vrefr", vel.right);
    //debug_output_float("pwmr", pi_velocity_r.out);
    //debug_output_float("error", error*v_sign);
    //debug_output_float("herror", TO_DEGREES(heading_error));
    //debug_output_flush();
    /* END DEBUG OUTPUT VARIABLES */

    if (error < 5 && effective_linear_distance_to_target <= 50) { // do not optimize if we are far than 5cm
        vel.left  = 0;
        vel.right = 0;
        return true;
    }

    float speed;

    if (abs_heading_error > HARD_HEADING_THRESHOLD && effective_linear_distance_to_target > 50) {
        // if the heading is greather than 30 degrees, rotate only and do not perform distance control
        vel.left = 0;
        vel.right = 0;
    }
    else {
        /* CONTROL ON DISTANCE */

        /* update speeds */
        if (error <= 20) {
            // ok, we are very near the target!
            speed = MINIMAL_LINEAR_SPEED;
        }
        else if (error < distance_controller.decel_distance) {
            // we shoud be in deceleration part
            // compute the expected speed for the given distance
            speed = sqrt(distance_controller.max_speed_2 -
                         2 * distance_controller.decel * (distance_controller.decel_distance - error));
            //speed = pid_evaluation_error(&distance_controller.c, error, robot_pos.linear);

            if (speed > linear_trajectory.current_robot_speed) {
                // but the speed we want to set is greater than the current robot speed, so we should accelerate
                linear_trajectory.current_robot_speed += (linear_trajectory.accel * P_TIMER_INTERVAL);
                if (linear_trajectory.current_robot_speed > speed) {
                    // No! do not overcome the speed we would like to set
                    linear_trajectory.current_robot_speed = speed;
                }
                else {
                    // Ok, we are in acceleration phase, set the desired speed according the speed profile
                    speed = linear_trajectory.current_robot_speed;
                }
            }
            else {
                // regular deceleration phase
                linear_trajectory.current_robot_speed = speed;
            }

            if (speed < MINIMAL_LINEAR_SPEED)
                speed = MINIMAL_LINEAR_SPEED;
        }
        else {
            //pid_set_process_var(&distance_controller.c, robot_pos.linear); // USELESS
            if (linear_trajectory.current_robot_speed < linear_trajectory.max_speed) {
                // accelerate
                linear_trajectory.current_robot_speed += (linear_trajectory.accel * P_TIMER_INTERVAL);
                if (linear_trajectory.current_robot_speed > linear_trajectory.max_speed)
                    linear_trajectory.current_robot_speed = linear_trajectory.max_speed;
            }
            speed = linear_trajectory.current_robot_speed;
        }

        speed *= v_sign;

        vel.left = speed;
        vel.right = speed;
    }

    /* CONTROL ON HEADING */

    if (heading_error < 0)
        w_sign = -1.0;
    else
        w_sign = 1.0;

    if (abs_heading_error < TO_RADIANS(0.5) || effective_linear_distance_to_target <= 50) {
        robot_pos.angular = 0;
        return false; // do not corread heading if the error is small or we are too near the target
    }

    /* update rotation speeds */
    if (abs_heading_error < heading_controller.decel_distance) {
        // we shoud be in deceleration part
        // compute the expected speed for the given heading error
        speed = sqrt(heading_controller.max_speed_2 -
                     2 * heading_controller.decel * (heading_controller.decel_distance - abs_heading_error));

        if (speed > rotation_trajectory.current_robot_speed) {
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > speed) {
                // No! do not overcome the speed we would like to set
                rotation_trajectory.current_robot_speed = speed;
            }
            else {
                // Ok, we are in acceleration phase, set the desired speed according the speed profile
                speed = rotation_trajectory.current_robot_speed;
            }
        }
        else {
            // regular deceleration phase
            rotation_trajectory.current_robot_speed = speed;
        }

        if (speed <= MINIMAL_HEADING_SPEED)
            speed = MINIMAL_HEADING_SPEED; // min speed
    }
    else {
        if (rotation_trajectory.current_robot_speed < rotation_trajectory.max_speed) {
            // accelerate
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > rotation_trajectory.max_speed)
                rotation_trajectory.current_robot_speed = rotation_trajectory.max_speed;
        }
        speed = rotation_trajectory.current_robot_speed;
    }

    /*
     * Ranges for heading_error when the target is in front
     *
     *   linear_speed > 0, v_sign > 0
     *
     * (0   ,  PI/2) --> target on the  LEFT, FRONT, w_sign > 0, rotation_speed > 0
     * (0   , -PI/2) --> target on the RIGHT, FRONT, w_sign < 0, rotation_speed < 0
     */

    /* Ranges for heading error when the target is behind
     *
     *   linear_speed < 0, v_sign < 0
     *
     * (PI/2 , 0)     --> target on the  LEFT, BEHIND, w_sign > 0, rotation_speed < 0
     * (0    , -PI/2) --> target on the RIGHT, BEHIND, w_sign < 0, rotation_speed > 0
     */

    speed = v_sign * w_sign * speed;

    speed = speed * (robot_pos.wheel_distance / 2.0);
    vel.left += -speed;
    vel.right += speed;

    /* debug_output_float("w", speed); */
    /* debug_output_float("vl", vel.left); */
    /* debug_output_float("vr", vel.right); */
    /* debug_output_float("error", error);//\*v_sign); */
    /* debug_output_float("herror", TO_DEGREES(heading_error)); */
    /* debug_output_flush(); */

    return false;
}





// --------------- LINE CONTROL -----------------

void set_line_control_parameters(float kp_heading, float kp_line_distance, float change_threshold)
{
    line_profile.kp_heading = kp_heading;
    line_profile.kp_line_distance = kp_line_distance;
    line_profile.change_threshold = change_threshold;
    line_profile.do_not_brake = false;
}



void line_for_two_points(float x1, float y1, float x2, float y2, float * a, float * b, float * c)
{
    float M;
    if (x1 == x2) {
        // singularity: the line is parallel to y axis, the equation is
        // x = x1 -->  - x + x1 = 0
        if (y2 > y1) {
            *a = -1;
            *b = 0;
            *c = x1;
        }
        else {
            *a = 1;
            *b = 0;
            *c = -x1;
        }
    }
    else {
        // the line is generic
        *a = - (y2 - y1) / (x2 - x1);
        *b = 1;
        *c = - y1 + ((y2 - y1) * x1 )/ (x2 - x1);

        M = atan2(y2-y1, x2-x1);

        // the line has a direction, according to the angular coefficient,
        // change the sign of the parameters
        if ((M < -HALF_PI) || (M > HALF_PI)) {
            *a = -(*a);
            *b = -(*b);
            *c = -(*c);
        }
    }
}

//#define LP_DEBUG

bool line_control(void)
{
    float dx, dy;
    float error, abs_dist_error, heading_error, abs_heading_error, hdg_point, speed, d;

    // abs_dist_error is the absolute value of dist_error

    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
    flagsR.frenata  = 0;
				
    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;
    flagsL.frenata  = 0;

    /* distance error */
    dx = target_x - robot_pos.x;
    dy = target_y - robot_pos.y;

#if 1
    abs_dist_error = sqrt(dx*dx + dy*dy);
#else
     // <-- modifica del 20/03 sera (fabio)
    line_profile.target_heading = atan2(dy, dx);
    abs_dist_error = dx*cos(line_profile.target_heading) + dy*sin(line_profile.target_heading);
#endif

    /* heading error */
    heading_error = normalize_angle(line_profile.target_heading - robot_pos.theta);
    abs_heading_error = fabs(heading_error);

    hdg_point = normalize_angle(atan2(dy, dx) - robot_pos.theta);

    if (line_profile.backward) {
        hdg_point = normalize_angle(PI - hdg_point);
        heading_error = normalize_angle(PI - heading_error);
        abs_heading_error = fabs(heading_error);
    }

    error = abs_dist_error;

    if (fabs(hdg_point) >= HALF_PI && line_profile.has_right_heading) {
        /* the point is on the back of the robot (or in front if we are going backward):
           it could be hard to recover, stop the robot! */
        vel.left  = 0;
        vel.right = 0;
#ifdef LP_DEBUG
        debug_output_float("x", 3000);
        debug_output_float("y", 2000);
        debug_output_flush();
#endif
        return true;
    }

    if (error < line_profile.current_change_threshold) {
        float next_heading;
        if (get_next_line_to_point_heading(&next_heading)) {
            if (fabs(normalize_angle(next_heading - line_profile.target_heading)) < HARD_HEADING_THRESHOLD) {
                line_profile.do_not_brake = true;
#ifdef LP_DEBUG
                debug_output_float("x", 0);
                debug_output_float("y", 0);
                debug_output_flush();
#endif
                return true;
            }
        }
    }


    if (error < 5) {
        vel.left  = 0;
        vel.right = 0;
#ifdef LP_DEBUG
        debug_output_float("x", 3000);
        debug_output_float("y", 0);
        debug_output_flush();
#endif
        return true;
    }


    /* CONTROL ON DISTANCE */

    if (abs_heading_error >= HARD_HEADING_THRESHOLD && !line_profile.has_right_heading) {
        // if the heading is greather than 30 degrees, rotate only and do not perform distance control
        vel.left = 0;
        vel.right = 0;
#ifdef LP_DEBUG
        debug_output_float("w", 0);
#endif
    }
    else {
#ifdef LP_DEBUG
        debug_output_float("w", 100);
#endif
        line_profile.has_right_heading = true;
        /* update speeds */
        if (error <= 20) {
            // ok, we are very near the target!
            speed = MINIMAL_LINEAR_SPEED;
        }
        else if (error < distance_controller.decel_distance) {
            // we shoud be in deceleration part
            // compute the expected speed for the given distance
            speed = sqrt(distance_controller.max_speed_2 -
                         2 * distance_controller.decel * (distance_controller.decel_distance - error));
            //speed = pid_evaluation_error(&distance_controller.c, error, robot_pos.linear);

            if (speed > linear_trajectory.current_robot_speed) {
                // but the speed we want to set is greater than the current robot speed, so we should accelerate
                linear_trajectory.current_robot_speed += (linear_trajectory.accel * P_TIMER_INTERVAL);
                if (linear_trajectory.current_robot_speed > speed) {
                    // No! do not overcome the speed we would like to set
                    linear_trajectory.current_robot_speed = speed;
                }
                else {
                    // Ok, we are in acceleration phase, set the desired speed according the speed profile
                    speed = linear_trajectory.current_robot_speed;
                }
            }
            else {
                // regular deceleration phase
                linear_trajectory.current_robot_speed = speed;
            }

            if (speed < MINIMAL_LINEAR_SPEED)
                speed = MINIMAL_LINEAR_SPEED;
        }
        else {
            //pid_set_process_var(&distance_controller.c, robot_pos.linear); // USELESS
            if (linear_trajectory.current_robot_speed < linear_trajectory.max_speed) {
                // accelerate
                linear_trajectory.current_robot_speed += (linear_trajectory.accel * P_TIMER_INTERVAL);
                if (linear_trajectory.current_robot_speed > linear_trajectory.max_speed)
                    linear_trajectory.current_robot_speed = linear_trajectory.max_speed;
            }
            speed = linear_trajectory.current_robot_speed;
        }

        if (line_profile.backward)
            speed = -speed;

        vel.left = speed;
        vel.right = speed;
    }

    d = (line_profile.a * robot_pos.x + line_profile.b * robot_pos.y + line_profile.c) / line_profile.den;

    if (line_profile.backward) d = -d;

    speed =  - d * line_profile.kp_line_distance + heading_error * line_profile.kp_heading;

    if (vel.left == 0 && vel.right == 0 && fabs(speed) < MINIMAL_LINEAR_SPEED)
        speed = sgn(speed) * MINIMAL_LINEAR_SPEED; // if we are only rotating, ensure a minimal speed */

    if (line_profile.backward) speed = -speed;

    vel.left += -speed;
    vel.right += speed;

#ifdef LP_DEBUG
    debug_output_float("vl", vel.read_L);
    debug_output_float("vr", vel.read_R);
    debug_output_float("refl", vel.left);
    debug_output_float("refr", vel.right);
    //debug_output_float("herr", TO_DEGREES(heading_error));//\*v_sign);
    //debug_output_float("d", d);//\*v_sign);
    debug_output_float("x", robot_pos.x);
    debug_output_float("y", robot_pos.y);
    debug_output_flush();
#endif

    return false;
}


// --------------- CIRCULAR ROTATION CONTROL -----------------

void set_circular_rotation_speed(float max_speed, float accel, float decel, float kd, float Kdist, float Khdg)
{
    circular_rotation_controller.linear_accel = accel;
    circular_rotation_controller.linear_decel = decel;
    circular_rotation_controller.linear_max_speed = max_speed;

    circular_rotation_controller.c.Kd = kd;
    circular_rotation_controller.Kdist = Kdist;
    circular_rotation_controller.Khdg = Khdg;
}

bool circular_rotation_control(void)
{
    const float diffX = robot_pos.x - circular_rotation_controller.cx;
    const float diffY = robot_pos.y - circular_rotation_controller.cy;
    const float diffDir = atan2(diffY, diffX);

    // Errore rispetto all'angolo target (positivo=senso antioriario)
    const float error =
        normalize_angle(circular_rotation_controller.target - diffDir) *
        (-sgn(circular_rotation_controller.target_radius));
    const float a_error = fabs(error);

    if (a_error < TO_RADIANS(0.5)) {
        vel.left  = 0;
        vel.right = 0;
        return true;
    }

    const int dir = sgn(error); // dir=1 senso antioriario, dir=-1 senso orario

    const float tanDir = normalize_angle(diffDir + HALF_PI);

    // Errore rispetto alla tangente
    float hdg_error = normalize_angle(tanDir - robot_pos.theta);
    if (circular_rotation_controller.target_radius > 0)
        hdg_error = normalize_angle(hdg_error + PI);

    float speed;

    if (a_error < TO_RADIANS(2)) {
        speed = MINIMAL_HEADING_SPEED;
    }
    if (a_error < circular_rotation_controller.decel_distance) {
        // we shoud be in deceleration part
        // compute the expected speed for the given heading error
        speed = sqrt(circular_rotation_controller.angular_max_speed_2 -
                     2 * circular_rotation_controller.angular_decel * (circular_rotation_controller.decel_distance - a_error));

        if (speed > rotation_trajectory.current_robot_speed) {
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > speed) {
                // No! do not overcome the speed we would like to set
                rotation_trajectory.current_robot_speed = speed;
            }
            else {
                // Ok, we are in acceleration phase, set the desired speed according the speed profile
                speed = rotation_trajectory.current_robot_speed;
            }
        }
        else {
            // regular deceleration phase
            rotation_trajectory.current_robot_speed = speed;
        }

        if (speed <= MINIMAL_HEADING_SPEED)
            speed = MINIMAL_HEADING_SPEED; // min speed
    }
    else {
        if (rotation_trajectory.current_robot_speed < rotation_trajectory.max_speed) {
            // accelerate
            rotation_trajectory.current_robot_speed += (rotation_trajectory.accel * P_TIMER_INTERVAL);
            if (rotation_trajectory.current_robot_speed > rotation_trajectory.max_speed)
                rotation_trajectory.current_robot_speed = rotation_trajectory.max_speed;
        }
        speed = rotation_trajectory.current_robot_speed;
    }

    speed = dir * fabs(circular_rotation_controller.target_radius) * speed;

    const float current_radius = sqrt((diffY*diffY)+(diffX*diffX));
    const float d = fabs(circular_rotation_controller.target_radius) - current_radius;
    const float diff = robot_pos.wheel_distance / (2 * circular_rotation_controller.target_radius)
        - d * sgn(circular_rotation_controller.target_radius) * circular_rotation_controller.Kdist;

    if (fabs(hdg_error) < HALF_PI / 2) {
        vel.left = speed * (1 + diff);
        vel.right = speed * (1 - diff);
    }
    else {
        vel.left = vel.right = 0;
    }

    vel.left -= hdg_error * circular_rotation_controller.Khdg;
    vel.right += hdg_error * circular_rotation_controller.Khdg;

    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;

    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;

    return false;
}



void check_target_got(int zr)
{
    if (zr == 1) {
        robot_pos.target_got_count++;
        if (robot_pos.target_got_count == 2) // CS: it was 5, but 2 is better (maybe!!!)
            robot_pos.target_got = 1;
    }
    else
        robot_pos.target_got_count = 0;
}



void check_motor_locked(void)
{
    if (robot_pos.position_control_state == CONTROL_STOP)
        return; // no check on motor locked (save last value)

    if (robot_pos.position_control_state == CONTROL_ON_BUMP ||
        robot_pos.position_control_state == CONTROL_ON_SIMPLE_BUMP ||
        robot_pos.position_control_state == CONTROL_SKIP) {

        // clear motor locked (we do not check lock here)

        robot_pos.motor_locked_count = 0;
        robot_pos.motor_locked = 0;

        return;
    }

    if (is_left_wheel_locked() || is_right_wheel_locked()) {

        robot_pos.motor_locked_count++;

        // it is called each 4*0.005 s = 0.020 s
        if (robot_pos.motor_locked_count >= 20) {  // for 0.4 second continuously

            SIGNAL_MOTOR_LOCKED(); // it's a MACRO, see above

        }
    }
    else
        robot_pos.motor_locked_count = 0;
}


void position_control(void)
{
    switch (robot_pos.position_control_state) {
    case CONTROL_SKIP:
        break;

    case CONTROL_OFF:
        break;

    case CONTROL_STOP:
        flagsR.pid_on   = 1;
        flagsR.motor_en = 1;
        flagsR.frenata  = 1;
				
        flagsL.pid_on   = 1;
        flagsL.motor_en = 1;
        flagsL.frenata  = 1;

        vel.left  = 0;
        vel.right = 0;

        break;

    case CONTROL_ON_HEADING_TO:
        check_target_got(heading_control());
        break;

    case CONTROL_ON_RELATIVE_ROTATION:
        check_target_got(relative_rotation_control());
        break;

    case CONTROL_ON_HEADING_WITH_OFFSET:
        check_target_got(heading_with_offset_control());
        break;

    case CONTROL_ON_CIRCULAR_ROTATION:
        check_target_got(circular_rotation_control());
        break;

    case CONTROL_ON_POINT:
        check_target_got(point_control());
        break;

    case CONTROL_ON_POINT_WITH_OFFSET:
        check_target_got(point_control_with_offset());
        break;

    case CONTROL_ON_LINE:
        check_target_got(line_control());
        break;

    case CONTROL_ON_BUMP:
        robot_pos.bump_timer_count++;
#if defined(ROBOT_GRANDE)
        if (bumper_pin_1 == 0) {
#else
        if (back_bumper_pin == 0) {
#endif
            ++robot_pos.bumper_count;
            if (robot_pos.bumper_count == 5) {
                vel.left  = 0;
                vel.right = 0;
                if (bump_and_set_data.value != 0) {
                    if (bump_and_set_data.set_x == 1)
                        robot_pos.x = bump_and_set_data.value;
                    else
                        robot_pos.y = bump_and_set_data.value;
                    robot_pos.theta = TO_RADIANS(bump_and_set_data.heading);
                }
                robot_pos.target_got = 1;
                robot_pos.position_control_state = CONTROL_STOP;
                distance_controller.target = 0;
                heading_controller.target = robot_pos.theta;
                robot_pos.linear = 0;
            }
        }
        else
            robot_pos.bumper_count = 0;
        /*
        // Bump timeout managing --> do not use now
        //
        if(bump_timer_count>250) {
            vel.left  = 0;
            vel.right = 0;
            target_got = 1;
            robot_pos.position_control_state = CONTROL_ON_HEADING_AND_DISTANCE;
            distance_controller.target = 0;
            heading_controller.target = robot_pos.theta;
            robot_pos.linear = 0;
        }
        */

        break;

    case CONTROL_ON_SIMPLE_BUMP:
#if defined(ROBOT_GRANDE)
        if (bumper_pin_1 == 0) {
#else
        if (back_bumper_pin == 0) {
#endif
            vel.left  = 0;
            vel.right = 0;
            robot_pos.target_got = 1;
            robot_pos.position_control_state = CONTROL_STOP;
            distance_controller.target = 0;
            heading_controller.target = robot_pos.theta;
            robot_pos.linear = 0;
        }
        break;

    }
}


void reset_motion_structures(bool reset_speed)
{
    robot_pos.linear = 0;
    robot_pos.angular = 0;
    robot_pos.target_got = 0;
    robot_pos.target_got_count = 0;
    robot_pos.motor_locked_count = 0;
    robot_pos.motor_locked = 0;

    linear_trajectory.max_speed = distance_controller.max_speed;
    linear_trajectory.accel = distance_controller.accel;
    linear_trajectory.decel = distance_controller.decel;
    if (reset_speed) {
        linear_trajectory.current_robot_speed = 0;
        line_profile.do_not_brake = false;
    }

    rotation_trajectory.max_speed = heading_controller.max_speed;
    rotation_trajectory.accel = heading_controller.accel;
    rotation_trajectory.decel = heading_controller.decel;
    if (reset_speed) {
        rotation_trajectory.current_robot_speed = 0;
        line_profile.do_not_brake = false;
    }

    if (line_profile.do_not_brake)
        line_profile.do_not_brake = false;
}


void reset_circular_motion_structure(void)
{
    robot_pos.linear = 0;
    robot_pos.angular = 0;
    robot_pos.target_got = 0;
    robot_pos.target_got_count = 0;
    robot_pos.motor_locked_count = 0;
    robot_pos.motor_locked = 0;

    rotation_trajectory.max_speed = circular_rotation_controller.angular_max_speed;
    rotation_trajectory.accel = circular_rotation_controller.angular_accel;
    rotation_trajectory.decel = circular_rotation_controller.angular_decel;
    rotation_trajectory.current_robot_speed = 0;
}


// ---------------------------------------------------------------
// Interface

void rotate_absolute(float heading)
{
    target_x = robot_pos.x;
    target_y = robot_pos.y;

    heading_controller.target = heading;

    robot_pos.position_control_state = CONTROL_ON_HEADING_TO;

    reset_motion_structures(true);
}


void rotate_relative(float heading)
{
    target_x = robot_pos.x;
    target_y = robot_pos.y;

    heading_controller.target = heading;

    robot_pos.position_control_state = CONTROL_ON_RELATIVE_ROTATION;

    reset_motion_structures(true);
}


void forward_to_distance(float distance)
{
    float x,y;

    localToGlobal(distance, 0, &x, &y);
    forward_to_point(x, y);
}


void forward_to_point(int x, int y)
{
    target_x = x;
    target_y = y;

    float dx = target_x - robot_pos.x;
    float dy = target_y - robot_pos.y;

    heading_controller.target = atan2(dy, dx);

    robot_pos.position_control_state = CONTROL_ON_POINT;

    reset_motion_structures(true);
}


void line_to_point(int x1, int y1,int x2, int y2, bool backward)
{
    line_for_two_points(x1, y1, x2, y2,
                        &line_profile.a,
                        &line_profile.b,
                        &line_profile.c);

    line_profile.den = sqrt (line_profile.a * line_profile.a +
                             line_profile.b * line_profile.b);

    line_profile.target_heading = atan2(y2 - y1, x2 - x1);

    robot_pos.position_control_state = CONTROL_ON_LINE;

    target_x = x2;
    target_y = y2;

    line_profile.backward = backward;
    line_profile.has_right_heading = false;

    float dx = target_x - robot_pos.x;
    float dy = target_y - robot_pos.y;

    float error = sqrt(dx*dx + dy*dy);

    if (error < line_profile.change_threshold)
        line_profile.current_change_threshold = error * 2.0 / 3.0;
    else
        line_profile.current_change_threshold = line_profile.change_threshold;

#if !defined(ROBOT_GRANDE)
    if (fabs(normalize_angle(backward * PI - (robot_pos.theta - line_profile.target_heading))) >= HARD_HEADING_THRESHOLD) {
        // the point is on the back, we should rotate first
        int p = get_current_path_position();
        push_path_element(p);
        put_heading_to(p, x2, y2, 2 * backward);
        // go to heading_to control
        heading_to(x2, y2, 2 * backward);
    }
    else
#endif
        reset_motion_structures(!line_profile.do_not_brake);
}


void circular_rotation(int center_delta_x, float angle)
{
    float angular_max_speed, angular_accel, angular_decel;

    // Calcola centro di rotazione, center_delta_x è lo scostamento del centro
    // di rotazione lungo l'asse x locale
    circular_rotation_controller.cx = robot_pos.x + center_delta_x * cos(robot_pos.theta - HALF_PI);
    circular_rotation_controller.cy = robot_pos.y + center_delta_x * sin(robot_pos.theta - HALF_PI);
    circular_rotation_controller.target = normalize_angle(robot_pos.theta + HALF_PI * sgn(center_delta_x) + angle);
    circular_rotation_controller.target_radius = center_delta_x;

    // convert speed in rad/s
    angular_max_speed = circular_rotation_controller.linear_max_speed / fabs(circular_rotation_controller.target_radius);
    // convert accel/decel in rad/s2
    angular_accel = circular_rotation_controller.linear_accel / fabs(circular_rotation_controller.target_radius);
    angular_decel = circular_rotation_controller.linear_decel / fabs(circular_rotation_controller.target_radius);

    circular_rotation_controller.decel_distance = (angular_max_speed * angular_max_speed) / (2 * angular_decel);
    circular_rotation_controller.c.Kp = angular_max_speed / circular_rotation_controller.decel_distance;

    robot_pos.position_control_state = CONTROL_ON_CIRCULAR_ROTATION;

    circular_rotation_controller.angular_max_speed = angular_max_speed;
    circular_rotation_controller.angular_max_speed_2 = angular_max_speed * angular_max_speed;
    circular_rotation_controller.angular_accel = angular_accel;
    circular_rotation_controller.angular_decel = angular_decel;

    reset_circular_motion_structure();

    robot_pos.linear = 0;
    robot_pos.angular = 0;
    robot_pos.target_got = 0;
    robot_pos.target_got_count = 0;
    robot_pos.motor_locked_count = 0;
    robot_pos.motor_locked = 0;
}


/* HEADING_FRONT = 0, */
/* HEADING_RIGHT = 1, */
/* HEADING_BACK = 2, */
/* HEADING_LEFT = 3 */
/* HEADING_FRONT_OR_BACK = 4 */
void heading_to(int x, int y, int side)
{
    float dx = x - robot_pos.x;
    float dy = y - robot_pos.y;

    target_x = robot_pos.x;
    target_y = robot_pos.y;

    if (side != 4) // if not HEADING_FRONT_OR_BACK
    {
        heading_controller.target = normalize_angle(atan2(dy, dx) + HALF_PI * side);
    }
    else
    {
        double target_front = atan2(dy, dx);
        if (fabs(target_front - robot_pos.theta) <= HALF_PI)
            heading_controller.target = target_front;
        else
            heading_controller.target = normalize_angle(target_front + PI);
    }

    robot_pos.position_control_state = CONTROL_ON_HEADING_TO;

    reset_motion_structures(true);
}


void heading_to_with_offset(float x, float y, float offs_x, float offs_y)
{
    target_x = x;
    target_y = y;
    offset_x = offs_x;
    offset_y = offs_y;

    critical_radius = offset_x*offset_x + offset_y*offset_y;

    robot_pos.position_control_state = CONTROL_ON_HEADING_WITH_OFFSET;

    reset_motion_structures(true);
}


/*
 * WARNING!!!!!!! THIS FUNCTION ASSUMES THAT THE ROBOT HEADING IS CORRECT,
 * IT SHOULD BE SUCH THAT A FORWARD MOTION WILL REACH THE TARGET (WITH. THE OFFSET POINT)
 * A HEADING_TO_WITH_OFFSET MUST BE MANDATORY PERFORMED BEFORE THIS CONTROL.
 */
void go_with_offset(float x, float y, float offs_x, float offs_y)
{
    float local_target_x, local_target_y;

    target_x = x;
    target_y = y;
    offset_x = offs_x;
    offset_y = offs_y;

    critical_radius = offset_x*offset_x + offset_y*offset_y;

    robot_pos.position_control_state = CONTROL_ON_POINT_WITH_OFFSET;

    reset_motion_structures(true);

    //globalToLocal(x, y, &local_target_x, &local_target_y);
    //forward_to_distance(local_target_x - offs_x);
}

void bump_and_set_x(int x, int hdg)
{
    bump_and_set_data.set_x = 1;
    bump_and_set_data.value = x;
    bump_and_set_data.heading = hdg;
    robot_pos.bump_timer_count=0; //bumper emergency timer
    robot_pos.position_control_state = CONTROL_ON_BUMP;

    robot_pos.target_got = 0;
    robot_pos.target_got_count = 0;
    robot_pos.motor_locked_count = 0;
    robot_pos.motor_locked = 0;

    vel.left = -BUMP_SPEED;
    vel.right = -BUMP_SPEED;

    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
				
    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;

    robot_pos.bumper_count = 0;
}

void bump_and_set_y(int y, int hdg)
{
    bump_and_set_data.set_x = 0;
    bump_and_set_data.value = y;
    bump_and_set_data.heading = hdg;
    robot_pos.bump_timer_count=0; //bumper emergency timer
    robot_pos.position_control_state = CONTROL_ON_BUMP;

    robot_pos.target_got = 0;
    robot_pos.target_got_count = 0;
    robot_pos.motor_locked_count = 0;
    robot_pos.motor_locked = 0;

    vel.left = -BUMP_SPEED;
    vel.right = -BUMP_SPEED;

    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
				
    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;

    robot_pos.bumper_count = 0;
}

void simple_bump(void)
{
    robot_pos.position_control_state = CONTROL_ON_SIMPLE_BUMP;

    robot_pos.target_got = 0;
    robot_pos.target_got_count = 0;
    robot_pos.motor_locked_count = 0;
    robot_pos.motor_locked = 0;

    vel.left = -BUMP_SPEED;
    vel.right = -BUMP_SPEED;

    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
				
    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;

}

