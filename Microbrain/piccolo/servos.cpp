#include "defines.h"
#include <xc.h>
#include <libpic30++.h>
#include <stdio.h>

#include "bus_interface.h"
#include "ecan_lib.h"
#include "wait.h"
#include "servos.h"
#include "gpio.h"

t_servo_status_piccolo servo_status;

void update_servo_status_piccolo(const uint8_t *data, unsigned int len, void *user_ptr)
{
    servo_status =  *(t_servo_status_piccolo*)data;

}

int automation_front_status(void)
{
    ecan_update_object(SERVO_STATUS_PICCOLO_OBJECT);
    return servo_status.front_status;
}

int automation_rear_status(void)
{
    ecan_update_object(SERVO_STATUS_PICCOLO_OBJECT);
    return servo_status.rear_status;
}

int omron_front_status(void)
{
    ecan_update_object(SERVO_STATUS_PICCOLO_OBJECT);
    return servo_status.front_sensor;
}

int omron_rear_status(void)
{
    ecan_update_object(SERVO_STATUS_PICCOLO_OBJECT);
    return servo_status.rear_sensor;
}


int baumer_front_status(void)
{
    return OBSTACLE_FRONT;
}

int baumer_rear_status(void)
{
    return OBSTACLE_REAR;
}

void servo_reset(void)
{
    set_servo_position(255, SERVO_AUTOMATION_RESET);
}

void servo_reset(t_side side)
{
    set_servo_position(255,
                       side == FRONT_SIDE ? SERVO_AUTOMATION_RESET_FRONT : SERVO_AUTOMATION_RESET_REAR);
}

void servo_reset_automation(t_side side)
{
    set_servo_position(255,
                       side == FRONT_SIDE ? SERVO_AUTOMATION_RESET_AUTOMATION_FRONT : SERVO_AUTOMATION_RESET_AUTOMATION_REAR);
}

void servos_off(void)
{
    set_servo_position(255, SERVO_AUTOMATION_OFF);
    sucker(FRONT_SIDE, false);
    sucker(REAR_SIDE, false);
}

void sucker(t_side side, bool on)
{
    if (side == FRONT_SIDE)
        sucker_front(on);
    else
        sucker_rear(on);
}

void arm_pump(t_side side, bool on)
{
    if (side == FRONT_SIDE)
        set_servo_position(255, (on ? SERVO_AUTOMATION_FRONT_PUMP_ON : SERVO_AUTOMATION_FRONT_PUMP_OFF) );
    else
        set_servo_position(255, (on ? SERVO_AUTOMATION_REAR_PUMP_ON : SERVO_AUTOMATION_REAR_PUMP_OFF) );
}

void arm_prepare_dispenser(t_side side)
{
    if (side == FRONT_SIDE)
        set_servo_position(255, SERVO_AUTOMATION_FRONT_PREPARE_DISPENSER );
    else
        set_servo_position(255, SERVO_AUTOMATION_REAR_PREPARE_DISPENSER );
}

void arm_save(t_side side, bool up)
{
    if (up)
        set_servo_position(255, (side == FRONT_SIDE ? SERVO_AUTOMATION_FRONT_SAVE : SERVO_AUTOMATION_REAR_SAVE) );
    else
        set_servo_position(255, (side == FRONT_SIDE ? SERVO_AUTOMATION_FRONT_SAVE_MID : SERVO_AUTOMATION_REAR_SAVE_MID) );
}


void arm_capture(t_side side, bool up)
{
    if (side == FRONT_SIDE) {
        set_servo_position(255,
                           (up ?
                            SERVO_AUTOMATION_FRONT_CAPTURE_UP :
                            SERVO_AUTOMATION_FRONT_CAPTURE_MID) );
    }
    else {
        set_servo_position(255,
                           (up ?
                            SERVO_AUTOMATION_REAR_CAPTURE_UP :
                            SERVO_AUTOMATION_REAR_CAPTURE_MID) );
    }
}

void arm_release(t_side side)
{
    set_servo_position(255, (side == FRONT_SIDE ? SERVO_AUTOMATION_FRONT_RELEASE : SERVO_AUTOMATION_REAR_RELEASE) );
}


void arm_change(t_side side)
{
    set_servo_position(255, (side == FRONT_SIDE ? SERVO_AUTOMATION_FRONT_CHANGE : SERVO_AUTOMATION_REAR_CHANGE) );
}


