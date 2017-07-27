/*
 *	servos.h
 */


#ifndef __SERVOS_H
#define __SERVOS_H

#include <stdint.h>

typedef enum {
    FRONT_SIDE = 0,
    REAR_SIDE,
    LEFT_SIDE,
    RIGHT_SIDE
} t_side;

void update_servo_status_piccolo(const uint8_t *data, unsigned int len, void *user_ptr);
int automation_front_status(void);
int automation_rear_status(void);
int omron_front_status(void);
int omron_rear_status(void);
int baumer_front_status(void);
int baumer_rear_status(void);
void servo_reset(void);
void servo_reset(t_side side);
void servo_reset_automation(t_side side);
void servos_off(void);
void sucker(t_side side, bool on);
void arm_prepare_dispenser(t_side side);
void arm_pump(t_side side, bool on);
void arm_save(t_side side, bool up);
void arm_capture(t_side side, bool up);
void arm_release(t_side side);
void arm_change(t_side side);

#endif
