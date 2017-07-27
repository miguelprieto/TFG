/*
 * automation.h
 */

#ifndef __AUTOMATION_H
#define __AUTOMATION_H

#include "bus_objects.h"
#include "fsm.h"

void automation_command(const t_servo_position * pos);
void init_automation(void);
void automation(void);


#endif
