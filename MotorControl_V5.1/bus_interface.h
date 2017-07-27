/*
 * bus_interface.h
 */

#ifndef __BUS_INTERFACE
#define __BUS_INTERFACE

#include <stdbool.h>
#include <stdint.h>

#include "bus_objects.h"

#define MOTION_COMMAND_OBJECT		0
#define OBSTACLE_AVOIDANCE_OBJECT	1
#define REMOTE_STDIO_RX_OBJECT		2

void send_can_objects(void);
void check_can_objects(void);
//void send_telemetry_data(t_can_telemetry_data * d);
void send_velocity_data(void);
void init_bus_objects(void);

#endif
