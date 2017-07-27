/*
 * bus_interface.h
 */

#ifndef __BUS_INTERFACE
#define __BUS_INTERFACE

#define SERVO_POSITION_OBJECT	0
#define REMOTE_STDIO_RX_OBJECT	1
#define LIFT_TELEMETRY_OBJECT   2

#define CAN_DELAY_MS   10

// main.cpp
void send_lift_position(int lift, int pos);
void lift_home(int lift);
void lift_stop(int lift);
bool lift_check_position(float target_pos);

#endif
