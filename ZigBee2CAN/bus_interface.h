#ifndef BUS_INTERFACE
#define BUS_INTERFACE

#include <stdint.h>

#include "bus_objects.h"

#define CAN_DELAY_MS 5

#define ROBOT_POSITION_OBJECT		0
#define TELEMETRY_OBJECT		1
#define REMOTE_RTSP_RX_OBJECT		2
#define AX12_COMMAND_OBJECT             3
#define REMOTE_STDIO_RX_OBJECT		4
#define SIMPLE_VAR_OUTPUT_OBJECT	5


void bus_objects_initialize();
int ecan_tx(uint8_t *rx_data);

#endif
