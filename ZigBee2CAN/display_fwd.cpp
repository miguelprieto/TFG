#include "display_fwd.h"
#include "ecan_lib.h"
#include "bus_interface.h"
#include "bus_objects.h"
#include "xbee.h"

#include <string.h>

static bool inoltra_stato = false;
static t_can_robot_status_update stato_robot;

static bool inoltra_start = false;
static unsigned char start_piccolo_flags;
static unsigned int start_piccolo_elapsed_time;

void display_fwd_process_can_frame_strategy(const uint8_t *data, unsigned int len, void *user_ptr)
{
	const t_can_strategy_command *m = (const t_can_strategy_command*)data;

	if (m->cmd == STRATEGY_COMMAND_START_PICCOLO) // grande -> piccolo
	{
		inoltra_start = true;
		start_piccolo_flags = m->flags;
		start_piccolo_elapsed_time = m->elapsed_time;
	}
	else if (m->cmd == STRATEGY_COMMAND_ALIGN_PICCOLO) // grande -> piccolo
	{
		uint8_t rf_data[10];
		uint16_t can_id = STRATEGY_COMMAND_CAN_ID;
		uint8_t dest_addr[2] = { 0xFF, 0xFF };

		memcpy(rf_data, &can_id, 2);
		memcpy(&rf_data[2], data, 8);

		xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, rf_data, 10);
	}
}

void display_fwd_process_can_frame_status(const uint8_t *data, unsigned int len, void *user_ptr)
{
	const t_can_robot_status_update *m = (const t_can_robot_status_update*)data;
	const t_can_robot_status_update_robot robot = (t_can_robot_status_update_robot)m->robot_selected;

	if (robot == CAN_ROBOT_STATUS_UPDATE_ROBOT_PICCOLO) // piccolo -> grande
	{
		inoltra_stato = true;
		stato_robot = *m;
	}
}

void display_fwd_interval()
{
	ecan_update_object(ROBOT_STATUS_UPDATE_OBJECT);

	if (inoltra_stato)
	{
		uint8_t rf_data[10];
		uint16_t can_id = ROBOT_STATUS_UPDATE_CAN_ID;
		uint8_t dest_addr[2] = { 0xFF, 0xFF };

		memcpy(rf_data, &can_id, 2);
		memcpy(&rf_data[2], &stato_robot, 8);

		xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, rf_data, 10);
	}

	if (inoltra_start)
	{
		if (++start_piccolo_elapsed_time >= 20) // trasmetti start solo per 5 secondi
		{
			inoltra_start = false;
		}
		else
		{
			uint8_t rf_data[10];
			uint16_t can_id = STRATEGY_COMMAND_CAN_ID;
			uint8_t dest_addr[2] = { 0xFF, 0xFF };

			t_can_strategy_command m;
			m.cmd = STRATEGY_COMMAND_START_PICCOLO;
			m.flags = start_piccolo_flags;
			m.elapsed_time = start_piccolo_elapsed_time;

			memcpy(rf_data, &can_id, 2);
			memcpy(&rf_data[2], &m, 8);

			xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, rf_data, 10);
		}
	}
}
