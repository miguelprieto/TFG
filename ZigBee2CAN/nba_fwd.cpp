#include <string.h>
#include "bus_interface.h"
#include "ecan_lib.h"
#include "nba_fwd.h"
#include "xbee.h"

static bool got_status = false;
static uint8_t status_rfdata[10];

void nba_fwd_process_can_frame_command(const uint8_t *data, unsigned int len, void *user_ptr)
{
	uint8_t rf_data[10];
	uint16_t can_id = NBA_COORDINATION_COMMAND_ID;
	uint8_t dest_addr[2] = { 0xFF, 0xFF };

	memcpy(rf_data, &can_id, 2);
	memcpy(&rf_data[2], data, 8);

	xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, rf_data, 10);
}

void nba_fwd_process_can_frame_status(const uint8_t *data, unsigned int len, void *user_ptr)
{
	uint16_t can_id = NBA_COORDINATION_STATUS_ID;
	memcpy(status_rfdata, &can_id, 2);
	memcpy(&status_rfdata[2], data, 8);
	got_status = true;
}

void nba_fwd_interval()
{
	ecan_update_object(NBA_COORDINATION_STATUS_OBJECT);

	if (got_status)
	{
		uint8_t dest_addr[2] = { 0xFF, 0xFF };
		xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, status_rfdata, 10);
	}
}
