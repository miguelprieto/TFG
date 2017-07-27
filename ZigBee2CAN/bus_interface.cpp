#include "defines.h"

#include <xc.h>
#include <libpic30++.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ecan_lib.h"
#include "bus_interface.h"
#include "canstdio_coord.h"
#include "rtsp_coord.h"
#include "simple_var_output.h"
#include "xbee.h"
#include "led.h"
#ifdef HAVE_AX12
#include "ax12_interface.h"
#endif

static void update_robot_position(const uint8_t *data, unsigned int len, void *user_ptr)
{
    uint8_t rf_data[10];
    uint16_t can_id = ROBOT_POSITION_CAN_ID;
    uint8_t dest_addr[2] = { 0xFF, 0xFF };

    memcpy(rf_data, &can_id, 2);
    memcpy(&rf_data[2], data, 8);

    green_led_toggle();

    xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, rf_data, 10);
}

static void update_telemetry_data(const uint8_t *data, unsigned int len, void *user_ptr)
{
    uint8_t rf_data[10];
    uint16_t can_id = ROBOT_WHEELS_VELOCITY_CAN_ID;
    uint8_t dest_addr[2] = { 0xFF, 0xFF };

    memcpy(rf_data, &can_id, 2);
    memcpy(&rf_data[2], data, 8);

    green_led_toggle();

    xbee_tx_request(dest_addr, XBEE_ADDR16_LENGTH, rf_data, 10);
}

void bus_objects_initialize()
{
    ecan_set_rx_object(ROBOT_POSITION_OBJECT, ROBOT_POSITION_CAN_ID, update_robot_position, NULL, 0);
    ecan_set_rx_object(TELEMETRY_OBJECT, ROBOT_WHEELS_VELOCITY_CAN_ID, update_telemetry_data, NULL, 0);
    ecan_set_rx_object(REMOTE_RTSP_RX_OBJECT, REMOTE_RTSP_CAN_ID(REMOTE_RTSP_COORD_ID), rtsp_coord_process_can_frame, NULL, 0);
#ifdef HAVE_AX12
    ecan_set_rx_object(AX12_COMMAND_OBJECT, AX12_SERVO_POSITION_CAN_ID, ax12_update, NULL, 0);
#endif

    // ecan_set_rx_object(STRATEGY_COMMAND_OBJECT, STRATEGY_COMMAND_CAN_ID, display_fwd_process_can_frame_strategy, NULL, 0);
    // ecan_set_rx_object(ROBOT_STATUS_UPDATE_OBJECT, ROBOT_STATUS_UPDATE_CAN_ID, display_fwd_process_can_frame_status, NULL, 0);

    // ecan_set_rx_object(NBA_COORDINATION_COMMAND_OBJECT, NBA_COORDINATION_COMMAND_ID, nba_fwd_process_can_frame_command, NULL, 0);
    // ecan_set_rx_object(NBA_COORDINATION_STATUS_OBJECT, NBA_COORDINATION_STATUS_ID, nba_fwd_process_can_frame_status, NULL, 0);

    // La modalità FIFO è necessaria solo se ci si collega a più di un endpoint per volta
    ecan_set_rx_object(REMOTE_STDIO_RX_OBJECT, REMOTE_STDIO_CAN_ID(REMOTE_STDIO_COORD_ID),
                       canstdio_coord_process_can_frame, NULL, ECAN_RX_FLAG_ASYNC /*| ECAN_RX_FLAG_FIFO*/);

    // La modalità FIFO non è strettamente necessaria, ma aiuta a non poerdere pacchetti CAN
    ecan_set_rx_object(SIMPLE_VAR_OUTPUT_OBJECT, SIMPLE_VAR_OUTPUT_ID,
                       simple_var_output_process_can_frame, NULL, ECAN_RX_FLAG_ASYNC | ECAN_RX_FLAG_FIFO);
}

int ecan_tx(uint8_t *rx_data)
{
    int r;
    uint16_t can_id = 0;

    memcpy((void *) &can_id, (void *) rx_data, 2);

    if (can_id == ROBOT_POSITION_CAN_ID) {
        // Inoltra posizione dell'altro robot modificando il can_id
        r = ecan_send(OTHER_ROBOT_POSITION_CAN_ID, (unsigned char *) &rx_data[2], 8, 0);
        __delay_ms(CAN_DELAY_MS);
    }
    else
    {
        r = ecan_send(can_id, (unsigned char *) &rx_data[2], 8, 0);
        __delay_ms(CAN_DELAY_MS);
    }

    return r;
}
