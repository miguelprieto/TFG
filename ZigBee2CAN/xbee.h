#ifndef XBEE_H
#define	XBEE_H

#include <stdbool.h>

#define XBEE_TX_STATUS_ENABLE

#define XBEE_START_DELIMITER                    0x7e

#define XBEE_TX64_REQUEST                       0x00
#define XBEE_TX16_REQUEST                       0x01
#define XBEE_AT_COMMAND                         0x08
#define XBEE_AT_COMMAND_QUEUE_REGISTER_VALUE    0x09
#define XBEE_REMOTE_AT_COMMAND                  0x17
#define XBEE_RX64_INDICATOR                     0x80
#define XBEE_RX16_INDICATOR                     0x81
#define XBEE_DIO_ADC_RX64_INDICATOR             0x82
#define XBEE_DIO_ADC_RX16_INDICATOR             0x83
#define XBEE_AT_COMMAND_RESPONSE                0x88
#define XBEE_TX_STATUS                          0x89
#define XBEE_MODEM_STATUS                       0x8a
#define XBEE_REMOTE_COMMAND_RESPONSE            0x97

#define XBEE_RX_INDICATOR_MAX_RX_DATA_LENGTH             100
#define XBEE_DIO_ADC_RX_INDICATOR_MAX_RX_DATA_LENGTH     100
#define XBEE_AT_COMMAND_RESPONSE_CMD_DATA_MAX_LENGTH     10
#define XBEE_REMOTE_COMMAND_RESPONSE_CMD_DATA_MAX_LENGTH 10

#define XBEE_ADDR64_LENGTH  8
#define XBEE_ADDR16_LENGTH  2

typedef enum {
    XBEE_STATE_START_DELIMITER = 0, 
    XBEE_STATE_LENGTH, 
    XBEE_STATE_API, 
    XBEE_STATE_RX64_INDICATOR, 
    XBEE_STATE_RX16_INDICATOR, 
    XBEE_STATE_DIO_ADC_RX64_INDICATOR, 
    XBEE_STATE_DIO_ADC_RX16_INDICATOR, 
    XBEE_STATE_AT_COMMAND_RESPONSE, 
    XBEE_STATE_TX_STATUS, 
    XBEE_STATE_MODEM_STATUS, 
    XBEE_STATE_REMOTE_COMMAND_RESPONSE,
    XBEE_STATE_CHECKSUM
} xbee_frame_reader_state_t;

typedef struct {
    uint8_t length;
    uint8_t api;
    union {
        uint8_t src_addr64[8];
        uint8_t src_addr16[2];
    } src_addr;
    uint8_t rssi;
    uint8_t options;
    uint8_t rx_data[XBEE_RX_INDICATOR_MAX_RX_DATA_LENGTH];
    uint8_t checksum;
} xbee_rx_indicator_t;

typedef struct {
    uint8_t length;
    uint8_t api;
    uint8_t frame_id;
    union {
        uint8_t src_addr64[8];
        uint8_t src_addr16[2];
    } src_addr;
    uint8_t rssi;
    uint8_t options;
    uint8_t rx_data[XBEE_DIO_ADC_RX_INDICATOR_MAX_RX_DATA_LENGTH];
    uint8_t checksum;
} xbee_dio_adc_rx_indicator_t;

typedef struct {
    uint8_t length;
    uint8_t api;
    uint8_t frame_id;
    uint8_t at_cmd;
    uint8_t cmd_status;
    uint8_t cmd_data[XBEE_AT_COMMAND_RESPONSE_CMD_DATA_MAX_LENGTH];
    uint8_t checksum;
} xbee_at_command_response_t;

typedef struct {
    uint8_t length;
    uint8_t api;
    uint8_t frame_id;
    uint8_t status;
    uint8_t checksum;
} xbee_tx_status_t;

typedef struct {
    uint8_t length;
    uint8_t api;
    uint8_t modem_status;
    uint8_t checksum;
} xbee_modem_status_t;

typedef struct {
    uint8_t length;
    uint8_t api;
    uint8_t frame_id;
    uint8_t rem_addr64[8];
    uint8_t rem_addr16[2];
    uint8_t at_cmd;
    uint8_t rem_status;
    uint8_t cmd_data[XBEE_REMOTE_COMMAND_RESPONSE_CMD_DATA_MAX_LENGTH];
    uint8_t checksum;
} xbee_remote_command_response_t;

void xbee_frame_reader(uint8_t data);
void xbee_tx_request(const uint8_t* dest_addr, uint8_t dest_addr_length, const uint8_t* rfdata, uint8_t rfdata_length);
bool xbee_clear_to_send();

#endif
