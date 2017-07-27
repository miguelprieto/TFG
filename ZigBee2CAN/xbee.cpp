#include <stdio.h>
#include "defines.h"
#include "simulator_interface.h"
#include "uart.h"
#include "xbee.h"

xbee_frame_reader_state_t xbee_frame_reader_state = XBEE_STATE_START_DELIMITER;
xbee_rx_indicator_t xbee_rx_indicator, xbee_rx_indicator_copy;
uint8_t xbee_rx_indicator_arrived = 0;

static bool tx_status_received = true;

void xbee_frame_reader(uint8_t data)
{
    static uint8_t frame_position;
    static uint8_t frame_length;
    static uint8_t frame_api;
    static uint8_t checksum;

    if (xbee_frame_reader_state == XBEE_STATE_START_DELIMITER) {
        if (data == XBEE_START_DELIMITER) {
            timer2_start();

            frame_position = 0;
            checksum = 0;
            xbee_frame_reader_state = XBEE_STATE_LENGTH;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_LENGTH) {
        frame_position++;

        /* Length LSB */
        if (frame_position == 2) {
            frame_length = data;

            frame_position = 0;
            xbee_frame_reader_state = XBEE_STATE_API;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_API) {
        frame_position++;

        frame_api = data;
        checksum += data;

        switch (data) {
            case XBEE_RX64_INDICATOR:
                xbee_rx_indicator.length = frame_length;
                xbee_rx_indicator.api = XBEE_RX64_INDICATOR;
                xbee_frame_reader_state = XBEE_STATE_RX64_INDICATOR;
                break;
            case XBEE_RX16_INDICATOR:
                xbee_rx_indicator.length = frame_length;
                xbee_rx_indicator.api = XBEE_RX16_INDICATOR;
                xbee_frame_reader_state = XBEE_STATE_RX16_INDICATOR;
                break;
            case XBEE_DIO_ADC_RX64_INDICATOR:
                xbee_frame_reader_state = XBEE_STATE_DIO_ADC_RX64_INDICATOR;
                break;
            case XBEE_DIO_ADC_RX16_INDICATOR:
                xbee_frame_reader_state = XBEE_STATE_DIO_ADC_RX16_INDICATOR;
                break;
            case XBEE_AT_COMMAND_RESPONSE:
                xbee_frame_reader_state = XBEE_STATE_AT_COMMAND_RESPONSE;
                break;
            case XBEE_TX_STATUS:
                xbee_frame_reader_state = XBEE_STATE_TX_STATUS;
                break;
            case XBEE_MODEM_STATUS:
                xbee_frame_reader_state = XBEE_STATE_MODEM_STATUS;
                break;
            case XBEE_REMOTE_COMMAND_RESPONSE:
                xbee_frame_reader_state = XBEE_STATE_REMOTE_COMMAND_RESPONSE;
                break;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_RX64_INDICATOR) {
        frame_position++;
        checksum += data;

        switch (frame_position) {
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                xbee_rx_indicator.src_addr.src_addr64[frame_position - 2] = data;
                break;
            case 10:
                xbee_rx_indicator.rssi = data;
                break;
            case 11:
                xbee_rx_indicator.options = data;
                break;
            default:
                xbee_rx_indicator.rx_data[frame_position - 12] = data;
                break;
        }
        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_RX16_INDICATOR) {
        frame_position++;
        checksum += data;

        switch (frame_position) {
            case 2:
            case 3:
                xbee_rx_indicator.src_addr.src_addr16[frame_position - 2] = data;
                break;
            case 4:
                xbee_rx_indicator.rssi = data;
                break;
            case 5:
                xbee_rx_indicator.options = data;
                break;
            default:
                xbee_rx_indicator.rx_data[frame_position - 6] = data;
                break;
        }
        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_DIO_ADC_RX64_INDICATOR) {
        frame_position++;
        checksum += data;

        /* Ignore DIO/ADC Rx64 Indicator frame body */

        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_DIO_ADC_RX16_INDICATOR) {
        frame_position++;
        checksum += data;

        /* Ignore DIO/ADC Rx16 Indicator frame body */

        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_AT_COMMAND_RESPONSE) {
        frame_position++;
        checksum += data;

        /* Ignore AT Command Response frame body */

        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_TX_STATUS) {
        frame_position++;
        checksum += data;

        /* Ignore Tx Status frame body */
        tx_status_received = true;

        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_MODEM_STATUS) {
        frame_position++;
        checksum += data;

        /* Ignore Modem Status frame body */

        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_REMOTE_COMMAND_RESPONSE) {
        frame_position++;
        checksum += data;

        /* Ignore Remote Command Response frame body */

        if (frame_position == frame_length) {
            xbee_frame_reader_state = XBEE_STATE_CHECKSUM;
        }
    } else if (xbee_frame_reader_state == XBEE_STATE_CHECKSUM) {
        checksum += data;

        /* Frame VALID */
        if (checksum == 0xFF) {
            timer2_stop();

            if (frame_api == XBEE_RX64_INDICATOR || frame_api == XBEE_RX16_INDICATOR) {
                if (xbee_rx_indicator_arrived == 0) {
                    xbee_rx_indicator_copy = xbee_rx_indicator;
                    xbee_rx_indicator_arrived = 1;
                }
            }
        }
        xbee_frame_reader_state = XBEE_STATE_START_DELIMITER;
    }
}

void xbee_tx_request(const uint8_t* dest_addr, uint8_t dest_addr_length, const uint8_t* rfdata, uint8_t rfdata_length)
{
    static uint8_t frame_id = 0;
    uint8_t checksum = 0, i;

#ifdef XBEE_TX_STATUS_ENABLE
    // Attendiamo che il pacchetto precedentemente inviato sia stato processato
    while (xbee_clear_to_send() == false)
        simulator_relax();
#endif

    /* Start Delimiter */
    uart1_send(XBEE_START_DELIMITER);

    /* Length MSB */
    uart1_send(0x00);

    if (dest_addr_length == XBEE_ADDR64_LENGTH) {
        /* Length LSB */
        uart1_send(0x0b + rfdata_length);

        /* API */
        uart1_send(XBEE_TX64_REQUEST);
        checksum += XBEE_TX64_REQUEST;
    } else {
        /* Length LSB */
        uart1_send(0x05 + rfdata_length);

        /* API */
        uart1_send(XBEE_TX16_REQUEST);
        checksum += XBEE_TX16_REQUEST;
    }

#ifdef XBEE_TX_STATUS_ENABLE
    tx_status_received = false;
    /* FrameID */
    frame_id++;
    if (frame_id == 0)     /* If FrameID = 0 no Tx Status will be received */
        frame_id = 1;
#endif
    uart1_send(frame_id);
    checksum += frame_id;

    /* DestAddr */
    for (i = 0; i < dest_addr_length; i++) {
        uart1_send(dest_addr[i]);
        checksum += dest_addr[i];
    }

    /* Options */
    uart1_send(0x00);
    checksum += 0x00;

    /* RFData */
    for (i = 0; i < rfdata_length; i++) {
        uart1_send(rfdata[i]);
        checksum += rfdata[i];
    }

    /* Checksum */
    uart1_send(0xFF - checksum);
}

bool xbee_clear_to_send()
{
	return tx_status_received;
}
