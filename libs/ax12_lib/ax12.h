/*
 * ax12.h
 */

#ifndef __AX12_H
#define __AX12_H

#include <stdint.h>
#include "serial.h"

typedef int t_dymx_error;

void dynamixel_rx_handler(char ch);

class Dynamixel {
 public:
    Dynamixel(UART_BASE & uart);
    void setup(int speed, int RB_TX, int RB_RX, bool tx_open_drain, bool tx_interrupt, bool rx_interrupt);
    void setup_uart_speed(int speed);

    t_dymx_error reset(uint8_t id);
    t_dymx_error ping (uint8_t id);
    t_dymx_error write_register_8(uint8_t id, int reg, uint8_t value);
    t_dymx_error write_register_16(uint8_t id, int reg, uint16_t value);

    t_dymx_error read_register_8(uint8_t id, int reg, uint8_t & value);
    t_dymx_error read_register_16(uint8_t id, int reg, uint16_t & value);

    t_dymx_error set_servo_led(uint8_t ID, bool state);
    t_dymx_error move(uint8_t ID, int position);
    t_dymx_error degrees(uint8_t ID, float position);
    t_dymx_error torque(uint8_t ID, bool state);
    t_dymx_error current_position(uint8_t id, int & position);
    t_dymx_error current_angle(uint8_t id, float & position);

    friend void dynamixel_rx_handler(char ch);

    t_dymx_error set_new_speed(uint8_t id, int new_pic_speed);
    t_dymx_error set_new_id(uint8_t ID, uint8_t new_id);

 private:
    UART_BASE & m_uart;
    bool m_rx_on, m_notification;
    t_dymx_error protocol_transaction(uint8_t id, uint8_t instruction, uint8_t * parameters, int param_size);
    bool wait();
    void notify();
};

#define AX12_PING        1
#define AX12_READ        2
#define AX12_WRITE       3
#define AX12_REG_WRITE   4
#define AX12_ACTION      5
#define AX12_RESET       6
#define AX12_SYNC_WRITE  0x83

// values to be set to register 4 of the AX12 to change the serial speed
#define AX12_LOCALSPEED_1M     1
#define AX12_LOCALSPEED_57600  34
#define AX12_LOCALSPEED_20408  97
#define AX12_LOCALSPEED_13071  152
#define AX12_LOCALSPEED_58K    33

#define AX12_PIC_SPEED_1M      9
#define AX12_PIC_SPEED_57600   (B57600+1)
#define AX12_PIC_SPEED_20408   117
#define AX12_PIC_SPEED_13071   184
#define AX12_PIC_SPEED_58K     40
// ----------------+-----------------+-----------
//        AX12     |       dsPIC     |
//   Speed   Value |   Speed  Value  |  Error (%)
// ----------------+-----------------+-----------
//     58823   33  |    58982   40   |    0.27



// errors

#define ERR_NO_ERROR              0
#define ERR_BAD_PARAM             -1
#define ERR_TIMEOUT               0x80
#define ERR_BAD_INSTRUCTION_BIT   0x40
#define ERR_OVERLOAD_BIT          0x20
#define ERR_CHECKSUM_BIT          0x10
#define ERR_RANGE_BIT             0x08
#define ERR_OVERHEATING_BIT       0x04
#define ERR_ANGLE_LIMIT_BIT       0x02
#define ERR_VOLTAGE_BIT           0x01

// AX12 Registers

#define AX12_MODEL_NUMBER_L          0
#define AX12_MODOEL_NUMBER_H         1
#define AX12_VERSION                 2
#define AX12_ID                      3
#define AX12_BAUD_RATE               4
#define AX12_RETURN_DELAY_TIME       5
#define AX12_CW_ANGLE_LIMIT_L        6
#define AX12_CW_ANGLE_LIMIT_H        7
#define AX12_CCW_ANGLE_LIMIT_L       8
#define AX12_CCW_ANGLE_LIMIT_H       9
#define AX12_SYSTEM_DATA2            10
#define AX12_LIMIT_TEMPERATURE       11
#define AX12_DOWN_LIMIT_VOLTAGE      12
#define AX12_UAX12_LIMIT_VOLTAGE     13
#define AX12_MAX_TORQUE_L            14
#define AX12_MAX_TORQUE_H            15
#define AX12_RETURN_LEVEL            16
#define AX12_ALARM_LED               17
#define AX12_ALARM_SHUTDOWN          18
#define AX12_OPERATING_MODE          19
#define AX12_DOWN_CALIBRATION_L      20
#define AX12_DOWN_CALIBRATION_H      21
#define AX12_UP_CALIBRATION_L        22
#define AX12_UP_CALIBRATION_H        23
#define AX12_TORQUE_ENABLE           24
#define AX12_LED                     25
#define AX12_CW_COMPLIANCE_MARGIN    26
#define AX12_CCW_COMPLIANCE_MARGIN   27
#define AX12_CW_COMPLIANCE_SLOPE     28
#define AX12_CCW_COMPLIANCE_SLOPE    29
#define AX12_GOAL_POSITION_L         30
#define AX12_GOAL_POSITION_H         31
#define AX12_GOAL_SPEED_L            32
#define AX12_GOAL_SPEED_H            33
#define AX12_TORQUE_LIMIT_L          34
#define AX12_TORQUE_LIMIT_H          35
#define AX12_PRESENT_POSITION_L      36
#define AX12_PRESENT_POSITION_H      37
#define AX12_PRESENT_SPEED_L         38
#define AX12_PRESENT_SPEED_H         39
#define AX12_PRESENT_LOAD_L          40
#define AX12_PRESENT_LOAD_H          41
#define AX12_PRESENT_VOLTAGE         42
#define AX12_PRESENT_TEMPERATURE     43
#define AX12_REGISTERED_INSTRUCTION  44
// 45 reserved
#define AX12_MOVING                  46
#define AX12_LOCK                    47
#define AX12_PUNCH_L                 48
#define AX12_PUNCH_H                 49


#endif
