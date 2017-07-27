/*
 * ax12_interface.cpp
 */

#include "defines.h"

#ifdef HAVE_AX12

#include <p33FJ128MC802.h>
#include <libpic30++.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "clocking.h"
#include "serial.h"
#include "bus_objects.h"
#include "ax12.h"


UART_2 uart2;
Dynamixel ax12(uart2);


void ax12_init(void)
{
    // TXPIN = RB8/RP8
    // RXPIN = RB9/RP9
    // open drain configuration: yes
    // TX interrupt: no
    // RX interrupt: yes
    ax12.setup(AX12_PIC_SPEED_58K, 8, 9, true, false, true);

    __delay_ms(500);

    ax12.set_servo_led(0xfe, true);
}


bool ax12_set_servo_position(int servo_id, int servo_position)
{
    if (servo_position == -1)
        return ax12.torque(servo_id, false);
    else
        return ax12.move(servo_id, servo_position);
}


void ax12_update(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_servo_position *servo = (const t_servo_position*)data;
    ax12_set_servo_position(servo->servo_num, servo->position);
}



#endif
