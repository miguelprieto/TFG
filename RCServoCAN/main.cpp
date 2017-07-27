/*
 * main.cpp
 */


#include "defines.h"

#include <p33FJ128MC802.h>
#include <stdio.h>
#include <stdlib.h>
#include <libpic30++.h>
#include <string.h>
#include <math.h>

#include "bus_interface.h"
#include "clocking.h"
#include "canstdio_endpoint.h"
#include "led.h"
#include "servo.h"
#include "servo_pins.h"
#include "ecan_lib.h"
#include "bus_objects.h"
#include "automation.h"

_FPOR(PWMPIN_ON & FPWRT_PWR1 & ALTI2C_OFF);

_FICD(ICS_PGD3 & JTAGEN_OFF);

_FWDT(FWDTEN_OFF);

static float lift_position;
static int lift_status;

void initialize_peripherals(void)
{
    int i;

    init_clock();
    init_servo_pins();
    set_led();

    SRbits.IPL = 0;  // enable all interrupts

    IEC0 = 0;				// DISABLE ALL USER INTERRUPT
    IEC1 = 0;				
    IEC2 = 0;
    IEC3 = 0;	
    IEC4 = 0;	

    for (i = 0;i < 2;i++) {
        __delay_ms(200);
        led_on();
        __delay_ms(200);
        led_off();
    }
    led_on();

    initialize_servo();

    RCONbits.SWDTEN   = 0;	// disable Watchdog Timer

    AD1PCFGL = 0xffff;		//porte abilitate in digitale

#ifdef USE_ECAN
    // RX is RP5, TX is RP4 in open_drain configuration
    ecan_initialize(5, 4, true, true);
#else
    init_uart(0, B115200);
#endif

    init_automation();

}

void update_servo_position(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_servo_position *servo_pos_object = (const t_servo_position*)data;

    led_toggle();

    if (servo_pos_object->servo_num >= SERVO_BASE && servo_pos_object->servo_num < SERVO_BASE + NUM_SERVO) {
        set_servo(servo_pos_object->servo_num - SERVO_BASE, servo_pos_object->position);
    }
    else if (servo_pos_object->servo_num == SERVO_POSITION_AUTOMATION_COMMAND) {
        automation_command(servo_pos_object);
    }
}

static void update_lift_telemetry(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_lift_telemetry_encoder_data *m = (const t_can_lift_telemetry_encoder_data*)data;

    if (m->frame_id == 0x10 && m->sub_id == 0x01)
    {
        lift_position = m->value;
        lift_status = m->bumper;
    }
}

void init_bus_objects(void)
{
    ecan_set_rx_object(SERVO_POSITION_OBJECT, SERVO_POSITION_CAN_ID, update_servo_position, NULL, ECAN_RX_FLAG_FIFO);
    ecan_set_rx_object(REMOTE_STDIO_RX_OBJECT, REMOTE_STDIO_CAN_ID(CAN_STDIO_AND_RTSP_NODE_ID),
                       canstdio_endpoint_process_can_frame, NULL, ECAN_RX_FLAG_ASYNC);
    ecan_set_rx_object(LIFT_TELEMETRY_OBJECT, LIFT_TELEMETRY_DATA_CAN_ID, update_lift_telemetry, NULL, 0);
}

void send_lift_position(int lift, int pos)
{
    t_command_position_generic p;

    p._cmd = POSITION_GO;
    p.axis = lift;
    p.position = pos;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}

float get_lift_position()
{
    ecan_update_object(LIFT_TELEMETRY_OBJECT);
    return lift_position;
}

bool lift_check_position(float target_pos)
{
    return fabs(get_lift_position() - target_pos) < 5;
}

void lift_home(int lift)
{
    t_command_position_generic p;

    p._cmd = POSITION_HOME;
    p.axis = lift;
    p.position = 0;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}


void lift_stop(int lift)
{
    t_command_position_generic p;

    p._cmd = POSITION_OFF;
    p.axis = lift;
    p.position = 0;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}



void send_valve_command(int valve, int cmd)
{
    t_command_valve p;

    p.valve = valve;
    p.cmd = cmd;
    ecan_send(VALVE_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(50);
}


void send_gpio(unsigned char board_num,
               unsigned char s1,
               unsigned char s2,
               unsigned char o1,
               unsigned char o2,
               unsigned char o3)
{
    t_servo_gpio p;

    p.board_num = board_num;
    p.s1 = s1;
    p.s2 = s2;
    p.o1 = o1;
    p.o2 = o2;
    p.o3 = o3;
    ecan_send(SERVO_GPIO_CAN_ID, (unsigned char *)&p, 8, 0);
}


int main(void)
{
    initialize_peripherals();
    init_bus_objects();

    while (1) {
        ecan_update_object(SERVO_POSITION_OBJECT);
#ifndef SERVO_INTERRUPT
        if (IFS0bits.T2IF == 1) {
            update_servos();
            if (servo_turn == 0) {
                automation();
            }
        }
#endif
    }

    return 0;
}
