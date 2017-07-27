#include "defines.h"
#include <xc.h>
#include <libpic30++.h>
#include <stdio.h>
#include "bus_interface.h"
#include "canstdio_coord.h"
#include "clock.h"
#include "ecan_lib.h"
#include "led.h"
#include "rtsp_coord.h"
#include "simple_var_output.h"
#include "simulator_interface.h"
#include "uart.h"
#include "xbee.h"
#ifdef HAVE_AX12
#include "ax12_interface.h"
#endif

_FWDT(FWDTEN_OFF);
_FPOR(PWMPIN_OFF & FPWRT_PWR1 & ALTI2C_OFF);
_FICD(ICS_PGD1 & JTAGEN_OFF);

void peripherals_initialize(void);
void timer2_initialize(void);
void timer3_initialize(void);
void timer3_start(void);
void timer3_stop(void);

extern xbee_frame_reader_state_t xbee_frame_reader_state;
extern xbee_rx_indicator_t xbee_rx_indicator_copy;
extern uint8_t xbee_rx_indicator_arrived;

void T3SoftInterrupt(void);

int main(void)
{
    peripherals_initialize();
    canstdio_coord_init();
    rtsp_coord_init();


#ifndef HAVE_AX12
    __C30_UART = 2;
    printf("Starting... %x\n", RCON);
#else
    ax12_init();
#endif

    __delay_ms(1000);

    for (;;) {
        if (xbee_rx_indicator_arrived) {
            if (xbee_rx_indicator_copy.rx_data[0] == 0xFF && xbee_rx_indicator_copy.rx_data[1] == 0xFF)
                canstdio_coord_process_xbee_frame(xbee_rx_indicator_copy.src_addr.src_addr16, xbee_rx_indicator_copy.rx_data + 2);
            else if (xbee_rx_indicator_copy.rx_data[0] == 0xFF && xbee_rx_indicator_copy.rx_data[1] == 0xFE)
                rtsp_coord_process_xbee_frame(xbee_rx_indicator_copy.src_addr.src_addr16, xbee_rx_indicator_copy.rx_data);
            else
                ecan_tx(xbee_rx_indicator_copy.rx_data);

            red_led_toggle();
            xbee_rx_indicator_arrived = 0;
        }

        if (IFS0bits.T3IF == 1)
            T3SoftInterrupt();
        else
        {
            ecan_update_object(TELEMETRY_OBJECT);
#ifdef HAVE_AX12
            ecan_update_object(AX12_COMMAND_OBJECT);
#endif
            canstdio_coord_idle();
            simulator_relax();
            simple_var_output_relax();
        }
    }
    return 0;
}

void peripherals_initialize(void)
{
    int i;

    clock_initialize();

    IEC0 = 0;                           // Disable all user interrupts
    IEC1 = 0;
    IEC2 = 0;
    IEC3 = 0;
    IEC4 = 0;
    RCONbits.SWDTEN = 0;                // Disable Watchdog Timer

    led_initialize();                   // Set Green/Red Leds

    TRISAbits.TRISA3 = 0;               // Set ZBRESET Pin
    LATAbits.LATA3 = 1;

    PWM1CON1 = 0;                       // Disable PWM
    PWM2CON1 = 0;

    for (i = 0; i < 10; i++) {
        __delay_ms(100);
        green_led_toggle();
    }

    /* CAN Initialization */
    ecan_initialize(6, 7, true, true);

    RPOR6bits.RP12R = 0;

    bus_objects_initialize();

    /* UART1 and UART2 Initialization */
    uart1_initialize(B57600, 1);  // Initialize USART1 (connected to ZigBee)
    uart1_start();                // Start UART1

#ifndef HAVE_AX12
    uart2_initialize(B115200, 0); // Initialize USART2 (connected to Bluetooth)
    uart2_start();                // Start UART2
#endif

    timer2_initialize();
    timer3_initialize();
    timer3_start();

}

void timer2_initialize(void)
{
    T2CONbits.TON = 0;      // Disable Timer2
    T2CONbits.TSIDL = 0;
    T2CONbits.T32 = 0 ;
    T2CONbits.TCS = 0;      // Select internal instruction cycle clock
    T2CONbits.TGATE = 0;    // Disable Gated Timer mode

    T2CONbits.TCKPS = 0b11; // Select 1:256 Prescaler

    IPC1bits.T2IP = 1;	    // Set Timer2 Interrupt Priority Level
    IFS0bits.T2IF = 0; 	    // Clear Timer2 Interrupt Flag
    IEC0bits.T2IE = 1;      // Enable Timer2 interrupt
}

void timer2_start(void)
{
    T2CONbits.TON = 0;      // Disable Timer2

    TMR2 = 0;
    PR2 = 18890;            // period_timer(s)*((Fosc)/(2*prescaler)) 125ms

    T2CONbits.TON = 1;      // Enable Timer2
}

void timer2_stop(void)
{
    T2CONbits.TON = 0;      // Disable Timer2
}

extern "C" void __attribute__((__interrupt__, __auto_psv__, __shadow__)) _T2Interrupt(void)
{
    int i;

    for (i = 0; i < 5; i++) {
        __delay_ms(50);
        red_led_toggle();
    }

    xbee_frame_reader_state = XBEE_STATE_START_DELIMITER;

    IFS0bits.T2IF = 0; 	    // Clear Timer2 Interrupt Flag
}

void timer3_initialize(void)
{
    T3CONbits.TON = 0;      // Disable Timer3
    T3CONbits.TSIDL = 0;
    T3CONbits.TCS = 0;      // Select internal instruction cycle clock
    T3CONbits.TGATE = 0;    // Disable Gated Timer mode

    T3CONbits.TCKPS = 0b11; // Select 1:256 Prescaler

    TMR3 = 0;
    PR3 = 7557;             // period_timer(s)*((Fosc)/(2*prescaler)) 50ms

    IPC2bits.T3IP = 1;	    // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0; 	    // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 0;      // No Enable Timer3 interrupt
}

void timer3_start(void)
{
    T3CONbits.TON = 1;      // Enable Timer3
}

void timer3_stop(void)
{
    T3CONbits.TON = 0;      // Disable Timer3
}

//extern "C" void __attribute__((__interrupt__, __auto_psv__, __shadow__)) _T3Interrupt(void)
void T3SoftInterrupt(void)
{
    static uint8_t count = 0;

    canstdio_coord_interval();
    rtsp_coord_interval();

    if (count++ == 5)       // Every 250ms
    {
        ecan_update_object(ROBOT_POSITION_OBJECT);
        count = 0;
    }
    // else if (count == 1)
    // {
    //     display_fwd_interval();
    // }
    // else if (count == 2)
    // {
    //     nba_fwd_interval();
    // }

    IFS0bits.T3IF = 0; 	    // Clear Timer3 Interrupt Flag
}
