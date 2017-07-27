#include <xc.h>
#include "defines.h"
#include "xbee.h"

/*
 * USART1 (connected to ZigBee module).
 */
void uart1_initialize(uint16_t baud_rate, uint8_t interrupt)
{
    TRISBbits.TRISB4 = 1;           // Set RB4 pin as input
    TRISBbits.TRISB5 = 0;           // Set RB5 pin as output
    ODCBbits.ODCB5 = 0;

    RPINR18bits.U1RXR = 0b00100;    // Assign UART1 Rx to RP4
    RPOR2bits.RP5R = 0b00011;       // Assign UART1 Tx to RP5

    U1MODEbits.STSEL = 0;           // 1 stop bit
    U1MODEbits.PDSEL = 0;           // No Parity, 8 data bits
    U1MODEbits.ABAUD = 0;           // Auto-Baud Disabled
    U1MODEbits.BRGH = 0;            // Low Speed mode
    U1BRG = baud_rate;              // BAUD Rate Setting

    U1MODEbits.UEN = 0b00;
    U1STAbits.URXISEL = 0b00;

    U1STAbits.UTXISEL0 = 0;         // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;

    IPC2bits.U1RXIP = 0b111;        // Set hightest level priority
    IFS0bits.U1RXIF = 0;            // Clear interrupt flag
    IEC0bits.U1RXIE = interrupt;    // Set RX interrupt
    IEC0bits.U1TXIE = 0;            // No TX interrupt
}

void uart1_start(void)
{
    U1MODEbits.UARTEN = 1;          // Enable UART1
    U1STAbits.UTXEN = 1;            // Enable UART1 Tx
}

void uart1_stop(void)
{
    U1MODEbits.UARTEN = 0;          // Disable UART1
}

void uart1_send(uint8_t data)
{
    while (U1STAbits.TRMT == 0);    // If TRMT == 1 buffer is empty
    U1TXREG = data;
}

extern "C" void __attribute__((__interrupt__, __auto_psv__, __shadow__)) _U1RXInterrupt(void)
{
    uint8_t data;
    data = U1RXREG;
    xbee_frame_reader(data);
    
    IFS0bits.U1RXIF = 0;	        // Clear interrupt flag
}

/*
 * USART2 (connected to Bluetooth module).
 */
void uart2_initialize(uint16_t baud_rate, uint8_t interrupt)
{
    TRISBbits.TRISB11 = 1;          // Set RB10 pin as input
    TRISBbits.TRISB10 = 0;          // Set RB11 pin as output

    RPINR19bits.U2RXR = 0b01011;    // Assign UART2 Rx to RP11
    RPOR5bits.RP10R = 0b00101;      // Assign UART2 Tx to RP10

    U2MODEbits.STSEL = 0;           // 1 stop bit
    U2MODEbits.PDSEL = 0;           // No Parity, 8 data bits
    U2MODEbits.ABAUD = 0;           // Auto-Baud Disabled
    U2MODEbits.BRGH = 0;            // Low Speed mode
    U2BRG = baud_rate;              // BAUD Rate Setting

    U2MODEbits.UEN = 0b00;
    U2STAbits.URXISEL = 0b00;

    U2STAbits.UTXISEL0 = 0;         // Interrupt after one TX Character is transmitted
    U2STAbits.UTXISEL1 = 0;

    IPC7bits.U2RXIP = 0b111;        // Set hightest level priority
    IFS1bits.U2RXIF = 0;            // Clear interrupt flag
    IEC1bits.U2RXIE = interrupt;    // Set Rx interrupt
    IEC1bits.U2TXIE = 0;            // No TX interrupt
}

void uart2_start(void)
{
    U2MODEbits.UARTEN = 1;          // Enable UART2
    U2STAbits.UTXEN = 1;            // Enable UART2 Tx
}

void uart2_stop(void)
{
    U2MODEbits.UARTEN = 0;          // Disable UART2
}

void uart2_send(uint8_t data)
{
    while (U2STAbits.TRMT == 0);    // If TRMT == 1 buffer is empty
    U2TXREG = data;
}
