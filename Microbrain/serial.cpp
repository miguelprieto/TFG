/*
 * serial.c
 */

#include "defines.h"

#include <xc.h>
#include <libpic30++.h>
#include <stdio.h>
#include "serial.h"

#include <string.h>

void serial2_start(int use_interrupt, int baud_rate)
{
    //INIZIALIZZAZIONE SERIALE
#if defined(MICROBRAIN_BOARD_2013)
    ODCBbits.ODCB12 = 0; // no open drain
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB7 = 1;
    RPINR19bits.U2RXR = 0b0111;    // U2<-RX RP7 (RB7)
    RPOR6bits.RP12R = 0b00101;     // U2->TX RP12 (RB12)
#elif defined(MICROBRAIN_BOARD_2015)
    ODCBbits.ODCB8 = 0; // no open drain
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB11 = 1;
    RPINR19bits.U2RXR = 0b0101011; // U2<-RX RPI43 (RB11)
    RPOR3bits.RP40R = 0b000011;    // U2->TX RP40 (RB8)
#endif

    //SET SERIAL
    U2MODEbits.STSEL = 0; // 1-stop bit
    U2MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U2MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U2MODEbits.BRGH = 0; // Low Speed mode
    U2BRG = baud_rate; // BAUD Rate Setting for 115200

    U2MODEbits.UEN = 0b00;
    U2STAbits.URXISEL = 0b00;

    U2STAbits.UTXISEL0 = 0;
    U2STAbits.UTXISEL1 = 0;

    //#ifndef DEBUG_232
    IPC7bits.U2RXIP = 0b111; // set hightest level priority
    IFS1bits.U2RXIF = 0; // clear interrupt flag
    IEC1bits.U2RXIE = use_interrupt; // set RX interrupt

    IEC1bits.U2TXIE = 0; // no TX interrupt

    //IPC2bits.U1RXIP = 2;
    U2MODEbits.UARTEN = 1; // Enable UART

    U2STAbits.UTXEN = 1; // Enable UART Tx

    //LATBbits.LATB7 = 0; //Scrittura su 485 disabilitata
    //LATBbits.LATB11 = 1; //Tx line set to 1
}

void serial2_stop()
{
    U2MODEbits.UARTEN = 0;
}

bool serial2_is_input_available()
{
	if (U2STAbits.OERR == 1)
		U2STAbits.OERR = 0; // clear overrun if it occurs

	return U2STAbits.URXDA == 1;
}

char serial2_getcharacter(bool blocking)
{
	while (blocking && !serial2_is_input_available());

	if (serial2_is_input_available())
		return U2RXREG;
	else
		return 0;
}

void serial2_send(const char *data, int data_len)
{
	while (data_len-- != 0)
	{
		while (U2STAbits.UTXBF != 0);
		U2TXREG = *data++;
	}
}
