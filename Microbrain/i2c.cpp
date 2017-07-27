/*
 * i2c.cpp
 */

#include "defines.h"

#include <xc.h>
#include <libpic30++.h>
#include <stdio.h>

#if defined(MICROBRAIN_BOARD_2013)
#define I2CSFR(name)	I2C1 ## name
#elif defined(MICROBRAIN_BOARD_2015)
#define I2CSFR(name)	I2C2 ## name
#endif

#define ACK         0
#define NOT_ACK     1

#define i2c_relax() { __delay_us(50); }

#define i2c_start() {                           \
	I2CSFR(CONbits).SEN = 1;                    \
	while (I2CSFR(CONbits).SEN == 1) ;          \
    }

#define i2c_stop() {                            \
	I2CSFR(CONbits).PEN = 1;                    \
	while (I2CSFR(CONbits).PEN == 1) ;          \
    }

#define i2c_restart() {                                         \
        I2CSFR(CONbits).RSEN = 1;                                   \
        while (I2CSFR(CONbits).RSEN);                               \
    }



// flag = 1 --> error in communication
#define i2c_write(data,flag){                       \
        I2CSFR(TRN) = (data);                           \
	while(I2CSFR(STATbits).TBF == 1) ;               \
	while(I2CSFR(STATbits).TRSTAT == 1) ;            \
	flag = I2CSFR(STATbits).ACKSTAT;              \
    }

#define i2c_read(data, ack){                        \
    I2CSFR(CONbits).RCEN = 1;                           \
    Nop();                                          \
    while (I2CSFR(STATbits).RBF == 0) ;                 \
    (data) = I2CSFR(RCV;)                               \
    I2CSFR(CONbits).ACKDT = ack;                        \
    I2CSFR(CONbits).ACKEN = 1;                          \
    while (I2CSFR(CONbits).ACKEN == 1) ;                \
    I2CSFR(CONbits).ACKDT = 0;                \
}

#define i2c_idle(){                             \
	while(I2CSFR(STATbits).TRSTAT) ;            \
    }

#define i2c_ack_status(){                       \
	flag = I2CSFR(STATbits).ACKSTAT;            \
    }


/* #define i2c_error() {                           \ */
/*     for (;;) {                                  \ */
/*         red_led_toggle();                       \ */
/*         violet_led_toggle();                    \ */
/*         __delay_ms(50);                         \ */
/*     }\ */
/* } */

// #define i2c_error() {                           \
//     i2c_idle();                                 \
//     i2c_stop();                                 \
//     stop_motor();                               \
//     RCONbits.SWDTEN = 0;                        \
//     for (;;) {                                  \
//         red_led_toggle();                       \
//         violet_led_toggle();                    \
//         __delay_ms(50);                         \
//     }\
// }

#define i2c_error() { printf("i2c-error in %s!\n", __func__); } //while(1); }

void i2c_setup(void)
{
    I2CSFR(BRG) = 720;   // SCL frequency = 50 KHz --> BRG = 720
    //I2CSFR(BRG) = 360;   // SCL frequency = 100 KHz --> BRG = 360
    //I2CSFR(BRG) = 180;   // SCL frequency = 200 KHz --> BRG = 180
    //I2CSFR(BRG) = 135;     // SCL frequency = 300 KHz --> BRG = 135
    //I2CSFR(BRG) = 90;    // SCL frequency = 400 KHz --> BRG = 90
    I2CSFR(CON) = 0x1200;
    I2CSFR(RCV) = 0;
    I2CSFR(TRN) = 0;
    I2CSFR(CON) = 0x9200;
}

short i2c_read_register(short address, short location, short * data)
{
    int flag;

    i2c_relax();

    i2c_idle();
    i2c_start();

    i2c_write(address,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_write(location,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_restart();
    i2c_write(address | 0x01,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_read( *data, NOT_ACK );

    i2c_stop();

    return 1;	
}

short i2c_read_16_bit_register(short address, short location, int * data)
{
    int flag;
    int h, l;

    i2c_relax();

    i2c_idle();
    i2c_start();

    i2c_write(address,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_write(location,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_restart();
    i2c_write(address | 0x01,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_read( h, ACK );

    i2c_read( l, NOT_ACK );

    i2c_stop();

    *data = (h << 8) | l;

    return 1;	
}

short i2c_read_buffer(short address, short location, unsigned char * data, int data_len)
{
    int flag;
    int i;

    i2c_relax();

    i2c_idle();
    i2c_start();

    i2c_write(address,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_write(location,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_restart();
    i2c_write(address | 0x01,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    for (i = 0; i < data_len;i++) {
        if (i == (data_len - 1)) {
            i2c_read( data[i], NOT_ACK );
        }
        else {
            i2c_read( data[i], ACK );
        }
    }

    i2c_stop();

    return 1;	
}

short i2c_write_register(short address,short location,short data)
{
    int flag;

    i2c_relax();

    i2c_idle();
    i2c_start();

    i2c_write(address,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_write(location,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_write(data,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    i2c_stop();

    return 1;
}

short i2c_read_buffer_raw(short address, unsigned char * data, int data_len)
{
    int flag;
    int i;

    i2c_relax();

    i2c_idle();
    i2c_start();

    i2c_write(address | 0x01,flag);
    if (flag == 1) {
        i2c_stop();
        i2c_error();
        return 0;
    }
    i2c_idle();

    for (i = 0; i < data_len;i++) {
        if (i == (data_len - 1)) {
            i2c_read( data[i], NOT_ACK );
        }
        else {
            i2c_read( data[i], ACK );
        }
    }

    i2c_stop();

    return 1;	
}

