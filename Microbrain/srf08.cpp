/*
 * srf08.c
 */

#include "defines.h"

#include <libpic30++.h>
#include <stdio.h>

#include "i2c.h"

enum {
    W_COMMAND = 0,
    W_GAIN,
    W_RANGE
} ;

enum {
    R_SOFTWARE_REV = 0,
    R_LIGHT_SENSOR,
    R_ECHO_FIRST_HIGH,
    R_ECHO_FIRST_LOW
};

#define SRF08_ADDRESS   0xe8

float srf08_filter, srf08_current = 0;

void srf08_set_filter(float alpha)
{
    srf08_filter = alpha;
}

float srf08_get_filter(void)
{
    return srf08_filter;
}

void srf08_write(int reg, int value)
{
    i2c_write_register(SRF08_ADDRESS, reg, value);
}

void srf08_initialize(void)
{

/* 0  0x00 Set Maximum Analogue Gain to 94 */
/* 1  0x01 Set Maximum Analogue Gain to 97 */
/* 2  0x02 Set Maximum Analogue Gain to 100 */
/* 3  0x03 Set Maximum Analogue Gain to 103 */
/* 4  0x04 Set Maximum Analogue Gain to 107 */
/* 5  0x05 Set Maximum Analogue Gain to 110 */
/* 6  0x06 Set Maximum Analogue Gain to 114 */
/* 7  0x07 Set Maximum Analogue Gain to 118 */
/* 8  0x08 Set Maximum Analogue Gain to 123 */
/* 9  0x09 Set Maximum Analogue Gain to 128 */
/* 10 0x0A Set Maximum Analogue Gain to 133 */
/* 11 0x0B Set Maximum Analogue Gain to 139 */
/* 12 0x0C Set Maximum Analogue Gain to 145 */
/* 13 0x0D Set Maximum Analogue Gain to 152 */
/* 14 0x0E Set Maximum Analogue Gain to 159 */
/* 15 0x0F Set Maximum Analogue Gain to 168 */
/* 16 0x10 Set Maximum Analogue Gain to 177 */
/* 17 0x11 Set Maximum Analogue Gain to 187 */
/* 18 0x12 Set Maximum Analogue Gain to 199 */
/* 19 0x13 Set Maximum Analogue Gain to 212 */
/* 20 0x14 Set Maximum Analogue Gain to 227 */
/* 21 0x15 Set Maximum Analogue Gain to 245 */
/* 22 0x16 Set Maximum Analogue Gain to 265 */
/* 23 0x17 Set Maximum Analogue Gain to 288 */
/* 24 0x18 Set Maximum Analogue Gain to 317 */

    srf08_write(W_GAIN, 8);
    srf08_write(W_RANGE, 255);
}

int srf08_check_ranging_complete(void)
{
    unsigned char c;

    //puts("srf08_check_ranging_complete");
    if (i2c_read_buffer(SRF08_ADDRESS, R_SOFTWARE_REV, &c, 1) == 0)
        return 0;
    else
        return (c != 255);
}

int srf08_read_range(float * range_val)
{
    unsigned char c[2];
    int range;
    float f_range;

    //puts("srf08_read_range");
    if (i2c_read_buffer(SRF08_ADDRESS, R_ECHO_FIRST_HIGH, c, 2) == 0)
        return 0;
    else {
        range = ((c[0] << 8) | c[1]);
        f_range = range * 0.03 / 2.0;
        srf08_current = f_range * srf08_filter + srf08_current * (1 - srf08_filter);
        *range_val = srf08_current;
        return 1;
    }
}


void srf08_start_ranging(int units)
{
    srf08_write(W_COMMAND, units);
}

