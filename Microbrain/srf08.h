/*
 * srf08.h
 */

#ifndef __SRF08_H
#define __SRF08_H


void srf08_initialize(void);
void srf08_set_filter(float alpha);
float srf08_get_filter(void);
int srf08_check_ranging_complete(void);
int srf08_read_range(float * range_val);
void srf08_start_ranging(int units);

#define RANGE_MODE_INCHES       0x50
#define RANGE_MODE_CENTIMETERS  0x51
#define RANGE_MODE_MICROSECS    0x52

#endif
