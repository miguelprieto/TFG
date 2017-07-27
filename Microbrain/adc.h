/*
 * adc.h
 */

#ifndef __ADC_H
#define __ADC_H
#include <stdint.h>
#include <gpio.h>

void init_adc(void);
uint16_t get_sensor(int channel);

#endif
