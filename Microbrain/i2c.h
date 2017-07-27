/*
 * i2c.h
 */

#ifndef __I2C_H
#define __I2C_H

void i2c_setup(void);
short i2c_read_register(short address, short location, short * data);
short i2c_read_16_bit_register(short address, short location, int * data);
short i2c_read_buffer(short address, short location, unsigned char * data, int data_len);
short i2c_write_register(short address,short location,short data);
short i2c_read_buffer_raw(short address, unsigned char * data, int data_len);


#endif
