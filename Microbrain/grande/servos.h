/*
 *	servos.h
 */


#ifndef __SERVOS_H
#define __SERVOS_H

#include <stdint.h>

#include "color.h"

void servo_reset(void);
void servos_off(void);
void update_servo_status_grande(const uint8_t *data, unsigned int len, void *user_ptr);
bool automation_busy(void);

void set_baffi_start(void);
void set_abbassa_start(void);
void set_spazzola_start(void);

#endif
