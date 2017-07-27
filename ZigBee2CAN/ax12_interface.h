/*
 * ax12_interface.h
 */

#ifndef __AX12_INTERFACE_H
#define __AX12_INTERFACE_H

void ax12_init(void);
bool ax12_set_servo_position(int servo_id, int servo_position);
void ax12_update(const uint8_t *data, unsigned int len, void *user_ptr);

#endif
