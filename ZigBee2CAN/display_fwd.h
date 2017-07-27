#ifndef DISPLAY_FWD_H
#define DISPLAY_FWD_H

#include <stdbool.h>
#include <stdint.h>

/* Gestore frame CAN */
void display_fwd_process_can_frame_strategy(const uint8_t *data, unsigned int len, void *user_ptr);
void display_fwd_process_can_frame_status(const uint8_t *data, unsigned int len, void *user_ptr);

/* Viene invocato 4 volte al secondo */
void display_fwd_interval();

#endif /* DISPLAY_FWD_H */
