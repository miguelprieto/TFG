#ifndef NBA_FWD_H
#define NBA_FWD_H

#include <stdbool.h>
#include <stdint.h>

/* Gestore frame CAN */
void nba_fwd_process_can_frame_command(const uint8_t *data, unsigned int len, void *user_ptr);
void nba_fwd_process_can_frame_status(const uint8_t *data, unsigned int len, void *user_ptr);

/* Viene invocato 4 volte al secondo */
void nba_fwd_interval();

#endif /* NBA_FWD_H */
