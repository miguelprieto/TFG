#ifndef RTSP_COORD_H
#define RTSP_COORD_H

#include <stdbool.h>
#include <stdint.h>

/* Inzializzazione strutture dati */
void rtsp_coord_init();

/* Gestore frame CAN */
void rtsp_coord_process_can_frame(const uint8_t *data, unsigned int len, void *user_ptr);

/* Gestore frame XBee */
void rtsp_coord_process_xbee_frame(const uint8_t *sender_addr, const uint8_t *data);

/* Viene invocato 20 volte al secondo */
void rtsp_coord_interval();

#endif /* RTSP_COORD_H */
