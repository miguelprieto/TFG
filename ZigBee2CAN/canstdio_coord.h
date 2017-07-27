#ifndef CANSTDIO_COORD_H
#define CANSTDIO_COORD_H

#include <stdbool.h>
#include <stdint.h>

/* Inzializzazione strutture dati */
void canstdio_coord_init();

/* Gestore frame CAN */
void canstdio_coord_process_can_frame(const uint8_t *data, unsigned int len, void *user_ptr);

/* Gestore frame XBee */
void canstdio_coord_process_xbee_frame(const uint8_t *sender_addr, const uint8_t *data);

/* Invia richiesta di switch a modalità RTSP */
void canstdio_coord_send_request_rtsp(int node_id);

/* Viene invocato continuamente */
void canstdio_coord_idle();

/* Viene invocato 20 volte al secondo */
void canstdio_coord_interval();

#endif /* CANSTDIO_COORD_H */
