/*
 * shared_defines.h
 */

#ifndef __SHARED_DEFINES_H
#define __SHARED_DEFINES_H

#include <stdbool.h>

typedef enum
{
	BACK,
	FRONT
} dir_t;

#if defined(__dsPIC33FJ128MC802__)
#define MICROBRAIN_BOARD_2013
#elif defined(__dsPIC33EP512MC502__)
#define MICROBRAIN_BOARD_2015
#else
#error Unsupported device: neither __dsPIC33FJ128MC802__ nor __dsPIC33EP512MC502__ is defined!
#endif

#define FCY   (77385000 / 2)

extern volatile int game_timer;
extern int start_time; // istante di inizio gara
extern bool game_started;

#endif
