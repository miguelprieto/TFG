/*
 * defines.h
 */

#ifndef __DEFINES_H
#define __DEFINES_H

#include "shared_defines.h"

#define ROBOT_PICCOLO
#define ROBOT_NAME "Piccolo2017"

// DISTANZE DEI BORDI RISPETTO AL CENTRO DI ROTAZIONE
#define DIM_H_TOTAL	170 // distanza davanti/dietro
#define DIM_H_BACK	85  // distanza centro/dietro
#define DIM_H_FRONT	(DIM_H_TOTAL - DIM_H_BACK) // distanza centro/davanti
#define DIM_SIDE	105 //distanza centro/lato

#define END_GAME	360	// fine gara 90 secondi

#endif
