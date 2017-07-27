/*
 * defines.h
 */

#ifndef __DEFINES_H
#define __DEFINES_H

#include "shared_defines.h"

#define ROBOT_GRANDE
#define ROBOT_NAME "Grande2017"

// DISTANZE DEI BORDI RISPETTO AL CENTRO DI ROTAZIONE
#define DIM_H_TOTAL	270 			// Distanza davanti/dietro
#define DIM_H_BACK	61 				// Distanza centro/dietro
#define DIM_H_FRONT	(DIM_H_TOTAL - DIM_H_BACK) 	// Distanza centro/davanti
#define DIM_SIDE	170 			// Distanza centro/lato

#define END_GAME	360				// Fine gara 90 secondi


// COPIARE TUTTI I DEFINES SU GRANDE_AUTOMATION PER I SERVO


// ASCENSORE
#define LIFT_LOAD		-174
#define LIFT_LOAD_MID	LIFT_LOAD+21

#define LIFT_BALL		-103
#define LIFT_BALL_BAF	LIFT_BALL-35
#define LIFT_BALL_MID	LIFT_BALL-12

#define LIFT_RUN		-155
#define LIFT_CASA		LIFT_BALL+30

#define LIFT_BASC_GIU	-170
#define LIFT_BASC_SU	LIFT_BASC_GIU+40


// ABBASSA BASCULA
#define ABB_BASC		1

#define ABB_ALTA		370
#define ABB_BASSA		1320


// PARETIE
#define PAR_SX			5
#define PAR_DX			9

#define PAR_DX_OPEN 	670
#define PAR_DX_CLOSED 	1330

#define PAR_SX_OPEN 	1060
#define PAR_SX_CLOSED 	450

#define PAR_PALLE_1 	860


// BAFFO
#define BAF_SX 		0
#define BAF_DX		4
#define UPD_SX		2
#define UPD_DX		7

#define BAF_SX_OPEN	1190
#define BAF_DX_OPEN	460

#define BAF_SX_OPEN_Y	1010
#define BAF_DX_OPEN_Y	590

#define BAF_SX_OPEN_B	1000
#define BAF_DX_OPEN_B	630

#define BAF_SX_CLOSED	470
#define BAF_DX_CLOSED 	1130

#define BAF_SX_MAX	1290
#define BAF_DX_MAX	330

#define BAF_SX_UPD	1000
#define BAF_DX_UPD	620

#define UPD_SX_UP	1150
#define UPD_DX_UP	390

#define UPD_SX_DOWN	630
#define UPD_DX_DOWN	910

#define UPD_SX_MID	950
#define UPD_DX_MID	590

#define BAF_SX_OFF	0
#define BAF_DX_OFF	0

#define UPD_SX_OFF	0
#define UPD_DX_OFF	0


// SPAZZOLA SCARICA
#define SPA 8

#define SPA_CEN 	790

#define SPA_YEL 	1160

#define SPA_BLU 	460

#endif
