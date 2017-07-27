/*
 * odometry.h
 */

#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "velocity.h"
#include "position.h"

#define MOTO_RIDUTTORE		1
//impulsi encoder in un giro, controllare modalita' funzionamento encoder qei.c
#define TIC_GIRO			4000



void kinematics_update(void);

#endif

