/*
 * defines.h
 */

#ifndef __DEFINES_H
#define __DEFINES_H

#define PORT_OUT  0
#define PORT_IN   1

#define LED_ON    1
#define LED_OFF   0

#define TRUE 	  1

//#define EXTERNAL_CLOCK
#define INTERNAL_CLOCK

#if defined(EXTERNAL_CLOCK) && defined(INTERNAL_CLOCK)
#error Both EXTERNAL_CLOCK and INTERNAL_CLOCK defined
#endif

#if !defined(EXTERNAL_CLOCK) && !defined(INTERNAL_CLOCK)
#error None of EXTERNAL_CLOCK and INTERNAL_CLOCK defined
#endif


#ifdef EXTERNAL_CLOCK
#define FCY   (77414400/2)
#endif



#ifdef INTERNAL_CLOCK
#define FCY   (77385000/2)
#endif


#define PI       3.14159265358979
#define TWO_PI   6.28318530717959
#define HALF_PI  1.570796326794895
#define DEGREES_60 1.0471975511965976
#define DEGREES_75 1.309

#define LINE_TO_POINT_HDG_THRESHOLD   DEGREES_75

#define TO_DEGREES(a)  ((a) * (180.0/PI))
#define TO_RADIANS(a)  ((a) * (PI/180.0))

#define BOARD_VERSION_2

#endif

