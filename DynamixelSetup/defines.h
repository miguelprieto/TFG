/*
 * defines.h
 */

#ifndef __DEFINES_H
#define __DEFINES_H

#define PORT_OUT  0
#define PORT_IN   1

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

#define TO_DEGREES(a)  ((a) * (180.0/PI))
#define TO_RADIANS(a)  ((a) * (PI/180.0))

//#define DO_FUSION

//#define USE_QUATERNIONS

//#define USE_ZIGBEE

//#define HEIGHT_CONTROL_ON

//#define USE_DIRECT_ATTITUDE_CONTROL

#define PWM_RATE_MS             2.5
//#define RATE_CONTROL_COUNT      1    // 2.5ms
//#define ATTITUDE_CONTROL_COUNT  20   // 50ms
//#define HEIGHT_CONTROL_COUNT    40   // 100ms

#define BOARD_VERSION_2


#endif

