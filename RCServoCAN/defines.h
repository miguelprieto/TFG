/*
 * defines.h
 */

#ifndef __DEFINES_H
#define __DEFINES_H

#define PORT_OUT  0
#define PORT_IN   1

#define USE_ECAN

//#define EXTERNAL_CLOCK
#define INTERNAL_CLOCK

#if defined(EXTERNAL_CLOCK) && defined(INTERNAL_CLOCK)
#error Both EXTERNAL_CLOCK and INTERNAL_CLOCK defined
#endif

#if !defined(EXTERNAL_CLOCK) && !defined(INTERNAL_CLOCK)
#error None of EXTERNAL_CLOCK and INTERNAL_CLOCK defined
#endif


#ifdef EXTERNAL_CLOCK
#define FCY   (77414400 / 2)
#endif



#ifdef INTERNAL_CLOCK
#define FCY   (77385000 / 2)
#endif

typedef unsigned char uint8_t;


#endif
