/*
 * timers.h
 */

#ifndef __TIMERS_H
#define __TIMERS_H


void timer2_init(void);
void timer2_start(int usec_val);
bool timer2_elapsed(void);
void timer4_init(void);
bool timer4_elapsed(void);

#endif
