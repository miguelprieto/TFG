/*
 * pwm_control.h
 */

#ifndef __PWM_CONTROL
#define __PWM_CONTROL

void init_pwm(void);
void set_pwm(int configS, int configD);
void break_on();
void break_off();

#define MAX_PWM 4096
#define LEFT_DIR    P1OVDCONbits.POUT1L
#define RIGHT_DIR   P1OVDCONbits.POUT2L


#endif
