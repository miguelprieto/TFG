/*
 * controller.h
 */


#ifndef __CONTROLLER_H
#define __CONTROLLER_H

/* controllore pid per il loop in velocita' e posizione */

typedef struct {
    float Ki;
    float Kp;
    float Kd;
    short windup;
    float u_i;
    float error_old;
    float error;
    float out;
    float old_process_var;
    float dt;	
    float saturation;
    char has_saturation;
} controller;

	/* funzioni di controllo */

void pi_evaluation(controller *parametri);
void pid_init(controller * c, float kp, float ki, float kd, float dt, char has_saturation, float saturation);
void pid_set_process_var(controller * c, float process_var);
float pid_evaluation_error(controller * c, float err, float process_var);
float pid_evaluation(controller * c, float target, float process_var);
void pid_reset(controller * c);

#endif
