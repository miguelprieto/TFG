/*
 * controller.c
 */

#include "controller.h"

void pi_evaluation(controller *parametri)
{
    float u_p = 0;

    u_p  = parametri->Kp * parametri->error;

    if(!parametri->windup)
        parametri->u_i += parametri->Ki* (((parametri->error + parametri->error_old) / 2.0) * parametri->dt);

    parametri->out = u_p + parametri->u_i ;

    if(parametri->out >= parametri->saturation){
        parametri->out = parametri->saturation;
        parametri->windup = 1;
    }
    else if(parametri->out <= (-1.0 * parametri->saturation)){
        parametri->out = -1.0 * parametri->saturation;
        parametri->windup = 1;
    }
    else
        parametri->windup = 0;

    parametri->error_old = parametri->error;
}

void pid_reset(controller * c)
{
    c->u_i = 0;
    c->error_old = 0;
    c->old_process_var = 0;
}

void pid_init(controller * c, float kp, float ki, float kd, float dt, char has_saturation, float saturation)
{
    c->u_i = 0;
    c->windup = 0;
    c->error_old = 0;
    c->old_process_var = 0;
    c->Kp = kp;
    c->Ki = ki;
    c->Kd = kd;
    c->dt = dt;
    c->has_saturation = has_saturation;
    c->saturation = saturation;
}

inline void pid_set_process_var(controller * c, float process_var)
{
    c->old_process_var = process_var;
}

float pid_evaluation(controller * c, float target, float process_var)
{
    return pid_evaluation_error(c, target - process_var, process_var);
}

float pid_evaluation_error(controller * c, float err, float process_var)
{
    float u_p, u_d;

    c->error = err;

    u_p  = c->Kp * c->error;
    u_d  = - c->Kd * (process_var - c->old_process_var) / c->dt;

    c->old_process_var = process_var;

    if(!c->windup || !c->has_saturation)
        c->u_i += c->Ki * (((c->error + c->error_old) / 2.0) * c->dt);

    c->out = u_p + c->u_i + u_d;

    if (c->has_saturation) {
        if(c->out >= c->saturation) {
            c->out = c->saturation;
            c->windup = 1;
        }
        else if(c->out <= (-1.0 * c->saturation)) {
            c->out = -1.0 * c->saturation;
            c->windup = 1;
        }
        else
            c->windup = 0;
    }

    c->error_old = c->error;
    return c->out;
}
