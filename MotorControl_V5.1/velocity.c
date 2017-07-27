/*
 * velocity.c
 */

#include "velocity.h"
#include "odometry.h"
#include "gpio.h"

#include <p33FJ128MC802.h>
#include <libpic30.h>


void init_velocity_controllers(void)
{
    /*inizializzo le strutture di controllo del loop velocita' destra */

    pid_init(&pi_velocity_r, 0, 0, 0, V_TIMER_INTERVAL, 1, SATURATION);
	
    flagsR.pid_on 	= 0;
    flagsR.motor_en 	= 0;
    flagsR.frenata 	= 0;
	
    /*inizializzo strutture di controllo del loop velocita' sinistra */
	
    pid_init(&pi_velocity_l, 0, 0, 0, V_TIMER_INTERVAL, 1, SATURATION);

    flagsL.pid_on 	= 0;
    flagsL.motor_en 	= 0;
    flagsL.frenata 	= 0;
	
    /* inizializzo le strutture di informazioni sulla velocità */
	
    vel.right	= 0.0;
    vel.left 	= 0.0;
    vel.read_R = 0.0;
    vel.read_L = 0.0;

    vel.mean_smooting     = 0.0;
    vel.mean_smooting_old = 0.0;


    vel.threshold        = 0.0;
    vel.threshold_vel    = 0.0;

    vel.read_mean_linear    = 0.0;
    vel.read_mean_angular   = 0.0;
    vel.mean                = 0.0;	

    vel.pendenza_nominale   = 0.0;	
    vel.percentuale         = 0.0;                 							
    vel.pendenza            = 0.0;      	    	
    vel.pendenza_dt         = 0.0;
}


void reset_velocity_controllers(void)
{
    /*inizializzo le strutture di controllo del loop velocita' destra */
	
    pi_velocity_r.error 	= 0.0;
    pi_velocity_r.error_old 	= 0.0;
    pi_velocity_r.windup 	= 0;
    pi_velocity_r.u_i 		= 0.0;
    pi_velocity_r.out 		= 0.0;
	
    flagsR.pid_on	 = 0;
    flagsR.motor_en      = 0;
    flagsR.frenata	 = 0;
	
    /*inizializzo strutture di controllo del loop velocita' sinistra */
	
    pi_velocity_l.error 	= 0.0;
    pi_velocity_l.error_old 	= 0.0;
    pi_velocity_l.windup 	= 0;
    pi_velocity_l.u_i 		= 0.0;
    pi_velocity_l.out 		= 0.0;
	
    flagsL.pid_on 	= 0;
    flagsL.motor_en 	= 0;
    flagsL.frenata 	= 0;
	
    /* inizializzo le strutture di informazioni sulla velocità */

    vel.read_R = 0.0;
    vel.read_L = 0.0;	

    vel.mean_smooting      = 0.0;
    vel.mean_smooting_old  = 0.0;

    vel.threshold        = 0.0;
    vel.threshold_vel    = 0.0;

    vel.read_mean_linear   = 0.0;
    vel.read_mean_angular  = 0.0;							

    set_pwm(0 , 0);
	
    flagsR.pid_on   = 1;
    flagsR.motor_en = 1;
	
    flagsL.pid_on   = 1;
    flagsL.motor_en = 1;

}


void read_velocity(void)
{
    float vel_R;    	
    float vel_L;
    int poscount_R;
    int poscount_L;

    poscount_L = POS1CNT;
    poscount_R = POS2CNT;

    robot_pos.delta_tic_R = poscount_R - robot_pos.tic_R;
    vel_R = (robot_pos.delta_tic_R ) / U_TIMER_INTERVAL; // ticks per seconds
    /* vel_R = vel_R / TIC_GIRO; // rev per seconds */
    /* vel_R = vel_R * 60.0; // rev per minute */
    vel_R = vel_R * robot_pos.wheel_factor_r;  // mm per seconds
    robot_pos.tic_R = poscount_R;

    robot_pos.delta_tic_L = robot_pos.tic_L - poscount_L;
    vel_L = (robot_pos.delta_tic_L) / U_TIMER_INTERVAL; // ticks per seconds
    /* vel_L = vel_L / TIC_GIRO; // rev per seconds */
    /* vel_L = vel_L * 60.0; // rev per minute */
    vel_L = vel_L * robot_pos.wheel_factor_l;  // mm per seconds
    robot_pos.tic_L = poscount_L;

    vel.read_L = vel_L;
    vel.read_R = vel_R;

    vel.read_mean_linear  = ( vel.read_R + vel.read_L ) / 2.0 ;
    // velocità del punto medio nel caso di moto lineare in mm/s

    vel.read_mean_angular = ( vel.read_R - vel.read_L ) / robot_pos.wheel_distance ;
    // velocità angolare in rad/s
}

#define LOCKING_SPEED_THRESHOLD  10.0 // 1cm/s
#define PWM_LOCKING_THRESHOLD    3072.0

bool check_locking_condition(controller * c, float expected_speed, float measured_speed)
{
    return (fabs(expected_speed) > 0 && fabs(measured_speed) < LOCKING_SPEED_THRESHOLD && fabs(c->out) > PWM_LOCKING_THRESHOLD);
}

bool is_left_wheel_locked(void)
{
    if (flagsL.pid_on == 1 &&  flagsL.motor_en == 1)
        return check_locking_condition(&pi_velocity_l, vel.left, vel.read_L);
    else
        return false;
}

bool is_right_wheel_locked(void)
{
    if (flagsR.pid_on == 1 &&  flagsR.motor_en == 1)
        return check_locking_condition(&pi_velocity_r, vel.right, vel.read_R);
    else
        return false;
}

	/* filtro di smooting */
	
void smooting(velocity *vel, short segno)
{
    float v_in 		= (vel->mean)*segno;
    float v_diff 	= v_in - (vel->mean_smooting_old);
		
    if (fabs(v_diff) < vel->pendenza_dt)		//la pendenza del riferimento è minore di quella massima impostata
        vel->mean_smooting = v_in;		//mettiamo in uscita il riferimento stesso
    else if (v_diff > 0)
        vel->mean_smooting = vel->mean_smooting_old + vel->pendenza_dt;
    else
        vel->mean_smooting = vel->mean_smooting_old - vel->pendenza_dt;
			 		
    //NB: calcoleremo le velocita' destra e sinistra da inviare al modulo pwm basandoci sempre su vel->mean_smooting
    vel->mean_smooting_old = vel->mean_smooting;

}



void velocity_control(void)
{

    if (flagsL.pid_on == 0 && flagsR.pid_on == 0)
        return;

    /* eseguo il controllo in velocita' della ruota destra */
    if (flagsR.motor_en && flagsR.pid_on) {
        pid_evaluation(&pi_velocity_r, vel.right, vel.read_R);
    }

    /* eseguo il controllo in velocità della ruota sinistra */	
    if (flagsL.motor_en && flagsL.pid_on) {
        pid_evaluation(&pi_velocity_l, vel.left, vel.read_L);
    }

    if (flagsR.motor_en && flagsL.motor_en)
        set_pwm(pi_velocity_l.out , pi_velocity_r.out);
    else {
        set_pwm(0 , 0);

        pi_velocity_l.error = 0.0;
        pi_velocity_l.error_old = 0.0;
        pi_velocity_l.windup = 0;
        pi_velocity_l.u_i = 0.0;
        pi_velocity_l.old_process_var = 0.0;

        pi_velocity_r.error = 0.0;
        pi_velocity_r.error_old = 0.0;
        pi_velocity_r.windup = 0;
        pi_velocity_r.u_i = 0.0;
        pi_velocity_r.old_process_var = 0.0;
    }
}

