/*
 * odometry.c
 */


#include "defines.h"
#include "odometry.h"

/* variabili necessarie per il calcolo della posizione */
				


void kinematics_update(void)
{
    float dx = 0.0;
    float dy = 0.0;
	
    /* aggiornamento struttura robot */
	
    robot_pos.delta_R  = robot_pos.delta_tic_R * robot_pos.wheel_factor_r; 	// spostamento lineare ruota destra   [mm]
    robot_pos.delta_L  = robot_pos.delta_tic_L * robot_pos.wheel_factor_l; 	// spostamento lineare ruota sinistra [mm]
	
    robot_pos.cross_coupling_L += robot_pos.delta_L;
    // spostamento lineare ruota destra   [mm]
    // (viene cumulato per un numero di volte pari a position_time per il calcolo del segnale errore del Cross_Coupling)  	

    robot_pos.cross_coupling_R += robot_pos.delta_R;
    // spostamento lineare ruota sinistra [mm]
    // (viene cumulato per un numero di volte pari a position_time per il calcolo del segnale errore del Cross_Coupling)
	
    robot_pos.delta_linear  = (robot_pos.delta_R + robot_pos.delta_L) / 2 ;
    //  spostamento  lineare interasse    [mm]	
    robot_pos.delta_angular = (robot_pos.delta_R - robot_pos.delta_L) / robot_pos.wheel_distance;
    //  variazione orientamento interasse [rad]

    robot_pos.linear  += robot_pos.delta_linear  ; //  spostamento lineare totale interasse [mm] durante ogni singolo comando
    robot_pos.angular += robot_pos.delta_angular ; //  variazione totale orientamento interasse [rad] durante ogni singolo comando


    dx = (robot_pos.delta_linear * cosf( robot_pos.theta + (robot_pos.delta_angular/2.0)));
    //approssimazione di secondo ordine dell'odometria
    dy = (robot_pos.delta_linear * sinf( robot_pos.theta + (robot_pos.delta_angular/2.0)));
    //approssimazione di secondo ordine dell'odometria
	
    robot_pos.x = robot_pos.x + dx +  robot_pos.K_x * robot_pos.delta_angular * dy ; //compensazione della deriva con Kx
    robot_pos.y = robot_pos.y + dy -  robot_pos.K_y * robot_pos.delta_angular * dx ; //compensazione della deriva con ky

    robot_pos.theta = robot_pos.theta + robot_pos.delta_angular ;

    if ( robot_pos.theta > PI ) robot_pos.theta -= TWO_PI;
    if ( robot_pos.theta < -PI) robot_pos.theta += TWO_PI;

    if (vel.read_L == vel.read_R)
        robot_pos.rotation_radius_inf = 1;
    else {
        robot_pos.rotation_radius_inf = 0;
        robot_pos.rotation_radius = (robot_pos.wheel_distance / 2.0) * (vel.read_R + vel.read_L) / (vel.read_R - vel.read_L);
    }
}
