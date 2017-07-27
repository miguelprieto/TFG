/*
 * main.c
 */

#include "defines.h"

#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <stdio.h>

#include "clocking.h"
#include "qei.h"
#include "pwm_control.h"
#include "gpio.h"
#include "timers.h"
#include "odometry.h"
#include "controller.h"
#include "velocity.h"
#include "bus_interface.h"
#include "path.h"
#include "simulator_interface.h"

/* variabili comunicazione con embedded */
	
int path_done;


/* varibili globali */

velocity 		vel;

/* variabili per il controllo in velocita' */

controller	 	pi_velocity_r, pi_velocity_l;
flags 			flagsR, flagsL;

/* variabili controllo in posizione */

int 			state;		//stato macchina a stati

robot 			robot_pos;

/* variabili di temporizzazione dei controlli */

int vel_check_counter;				//contatore per parametrizzare il tempo di controllo in velocita'
int pos_check_counter;				//contatore per parametrizzare il tempo di controllo in posizione
int upd_check_counter;				//contatore per parametrizzare il tempo di update della cinematica

/*	funzioni  */

#ifdef BOARD_VERSION_2
_FPOR(PWMPIN_ON & FPWRT_PWR1 & ALTI2C_OFF);
_FICD(ICS_PGD2 & JTAGEN_OFF);
_FWDT(FWDTEN_OFF);
#else
_FPOR(PWMPIN_ON & FPWRT_PWR1 & ALTI2C_OFF);
_FICD(ICS_PGD3 & JTAGEN_OFF);
_FWDT(FWDTEN_OFF);
#endif

void initialize_peripherals(void)
{
    init_clock();

    IEC0 = 0;				// DISABLE ALL USER INTERRUPT
    IEC1 = 0;				
    IEC2 = 0;
    IEC3 = 0;	
    IEC4 = 0;	

    RCONbits.SWDTEN   = 0;	// disable Watchdog Timer

    init_gpio();
    init_bus_objects();
    init_pwm();
    init_timer2(); //init_timer2 per la temporizzazione controllo in posizione e velocita'
    init_qei();
}



int main(void)
{		
    int i;

    vel_check_counter = 0;
    pos_check_counter = 0;			
    upd_check_counter = 0;			
	
    /* inizializzazione variabili di comunicazione */
	
    robot_pos.path_done 		  = 0;
    robot_pos.motor_locked 	  = 0;
    robot_pos.motor_locked_count = 0;
    robot_pos.position_valid = 0;
	
    /* inizializzazione periferiche e protocollo di comunicazione */
	
    initialize_peripherals();

    /* led flashing to signal a reboot */
    for (i = 0;i < 5;i++) {
        __delay_ms(100);
        led_on();
        __delay_ms(100);
        led_off();
    }

    /* inizializzazione delle strutture di controllo per i loop di velocita' */
    init_velocity_controllers();
    init_path();
    init_position();

#if defined(ROBOT_GRANDE)
# define LEFT_WHEEL_RADIUS	20.66
# define RIGHT_WHEEL_RADIUS	20.66
# define W_DISTANCE		327
    set_robot_geometry(LEFT_WHEEL_RADIUS, RIGHT_WHEEL_RADIUS, W_DISTANCE, TIC_GIRO);

    /* pi_velocity_l.Kp = 38; */
    /* pi_velocity_l.Ki = 70; */
    /* pi_velocity_l.Kd = 0.025; */

    /* pi_velocity_r.Kp = 38; */
    /* pi_velocity_r.Ki = 70; */
    /* pi_velocity_r.Kd = 0.025; */

    pi_velocity_l.Kp = 18;
    pi_velocity_l.Ki = 82;
    pi_velocity_l.Kd = 0;

    pi_velocity_r.Kp = 18;
    pi_velocity_r.Ki = 82;
    pi_velocity_r.Kd = 0;

    set_distance_speed(1300, 2000, 1300, 0);
    set_heading_speed(1000, 2000, 1500, 0.0);
    set_line_control_parameters(400, 0.6, 500);
#elif defined(ROBOT_PICCOLO)
# define LEFT_WHEEL_RADIUS	-20.66
# define RIGHT_WHEEL_RADIUS	-20.66
# define W_DISTANCE		135
    set_robot_geometry(LEFT_WHEEL_RADIUS, RIGHT_WHEEL_RADIUS, W_DISTANCE, TIC_GIRO);

    pi_velocity_l.Kp = 3;
    pi_velocity_l.Ki = 50;
    pi_velocity_l.Kd = 0;

    pi_velocity_r.Kp = 3;
    pi_velocity_r.Ki = 50;
    pi_velocity_r.Kd = 0;

    set_distance_speed(300, 300, 300, 0);
    set_heading_speed(300, 300, 300, 0.0);
    set_line_control_parameters(400, 0.6, 500);
#endif

    TRISBbits.TRISB11 = 0; //

    while (TRUE) {

        check_can_objects();

        if (IFS0bits.T2IF == 1) {

            IFS0bits.T2IF = 0;		//reset bit di flag del timer2

            LATBbits.LATB11 = !LATBbits.LATB11;

            vel_check_counter++;	// conto intervalli di tempo per il controllo in velocita'
            pos_check_counter++;	// conto intervalli di tempo per il controllo in posizione
            upd_check_counter++;	// conto intervalli di tempo per l'update della cinematica
        	
            /* update cinematica */
        	
            if (upd_check_counter == U_CHECK_TIME) {
                upd_check_counter=0;
                read_velocity();
                kinematics_update();
            }

            /* controllo in posizione */
			
            if (pos_check_counter == P_CHECK_TIME) {
                pos_check_counter = 0;

                position_control();
                check_motor_locked();
                path_control();

                send_can_objects(); // Invio aggiornamento posizione

            } else if (pos_check_counter == 1) {
                // Questa azione starebbe concettualmente meglio in upd_check_counter == U_CHECK_TIME.
                // Tuttavia, dato che U_CHECK_TIME < P_CHECK_TIME e che P_CHECK_TIME è multiplo di U_CHECK_TIME,
                // per evitare di chiamare send_velocity_data() troppo spesso e anche di mandare due
                // messaggi per volta (con il rischio che se ne perda qualcuno), eseguiamo send_velocity_data()
                // all'iterazione immediatamente successiva a quella di send_can_objects()
                send_velocity_data(); // Invio aggiornamento velocità
            }
            else if (pos_check_counter == 2) {
                send_telemetry_data(); // send telemetry data
            }        	
        	
            /* controllo in velocita' */
        	
            if (vel_check_counter == V_CHECK_TIME) {
                vel_check_counter=0;

                velocity_control();

            }

        }//end if timer

        simulator_relax();
    }//end while(TRUE)

}//end main
