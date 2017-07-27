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
#include "controller.h"
#include "bus_interface.h"
#include "position.h"
#include "velocity.h"


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
#ifdef USE_CAN
    init_bus_objects();
#else
    init_uart(1, B115200);
#endif
    init_pwm();
    init_timer2(); //init_timer2 per la temporizzazione controllo in posizione e velocita'
    init_qei();

    init_position();
    init_velocity_controllers();
}



int main(void)
{		
    int i;

    vel_check_counter = 0;
    pos_check_counter = 0;			
    upd_check_counter = 0;			
	
    /* inizializzazione variabili di comunicazione */
	
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

    while (TRUE) {

        check_can_objects();

        if (IFS0bits.T2IF == 1) {

            IFS0bits.T2IF = 0;		//reset bit di flag del timer2

            upd_check_counter++;	//conto intervalli di tempo per l'update della cinematica
            pos_check_counter++;
            vel_check_counter++;
        	        	
            /* update cinematica */
        	
            if (upd_check_counter == U_CHECK_TIME) {
                upd_check_counter=0;
                read_velocity();
            }
        	        	
            if (vel_check_counter == V_CHECK_TIME) {
                vel_check_counter=0;
                velocity_control();
            }
        	        	
            if (pos_check_counter == P_CHECK_TIME) {
                pos_check_counter = 0;
                send_can_objects();
                machine(0);
                machine(1);
            }
        }//end if timer

    }//end while(TRUE)

}//end main
