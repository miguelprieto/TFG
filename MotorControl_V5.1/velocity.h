/*
 * velocity.h
 */


#ifndef __VELOCITY_H
#define __VELOCITY_H

#include <stdbool.h>
#include "pwm_control.h"
#include "controller.h"
#include "timers.h"
#include "math.h"

#define SATURATION 			MAX_PWM*0.96				//soglia di saturazione per il pwm
#define TO_RPM 				9.55						//fattore per passare da rad/s a rpm
#define V_CHECK_TIME		2							//parametro a moltiplicare il PERTMR2 per parametrizzare la temporizzazione del controllo in velocita'
#define V_TIMER_INTERVAL	(PERTMR2*V_CHECK_TIME) 		//tempo controllo in velocita'
#define VELOCITY_THRESHOLD  4


	/*strutture di controllo per il loop in velocita' */

typedef struct {
    short pid_on;
    short motor_en;
    short frenata;
} flags;                                //flags per il controllo della velocità destra e sinistra

typedef struct {
	/* TUTTE LE VELOCITA' SONO IN RPM */

    float right, left;		//velocita' riferimento per ruota destra e sinistra
    float read_R, read_L; 		//velocita' lette dagli encoder ruota destra e sinistra
    float mean; 			//velocita' del punto medio desiderata, nel caso di rotazioni su se stessi è una velocita' angolare
    float read_mean_linear;		//velocita' del punto medio letta (in rpm comunque), linear nel senso che si muove in linea retta
    float read_mean_angular;	//velocita' angolare del punto medio letta


    /* parametri necessari per il filtro di smooting */

    float mean_smooting; 	//velocita' che useremo per evaluate_velocity
    float mean_smooting_old;
    float pendenza_dt;		//pendenza*dt(dt=intervallo tra una chiamata e l'altra della funzione di smooting)	

    /* soglie per il controllo in posizione */

    float threshold;
    float threshold_vel;

    /* ulteriori parametri */

    float pendenza;			//derivata della velocita', nel caso di rotazioni su se stessi è un'accelerazione angolare
    float pendenza_nominale;
    float percentuale;		//è una percentuale utilizzata per ridurre la pendenza di frenata rispetto a quella di accelerazione

} velocity;
//tutte le informazioni sulle velocità, leggere i parametri come "nomeparametro"_velocity


#define sgn(v) ((v >= 0) ? 1.0 : -1.0)

#include "position.h"

/*variabili presenti nel main*/

extern flags 			flagsR, flagsL;
extern controller	 	pi_velocity_r, pi_velocity_l;
extern velocity 		vel;

/* funzioni */

void init_velocity_controllers(void);
void read_velocity(void);
void velocity_control(void);
void reset_velocity_controllers(void);

bool is_left_wheel_locked(void);
bool is_right_wheel_locked(void);


#endif
