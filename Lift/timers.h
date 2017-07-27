/*
 * timers.h
 */

#ifndef __TIMERS_H
#define __TIMERS_H


#define PERTMR2			0.005		//periodo timer2 (in secondi) (solo indicativo, modificare timers.c anche)

void init_timer2(void);

#define P_CHECK_TIME		8							//parametro a moltiplicare il PERTMR2 per parametrizzare la temporizzazione del controllo in posizione
#define U_CHECK_TIME		2							//parametro a moltiplicare il PERTMR2 per parametrizzare la temporizzazione dell'update della cinematica del robot
#define V_CHECK_TIME		2							//parametro a moltiplicare il PERTMR2 per pa

//U_CHECK_TIME deve essere uguale a V_CHECK_TIME !
#define P_TIMER_INTERVAL	(PERTMR2*P_CHECK_TIME) 		//tempo controllo in posizione	
#define U_TIMER_INTERVAL	(PERTMR2*U_CHECK_TIME)		//tempo update cinematica rametrizzare la temporizzazione del controllo in velocita'
#define V_TIMER_INTERVAL	(PERTMR2*V_CHECK_TIME) 		//tempo controllo in velocita'


#endif
