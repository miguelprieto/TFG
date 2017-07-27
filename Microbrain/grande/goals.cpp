/*
 *		goals.c
 */

#include "defines.h"

#include <xc.h>
#include <stdio.h>
#include <math.h>
#include <libpic30++.h>

#include "bus_interface.h"
#include "color.h"
#include "controller_presets.h"
#include "console.h"
#include "geometry.h"
#include "goals.h"
#include "goal_manager.h"
#include "gpio.h"
#include "routing.h"
#include "servos.h"
#include "wait.h"

static GoalStart goal_start("start");
static GoalCratere1 goal_cratere_1("cratere1");
static GoalCratere2 goal_cratere_2("cratere2");
static GoalMegaCratere goal_megacratere("megacratere");
static GoalScarica goal_scarica("scarica");


// Controlla se ci troviamo nella situazione prevista
static bool checkExpectedPosition(double x, double y)
{
	int acc = 45;
	double robot_t;
	Point pos = get_pos(&robot_t);

	if (fabs(pos.x - x) < acc && fabs(pos.y - y) < acc)
		return true;
	else
		return false;
}

static bool checkExpectedPositionwithAngle(double x, double y, double th)
{
	double robot_t;
	Point pos = get_pos(&robot_t);

	if (fabs(pos.x - x) < 60 && fabs(pos.y - y) < 60 && fabs(ANG_DIFF(th, robot_t)) < 10)
		return true;
	else
		return false;
}


void servo_run(void)
{
	__delay_ms(100);
	set_servo_position (UPD_SX, (UPD_SX_DOWN));
	set_servo_position (UPD_DX, (UPD_DX_DOWN));
	__delay_ms(100);
	set_servo_position (BAF_SX, (BAF_SX_UPD));
	set_servo_position (BAF_DX, (BAF_DX_UPD));
	__delay_ms(100);
	set_servo_position (UPD_SX, (UPD_SX_UP));
	set_servo_position (UPD_DX, (UPD_DX_UP));
}

void chiusura(int n, int controllo)
{
	int i;

	set_servo_position (UPD_DX, (UPD_DX_DOWN));
	set_servo_position (UPD_SX, (UPD_SX_DOWN));
	__delay_ms(140);

	// CHIUSURA BAFFI
	set_servo_position(BAF_DX, BAF_DX_CLOSED-250);	// DESTRO
	set_servo_position(BAF_SX, BAF_SX_CLOSED+250);	// SINISTRO
	__delay_ms(400);

	set_servo_position(BAF_SX, BAF_SX_CLOSED+340);	// SINISTRO
	__delay_ms(150);

	set_servo_position(BAF_DX, BAF_DX_CLOSED-170);	// DESTRO
	__delay_ms(150);

	set_servo_position(BAF_DX, BAF_DX_CLOSED-290);	// DESTRO
	set_servo_position(BAF_SX, BAF_SX_CLOSED+140);	// SINITRO
	__delay_ms(150);

	set_servo_position(BAF_SX, BAF_SX_CLOSED+270);	// DESTRO
	__delay_ms(100);

	// CICLO FOR - CHIUSURA PARZIALE
	for (i=0; i<=n; i++) {
		set_servo_position(BAF_DX, BAF_DX_CLOSED);	// DESTRO
		__delay_ms(70);
		set_servo_position(BAF_SX, BAF_SX_CLOSED);	// SINITRO
		__delay_ms(200);

		set_servo_position(BAF_SX, BAF_SX_CLOSED+270);	// SINITRO
		set_servo_position(BAF_DX, BAF_DX_CLOSED-210);	// DESTRO
		__delay_ms(100);
	}

	// CONSERVAZIONE PALLE (IN CASO DI TANTE PALLE)
	if (controllo == 1) {
		set_servo_position(PAR_SX, PAR_SX_OPEN-50); // SINISTRA
		__delay_ms(30);
		set_servo_position(PAR_DX, PAR_DX_OPEN+50); // DESTRA
		__delay_ms(50);

		for (i=0; i<=5; i++) {
			set_servo_position(BAF_DX, BAF_DX_CLOSED+100);	// DESTRO
			__delay_ms(50);
			set_servo_position(BAF_SX, BAF_SX_CLOSED-100);	// SINITRO
			__delay_ms(100);

			set_servo_position(PAR_SX, PAR_SX_OPEN-50); // SINISTRA
			__delay_ms(30);
			set_servo_position(PAR_DX, PAR_DX_OPEN+50); // DESTRA
			__delay_ms(100);
		}
	}

	// CHIUSURA TOTALE
	set_servo_position(BAF_DX, BAF_DX_CLOSED);	// DESTRO
	__delay_ms(50);
	set_servo_position(BAF_SX, BAF_SX_CLOSED);	// SINITRO
}

void palle(void)
{
    // APERTURA
    set_servo_position(PAR_SX, PAR_SX_OPEN); // SINISTRA
    set_servo_position(PAR_DX, PAR_DX_OPEN); // DESTRA

    __delay_ms(300);

    lift_go_to(LIFT_BALL_BAF);

    set_servo_position(BAF_DX, BAF_DX_CLOSED+180);
    __delay_ms(80);
    set_servo_position(BAF_SX, BAF_SX_CLOSED-180);

    lift_go_to(LIFT_BALL_MID);

    set_servo_position(BAF_SX, BAF_SX_CLOSED); // SINISTRA
    set_servo_position(BAF_DX, BAF_DX_CLOSED); // DESTRA
    __delay_ms(50);

    set_servo_position(UPD_SX, UPD_SX_DOWN); // SINISTRA
    set_servo_position(UPD_DX, UPD_DX_DOWN); // DESTRA
    __delay_ms(50);

    // CHIUSURA
    set_servo_position(PAR_SX, PAR_PALLE_1); // SINISTRA
    __delay_ms(200);

    lift_go_to(LIFT_BALL);
    __delay_ms(200);

    set_servo_position(PAR_DX, PAR_DX_CLOSED);  // DESTRA
    __delay_ms(60);
    set_servo_position(PAR_SX, PAR_SX_CLOSED);  // DESTRA
    __delay_ms(100);

    send_lift_position(0, LIFT_RUN);
}

void palle_alternativo (void)
{
	set_servo_position(PAR_SX, PAR_SX_OPEN); // SINISTRA
	set_servo_position(PAR_DX, PAR_DX_OPEN); // DESTRA

	__delay_ms(300);

	lift_go_to(LIFT_BALL_BAF);

	set_servo_position(BAF_DX, BAF_DX_CLOSED+150);
	__delay_ms(80);
	set_servo_position(BAF_SX, BAF_SX_CLOSED-150);

	lift_go_to(LIFT_CASA);
	set_servo_position(PAR_DX, PAR_DX_CLOSED);  // DESTRA
	__delay_ms(140);

	set_servo_position(PAR_SX, PAR_SX_CLOSED); // SINISTRA
	__delay_ms(100);
}

void scarica(void)
{
	int i, j;
	int tempo = 170;

	set_servo_position(UPD_DX, UPD_DX_DOWN);
	set_servo_position(UPD_SX, UPD_SX_DOWN);
	__delay_ms(100);

	set_servo_position(BAF_DX, BAF_DX_CLOSED+100);
	__delay_ms(60);
	set_servo_position(BAF_SX, BAF_SX_CLOSED-100);
	__delay_ms(100);

	set_servo_position(PAR_SX, PAR_SX_OPEN);
	__delay_ms(60);
	set_servo_position(PAR_DX, PAR_DX_OPEN);
	__delay_ms(250);

	printf("\nINIZIO SCARIMENTO PALLE - TEMPO: %.2f\n\n", (game_timer - start_time) * .25);

	// PRIMO PIANO
	lift_go_to(-30);
	__delay_ms(100);

	i = 120;
	for (j=0; j<=8; j++) {
		if(color==YELLOW)
			set_servo_position(SPA, SPA_CEN-i);
		else
			set_servo_position(SPA, SPA_CEN+i);
		__delay_ms(tempo);

		set_spazzola_start();
		__delay_ms(tempo);

	}

	// ULTIMO PIANO
	lift_go_to(0);
	__delay_ms(100);

	tempo = tempo+50;
	i = 400;
	for (j=0; j<=100; j++) {
		if(color==YELLOW)
			set_servo_position(SPA, SPA_CEN-i);
		else
			set_servo_position(SPA, SPA_CEN+i);
		__delay_ms(tempo);

		set_spazzola_start();
		__delay_ms(tempo);

	}

	printf("\nFINE SCARIMENTO PALLE - TEMPO: %.2f\n\n", (game_timer - start_time) * .25);
}


// USCITA DAL PRIMO CRATERE
void exit(void)
{
	servo_run();

	distance_default.apply();
	if (doPath(symmNodeifBlue(&Graph::CRATERE1_Y), true) == IMPOSSIBLE) {
		printf("\nDIREZIONE: RITORNO IMPOSSIBILE\n\n");
	}
	unchecked_wait(MotionEvent());
}

// USCITA DAL SECONDO CRATERE
void exit_secondo(void)
{
	servo_run();

	distance_default.apply();
	if (doPath(symmNodeifBlue(&Graph::CRATERI_Y), true) == IMPOSSIBLE) {
		printf("\nDIREZIONE: RITORNO IMPOSSIBILE\n\n");
	}
	unchecked_wait(MotionEvent());
}

// USCITA DAL MEGACRATERE
void exit_megacratere(void)
{
	servo_run();

	distance_default.apply();
	if (doPath(symmNodeifBlue(&Graph::CRATERI_Y), true) == IMPOSSIBLE) {
		printf("\nDIREZIONE: RITORNO IMPOSSIBILE\n\n");
	}
	unchecked_wait(MotionEvent());
}


void abbassa_bascula (void)
{
	set_servo_position(ABB_BASC, ABB_BASSA);
	__delay_ms(500);
	set_servo_position(ABB_BASC, 0);
	__delay_ms(1000);

	unchecked_wait(MotionEvent());
}


//------------------------------------------------------------
// INIZIO GOAL

GoalStart::GoalStart(const char *name)
: Goal(name)
{
}

int GoalStart::feasible()
{
	return 0; // Priorità massima
}

success_t GoalStart::execute()
{
	// DISATTIVAZIONE OBSTACLE_AVOIDANCE
  disable_obstacle_avoidance();

	GraphNode *point = symmNodeifBlue(&Graph::RITORNO_Y);

	Point inizio_path = point->pos;
	Point post_bascula (symmXifBlue(850), 0);
	Point iniziale;

	WaitResult bascula;

	printf("\nSTART\n\n");

	iniziale = get_pos(0);

	post_bascula.y = iniziale.y;

	distance_slow.apply();
	forward_to_distance(407);
	bascula = wait(MotionEvent());

	if (bascula == MOTOR_LOCKED) {
		do {
			distance_slow.apply();
			forward_to_point(iniziale);
			printf("\nBASCULA ANDATA IMPOSSIBILE\n\n");
			distance_default.apply();
			forward_to_distance(417);
			bascula = wait(MotionEvent());
		} while (bascula == MOTOR_LOCKED);
	}

	motion_stop();

	// TEMPO DI ATTESA SULLA BASCULA
	__delay_ms(1000);

	distance_default.apply();
	forward_to_point(post_bascula);

	heading_to(inizio_path, HEADING_FRONT);

	forward_to_point(inizio_path);
	unchecked_wait(MotionEvent());

	// ATTIVAZIONE OBSTACLE_AVOIDANCE
 	enable_obstacle_avoidance();

	return DONE;
}

// ------------------------------------------------------------

GoalCratere1::GoalCratere1(const char *name)
: Goal(name)
{
}

int GoalCratere1::feasible()
{
	return 1; // Priorità massima dopo Start
}

success_t GoalCratere1::execute()
{

	GraphNode *point = symmNodeifBlue(&Graph::CRATERE1_Y);

	Point cratere = point->pos;

	WaitResult controllo;

	printf("\nDIREZIONE: CRATERE 1\n\n");

	// PATH DI AVVICINAMENTO
	distance_default.apply();
	if (doPath(symmNodeifBlue(&Graph::CRATERE1_Y)) == IMPOSSIBLE) {
		printf("\nDIREZIONE: CRATERE 1 IMPOSSIBILE\n\n");
    	        resetRunningGoal();
		return FAIL;
	}
	wait(MotionEvent());

	// SI RADDRIZZA
	rotate_absolute(symmTifBlue(180));
	unchecked_wait(MotionEvent());

	// PREPARA L'ASCENSORE
	send_lift_position(0, LIFT_LOAD+4);

	// ABBASSA I BAFFI
	set_servo_position (UPD_SX, (UPD_SX_DOWN));
	set_servo_position (UPD_DX, (UPD_DX_DOWN));
	__delay_ms(100);

	// SISTEMA I BAFFI
	if(color == YELLOW) {
		set_servo_position(BAF_SX, BAF_SX_OPEN_Y);	//SINISTRO
		set_servo_position(BAF_DX, BAF_DX_OPEN_Y);	//DESTRO
	} else {
		set_servo_position(BAF_SX, BAF_SX_OPEN_B);	//SINISTRO
		set_servo_position(BAF_DX, BAF_DX_OPEN_B);	//DESTRO
	}

	// ASPETTA TUTTI I MOVIMENTI DEI SERVO
	unchecked_wait(MotionEvent());

	// AVVICINAMENTO AL CRATERE 1
	distance_slow.apply();
	forward_to_distance(100);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			printf("\nCRATERE1 IMPOSSIBILE PER: MOTOR LOCKED\n\n");
			exit();
			return FAIL;
			break;
		case PATH_DONE:
			break;
	}

	// SISTEMA L'ASCENSORE
	lift_go_to(LIFT_LOAD);
	
	// SI RADDRIZZA
	rotate_absolute(symmTifBlue(180));
	unchecked_wait(MotionEvent());

	// AVVICINAMENTO AL CRATERE 1
	distance_very_slow.apply();
	forward_to_distance(35);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			printf("\nCRATERE1 IMPOSSIBILE PER: MOTOR LOCKED\n\n");
			exit();
			return FAIL;
			break;
		case PATH_DONE:
			break;
	}

	// CHIUSURA
	chiusura(0, 0);

	__delay_ms(200);

	// PALLE
	set_servo_position (255, 1);

	// TORNA AL PUNTO DI PARTENZA
	distance_default.apply();
	forward_to_point(cratere);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			printf("\nCRATERE1 IMPOSSIBILE PER: MOTOR LOCKED\n\n");
			exit();
			return DONE;
			break;
		case PATH_DONE:
			break;
	}

	printf("\nFINE CRATERE 1\n\n");
	return DONE;
}

// ------------------------------------------------------------

GoalCratere2::GoalCratere2(const char *name)
: Goal(name)
{
}

int GoalCratere2::feasible()
{
	return 2; // Priorità dopo GoalStart
}

success_t GoalCratere2::execute()
{
	bool attesa;

	GraphNode *point = symmNodeifBlue(&Graph::CRATERI_Y);

	Point start_cratere = point->pos;
	Point posizione;

	WaitResult controllo;

	printf("\nDIREZIONE: CRATERE 2\n\n");

	// PATH DI AVVICINAMENTO
	posizione = get_pos(NULL);
	if( !checkExpectedPosition(start_cratere.x, start_cratere.y) ) {

		if(posizione.y <= start_cratere.y) {
			distance_fast.apply();
			if (doPath(symmNodeifBlue(&Graph::PRE_CRATERE_Y)) == IMPOSSIBLE) {
				printf("\nDIREZIONE: CRATERE 2 (0) IMPOSSIBILE\n\n");
				return FAIL;
			}
			wait(MotionEvent());
		}

		distance_default.apply();
		if (doPath(symmNodeifBlue(&Graph::CRATERI_Y)) == IMPOSSIBLE) {
			printf("\nDIREZIONE: CRATERE 2 (1) IMPOSSIBILE\n\n");
			return FAIL;
		}
		wait(MotionEvent());
	}

		// CONTROLLO DEL TEMPO PRIMA DI EFFETTUARE IL GOAL
	  if (((game_timer - start_time) * .25) >= 60) {
			printf("\nCRATERE 2 IMPOSSIBILE PER: TEMPO\n\n");
			return FAIL;
		}

	// ROTAZIONE VERSO IL CRATERE
	rotate_absolute(symmTifBlue(43));
	unchecked_wait(MotionEvent());

	// ATTENDE IL PALLE
	attesa = automation_busy();
	while(attesa) {
	 	printf("\nSTO ATTENDENDO\n");
		attesa = automation_busy();
	};

	// PREPARA L'ASCENSORE
	send_lift_position(0, LIFT_LOAD+4);

	// ABBASSA DI BAFFI
	set_servo_position (UPD_SX, (UPD_SX_DOWN));
	set_servo_position (UPD_DX, (UPD_DX_DOWN));
	__delay_ms(100);

	// ASPETTA TUTTI I MOVIMENTI DEI SERVO
	unchecked_wait(MotionEvent());

	// AVVICINAMENTO AL CRATERE 2
	distance_slow.apply();
	forward_to_distance(125);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			exit_secondo();
			return FAIL;
			break;
		case PATH_DONE:
			break;
	}

		// SPOSTAMENTO DEL CILINDRO
		if(color == YELLOW) {
			set_servo_position(BAF_SX, BAF_SX_CLOSED);		// SINISTRO
			set_servo_position(BAF_DX, BAF_DX_OPEN_Y-20);	// DESTRO
		} else {
			set_servo_position(BAF_SX, BAF_SX_OPEN_B-20);	// SINISTRO
			set_servo_position(BAF_DX, BAF_DX_CLOSED);		// DESTRO
		}

	__delay_ms(500);

	if(color == YELLOW) {
		set_servo_position(BAF_SX, BAF_SX_OPEN_Y);		// SINISTRO
		set_servo_position(BAF_DX, BAF_DX_OPEN_Y-20);	// DESTRO
		__delay_ms(200);
		set_servo_position(BAF_SX, BAF_SX_OFF);			// SINISTRO
	} else {
		set_servo_position(BAF_SX, BAF_SX_OPEN_B-20);	// SINISTRO
		set_servo_position(BAF_DX, BAF_DX_OPEN_B);		// DESTRO
		__delay_ms(200);
		set_servo_position(BAF_DX, BAF_DX_OFF);			// DESTRO
	}

	// AVVICINAMENTO AL CRATERE 2
	forward_to_distance(300);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			exit_secondo();
			return FAIL;
			break;
		case PATH_DONE:
			break;
	}

	// SISTEMA L'ASCENSORE
	lift_go_to(LIFT_LOAD);

	distance_very_slow.apply();
	forward_to_distance(35);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			exit_secondo();
			return FAIL;
			break;
		case PATH_DONE:
			break;
	}

	// CHIUSURA
	chiusura(1, 0);

	__delay_ms(200);

	// PALLE
	set_servo_position (255, 1);

	// TORNA AL PUNTO DI PARTENZA
	distance_default.apply();
	forward_to_point(start_cratere);controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			printf("\nCRATERE2 IMPOSSIBILE PER: MOTOR LOCKED\n\n");
			exit_secondo();
			return DONE;
			break;
		case PATH_DONE:
			break;
	}

	printf("\nFINE CRATERE 2\n\n");
	return DONE;
}

//-------------------------------------------------------------

GoalMegaCratere::GoalMegaCratere(const char *name)
: Goal(name)
{
}

int GoalMegaCratere::feasible()
{
	return 3;
}

success_t GoalMegaCratere::execute()
{
	// ANGOLO DI ROTAZIONE
	int angolo = symmTifBlue(136);

	int i;

	bool attesa;

	GraphNode *point = symmNodeifBlue(&Graph::CRATERI_Y);

	Point start_cratere = point->pos;

	WaitResult controllo;

	printf("\nDIREZIONE: MEGACRATERE\n\n");

	// PATH DI AVVICINAMENTO
	if( !checkExpectedPosition(start_cratere.x, start_cratere.y) ) {

		distance_fast.apply();
		if (doPath(symmNodeifBlue(&Graph::PRE_CRATERE_Y)) == IMPOSSIBLE) {
			printf("\nDIREZIONE: MEGACRATERE (0) IMPOSSIBILE\n\n");
			return FAIL;
		}
		wait(MotionEvent());

		distance_default.apply();
		if (doPath(symmNodeifBlue(&Graph::CRATERI_Y)) == IMPOSSIBLE) {
			printf("\nDIREZIONE: MEGACRATERE (1) IMPOSSIBILE\n\n");
			return FAIL;
		}
		wait(MotionEvent());
	}

	// CONTROLLO DEL TEMPO PRIMA DI EFFETTUARE IL GOAL
  if (((game_timer - start_time) * .25) >= 60) {
		printf("\nMEGACRATERE IMPOSSIBILE PER: TEMPO\n\n");
		return FAIL;
	}

	// ROTAZIONE VERSO IL MEGACRATERE
	rotate_absolute(angolo);
	unchecked_wait(MotionEvent());

	// ALLONTANAMENTO DAL MEGACRATERE
	distance_slow.apply();
	forward_to_distance(-90);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			printf("\nMEGACRATERE IMPOSSIBILE PER: MOTOR LOCKED\n\n");
			exit_megacratere();
			return FAIL;
			break;
		case PATH_DONE:
			break;
	}

	// ATTESA DEL PALLE DAL "SERVO"
	attesa = automation_busy();
	while(attesa) {
	 	printf("\nSTO ATTENDENDO\n");
		attesa = automation_busy();
	};

	// ROTAZIONE VERSO IL MEGACRATERE
	rotate_absolute(angolo);
	unchecked_wait(MotionEvent());

	// PRIMO CATTURA NEL MEGACRATERE
	printf("\nPRIMO CATTURA NEL MEGACRATERE - TEMPO: %.2f\n\n", (game_timer - start_time) * .25);

	// PREPARA L'ASCENSORE
	send_lift_position(0, LIFT_LOAD+4);

	// APERTURA BAFFI
	set_servo_position(UPD_SX, UPD_SX_MID);
	set_servo_position(UPD_DX, UPD_DX_MID);
	__delay_ms(100);

	set_servo_position(BAF_SX, BAF_SX_MAX);
	set_servo_position(BAF_DX, BAF_DX_MAX);
	__delay_ms(100);
/*	FIX_ME: DA PROVARE
	set_servo_position(BAF_SX, BAF_SX_MAX-100);
	set_servo_position(BAF_DX, BAF_DX_MAX+100);
	__delay_ms(100);
*/
	set_servo_position(UPD_SX, UPD_SX_DOWN);
	set_servo_position(UPD_DX, UPD_DX_DOWN);
	__delay_ms(100);

	// AVVICINAMENTO AL PRIMO CATTURA
	distance_slow.apply();
	forward_to_distance(220);
	controllo = wait(MotionEvent());

	switch (controllo) {
		case MOTOR_LOCKED:
			printf("\nMEGACRATERE IMPOSSIBILE PER: MOTOR LOCKED\n\n");
			exit_megacratere();
			return FAIL;
			break;
		case PATH_DONE:
			break;
	}

	// SISTEMA L'ASCENSORE
	lift_go_to(LIFT_LOAD);

	// CHIUDE I BAFFI LENTAMENTE
	set_servo_position(BAF_SX, BAF_SX_MAX-150);
	set_servo_position(BAF_DX, BAF_DX_MAX+150);
	__delay_ms(100);

	set_servo_position(BAF_SX, BAF_SX_MAX-250);
	set_servo_position(BAF_DX, BAF_DX_MAX+250);
	__delay_ms(200);

	// CHIUSURA
	chiusura(2, 1);

	__delay_ms(200);

	// PALLE
	palle();

	__delay_ms(250);

	// CONTROLLO DEL TEMPO PRIMA DI EFFETTUARE IL CATTURA SUCCESSIVO)
	if (((game_timer - start_time) * .25) >= 60) {
		printf("\nMEGACRATERE IMPOSSIBILE PER: TEMPO - TEMPO: %.2f\n\n", (game_timer - start_time) * .25);

		return FAIL;
	}

	// SECONDO CATTURA NEL MEGRATERE
	printf("\nSECONDO CATTURA NEL MEGACRATERE - TEMPO: %.2f\n\n", (game_timer - start_time) * .25);

	// TORNA INDIETRO
	distance_slow.apply();
	forward_to_distance(-210);
	unchecked_wait(MotionEvent());

	// SI RADDRIZZA
	rotate_absolute(angolo);
	unchecked_wait(MotionEvent());

	// APRE I BAFFI
	set_servo_position(BAF_SX, BAF_SX_MAX);
	set_servo_position(BAF_DX, BAF_DX_MAX);
	__delay_ms(150);

	// PREPARA L'ASCENSORE
	send_lift_position(0, LIFT_LOAD+4);

	// AVVICINAMENTO AL MEGACRATERE
	distance_slow.apply();
	forward_to_distance(160);
	unchecked_wait(MotionEvent());

	// CHIUDE I BAFFI
	set_servo_position(BAF_SX, BAF_SX_MAX-170);
	set_servo_position(BAF_DX, BAF_DX_MAX+170);
	__delay_ms(100);

	set_servo_position(BAF_SX, BAF_SX_MAX-270);
	set_servo_position(BAF_DX, BAF_DX_MAX+270);
	__delay_ms(100);

	set_servo_position(UPD_SX, UPD_SX_DOWN-50);
	set_servo_position(UPD_DX, UPD_DX_DOWN+50);
	__delay_ms(100);

	// AVVICINAMENTO MINIMO AL MEGACRATERE
	distance_very_slow.apply();
	forward_to_distance(40);
	unchecked_wait(MotionEvent());

	// SPEGNE I BAFFI
	set_servo_position(BAF_SX, BAF_SX_OFF);
	set_servo_position(BAF_DX, BAF_DX_OFF);
	__delay_ms(100);

	// SISTEMA L'ASCENSORE
	lift_go_to(LIFT_LOAD);

	// AVVICINAMENTO AL MEGACRATERE (FINO IN FONDO)
	distance_slow.apply();
	forward_to_distance(250);
	unchecked_wait(MotionEvent());

	__delay_ms(250);

	// CHIUDE I BAFFI (PER TRATTERERE LE PALLE)
	set_servo_position(UPD_DX, UPD_DX_DOWN+50);
	__delay_ms(20);
	set_servo_position(UPD_SX, UPD_SX_DOWN-50);
	__delay_ms(200);

	// TORNA UN PO' INDIETRO (PER AVERE SPAZIO)
	distance_very_slow.apply();
	forward_to_distance(-80);
	unchecked_wait(MotionEvent());

	// PROVA A CHIUDERLE DENTRO
	for (i=0; i<=5, i++) {
		set_servo_position(BAF_DX, BAF_DX_MAX+500);
		__delay_ms(50);
		set_servo_position(BAF_SX, BAF_SX_MAX-500);
		__delay_ms(80);

		set_servo_position(BAF_SX, BAF_SX_MAX-50);
		__delay_ms(50);
		set_servo_position(BAF_DX, BAF_DX_MAX+50);
		__delay_ms(120);
	}

	__delay_ms(200);

	// CHIUSURA
	chiusura(2, 1);

	__delay_ms(200);

	// PALLE ALTERNATIVO
	palle_alternativo();

	// RITORNO AL PUNTO DI PARTENZA
	distance_default.apply();
	forward_to_point(start_cratere);
	unchecked_wait(MotionEvent());

	printf("\nFINE MEGACRATERE\n\n");
	return DONE;
}

//-------------------------------------------------------------

GoalScarica::GoalScarica(const char *name)
: Goal(name)
{
}

int GoalScarica::feasible()
{
	if (((game_timer - start_time) * .25) >= 60)																														// SI ATTIVA DOPO 60 SECONDI
		return 0;
	else if (goal_cratere_1.isAchieved() && goal_cratere_2.isAchieved() && goal_megacratere.isAchieved())		// SI ATTIVA DOPO AVER COMPLETATO TUTTI I CRATERI
		return 0;
	else
		return IMPOSSIBLE;
}

success_t GoalScarica::execute()
{
	bool attesa;

	GraphNode *point = symmNodeifBlue(&Graph::BASCULA_Y);

	Point pre_basc = point->pos;

	WaitResult bascula;

	printf("\nDIREZIONE: CASA\n\n");

	// PATH DI AVVICINAMENTO
	distance_fast.apply();
	if (doPath(symmNodeifBlue(&Graph::RITORNO_Y), true) == IMPOSSIBLE) {
		printf("\nDIREZIONE: CASA (TRAGITTO) IMPOSSIBILE\n\n");
		return FAIL;
	}
	wait(MotionEvent());

	// ATTESA DEL PALLE DAL "SERVO"
	attesa = automation_busy();
	while(attesa) {
	 	printf("\nSTO ATTENDENDO\n");
		attesa = automation_busy();
	};

	// PRE BASCULA
	distance_slow.apply();
	if (doPath(symmNodeifBlue(&Graph::BASCULA_Y), true) == IMPOSSIBLE) {
		printf("\nDIREZIONE: CASA (VERSO BASCULANTE) IMPOSSIBILE\n\n");
		return FAIL;
	}
	wait(MotionEvent());

	// SI RADDRIZZA
	rotate_absolute(symmTifBlue(0));
	unchecked_wait(MotionEvent());

	// ABBASSA BASCULA
	abbassa_bascula();

	// BASCULA
	distance_slow.apply();
	forward_to_distance(-380);
	bascula = wait(MotionEvent());

	if (bascula == MOTOR_LOCKED) {
		do {
			distance_slow.apply();
			forward_to_point(pre_basc);
			printf("\nBASCULA RITORNO IMPOSSIBILE\n\n");
			distance_default.apply();
			forward_to_distance(-385);
			bascula = wait(MotionEvent());
		} while (bascula == MOTOR_LOCKED);
	}

	// ALZA BASCULA
	set_abbassa_start();
	__delay_ms(200);

	motion_stop();

	// TEMPO DI ATTESA SULLA BASCULA
	__delay_ms(1000);

	// POST BASCULA
	// SI RADDRIZZA
	rotate_absolute(symmTifBlue(0));
	unchecked_wait(MotionEvent());

	// BUMP AND SET
	distance_slow.apply();
	bump_and_set_x(symmXifBlue(DIM_H_BACK), symmTifBlue(0));
	unchecked_wait(MotionEvent());

	// VA IN AVANTI
	forward_to_distance(60);
	unchecked_wait(MotionEvent());

	// SI RADDRIZZA
	rotate_absolute(symmTifBlue(0));
	unchecked_wait(MotionEvent());

	__delay_ms(300);

	// SCARICO
	scarica();

	printf("\nFINE CASA!");
	printf("\nFINE GARA - TEMPO: %.2f\n\n", (game_timer - start_time) * .25);

	return DONE;
}

//---------------------------------------------------------------------------------------

void FunnyAction (void)
{
	// DISATTIVAZIONE OBSTACLE_AVOIDANCE
  disable_obstacle_avoidance();

  FUNNY_ACTION_DIR = 0;
  __delay_ms(500);
  FUNNY_ACTION = 1;
  __delay_ms(2000);
	FUNNY_ACTION = 0;
}
