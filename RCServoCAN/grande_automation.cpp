/*
 * grande_automation.cpp
 */

#include <stdbool.h>
#include <stdio.h>
#include "automation.h"
#include "ecan_lib.h"
#include "fsm.h"
#include "servo.h"
#include "bus_interface.h"


// ASCENSORE
#define LIFT_LOAD		-174
#define LIFT_LOAD_MID	LIFT_LOAD+21

#define LIFT_BALL		-103
#define LIFT_BALL_BAF	LIFT_BALL-35
#define LIFT_BALL_MID	LIFT_BALL-12

#define LIFT_RUN		-155
#define LIFT_CASA		LIFT_BALL+30

#define LIFT_BASC_GIU	-170
#define LIFT_BASC_SU	LIFT_BASC_GIU+40


// ABBASSA BASCULA
#define ABB_BASC		1

#define ABB_ALTA		370
#define ABB_BASSA		1320


// PARETIE
#define PAR_SX			5
#define PAR_DX			9

#define PAR_DX_OPEN 	670
#define PAR_DX_CLOSED 	1330

#define PAR_SX_OPEN 	1060
#define PAR_SX_CLOSED 	450

#define PAR_PALLE_1 	860


// BAFFO
#define BAF_SX 		0
#define BAF_DX		4
#define UPD_SX		2
#define UPD_DX		7

#define BAF_SX_OPEN	1190
#define BAF_DX_OPEN	460

#define BAF_SX_OPEN_Y	1010
#define BAF_DX_OPEN_Y	590

#define BAF_SX_OPEN_B	1000
#define BAF_DX_OPEN_B	630

#define BAF_SX_CLOSED	470
#define BAF_DX_CLOSED 	1130

#define BAF_SX_MAX	1290
#define BAF_DX_MAX	330

#define BAF_SX_UPD	1000
#define BAF_DX_UPD	620

#define UPD_SX_UP	1150
#define UPD_DX_UP	390

#define UPD_SX_DOWN	630
#define UPD_DX_DOWN	910

#define UPD_SX_MID	950
#define UPD_DX_MID	590

#define BAF_SX_OFF	0
#define BAF_DX_OFF	0

#define UPD_SX_OFF	0
#define UPD_DX_OFF	0


// SPAZZOLA SCARICA
#define SPA 8

#define SPA_CEN 	790

#define SPA_YEL 	1160

#define SPA_BLU 	460



class LoadPalleFSM : public FiniteStateMachine<LoadPalleFSM>
{
public:
    // API
    void doLoad();
    void off();
    bool isBusy() { return getCurrentState() != &LoadPalleFSM::nopState; };

protected:
    // Stati
    void apriParetie();
    void liftPosizioneMid35();
    void liftPosizioneMid();
    void bafSpegniBaffi();
    void updAbbassaBaffi();
    void chiudiBafSinistaPosizioneMid();
    void chiudiSinistraHalf();
    void liftPosizioneUp();
    void chiudiDestra();
    void chiudiSinistra();		
    void bafChiusi();			
    void liftRun();		
    void bafPrimo();			
};

static LoadPalleFSM loadpalle;

// --- Gestori degli stati ---

void LoadPalleFSM::apriParetie()
{
    set_servo(PAR_DX, PAR_DX_OPEN); // DESTRA
    set_servo(PAR_SX, PAR_SX_OPEN); // SINISTRA

    setNextState(&LoadPalleFSM::liftPosizioneMid35, 7); // 140ms
}

void LoadPalleFSM::liftPosizioneMid35()
{
    send_lift_position(0, LIFT_BALL_BAF);
	set_servo(UPD_DX, UPD_DX_DOWN-20);
	set_servo(UPD_SX, UPD_SX_DOWN+20);
    setNextState(&LoadPalleFSM::liftPosizioneMid);
}

void LoadPalleFSM::liftPosizioneMid()
{
    if (lift_check_position(LIFT_BALL_BAF)) {

        set_servo(BAF_DX, BAF_DX_CLOSED+180);
        setNextState(&LoadPalleFSM::chiudiBafSinistaPosizioneMid, 4); // 80ms
    }
}

void LoadPalleFSM::chiudiBafSinistaPosizioneMid() {
        set_servo(BAF_SX, BAF_SX_CLOSED-180);

        send_lift_position(0, LIFT_BALL_MID);
        setNextState(&LoadPalleFSM::bafSpegniBaffi);
}

void LoadPalleFSM::bafSpegniBaffi() 
{
    if (lift_check_position(LIFT_BALL_MID)) {

        set_servo(BAF_SX, BAF_SX_CLOSED);
        set_servo(BAF_DX, BAF_DX_CLOSED);

        setNextState(&LoadPalleFSM::updAbbassaBaffi, 5); // 100ms
    }
}

void LoadPalleFSM::updAbbassaBaffi() 
{
	set_servo(UPD_SX, UPD_SX_DOWN);
        set_servo(UPD_DX, UPD_DX_DOWN);

        setNextState(&LoadPalleFSM::chiudiSinistraHalf, 10); // 200ms

}

void LoadPalleFSM::chiudiSinistraHalf()
{
	set_servo(PAR_SX, PAR_PALLE_1); // SINISTRA

	setNextState(&LoadPalleFSM::liftPosizioneUp, 10); // 200ms
}

void LoadPalleFSM::liftPosizioneUp()
{
    send_lift_position(0, LIFT_BALL);
    setNextState(&LoadPalleFSM::chiudiDestra, 10); // 200ms
}

void LoadPalleFSM::chiudiDestra()
{
    if (lift_check_position(LIFT_BALL)) {
        set_servo(PAR_DX, PAR_DX_CLOSED);  // DESTRA
    	set_servo(BAF_SX, BAF_SX_CLOSED);
        setNextState(&LoadPalleFSM::chiudiSinistra, 5); // 100ms
    }
}

void LoadPalleFSM::chiudiSinistra()
{
    set_servo(PAR_SX, PAR_SX_CLOSED); // SINISTRA
    set_servo(BAF_DX, BAF_DX_CLOSED);
    setNextState(&LoadPalleFSM::bafChiusi, 3); // 60ms
}

void LoadPalleFSM::bafChiusi()
{	
    set_servo(UPD_SX, UPD_SX_DOWN);
    set_servo(UPD_DX, UPD_DX_DOWN);
    setNextState(&LoadPalleFSM::liftRun, 5); // 100ms
}

void LoadPalleFSM::liftRun()
{	
    set_servo(BAF_SX, BAF_SX_UPD);
    set_servo(BAF_DX, BAF_DX_UPD);
    send_lift_position(0, LIFT_RUN);
    setNextState(&LoadPalleFSM::bafPrimo, 5); // 100ms
}

void LoadPalleFSM::bafPrimo()
{	
    set_servo(UPD_SX, UPD_SX_UP);
    set_servo(UPD_DX, UPD_DX_UP);
    setNextState(&LoadPalleFSM::nopState);
}

// ------------------------------------------------------------------

void init_automation(void)
{
    loadpalle.start();
}

void automation(void)
{
    loadpalle.schedule();

    // Invia aggiornamenti via CAN ogni 5*20ms = 100ms
    static int ctr = 0;
    if (ctr++ == 5) {
        t_servo_status_grande p;
        ctr = 0;

        p.busy_flags = loadpalle.isBusy();

        ecan_send(SERVO_STATUS_GRANDE_CAN_ID, (unsigned char *)&p, 8, 0);
    }
}


// --- API ---

void LoadPalleFSM::doLoad()
{
    setNextState(&LoadPalleFSM::apriParetie);
}

void LoadPalleFSM::off()
{
    setNextState(&LoadPalleFSM::nopState);
}

// -----------------------------------------

void automation_command(const t_servo_position * pos)
{
    switch (pos->position) {

    case SERVO_POSITION_GRANDE_OFF:
        loadpalle.off();
        break;

    case SERVO_POSITION_GRANDE_LOAD_PALLE:
        loadpalle.doLoad();
        break;

    default:
        break;

    }
}
