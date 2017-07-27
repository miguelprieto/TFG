/*
 * grande1_automation.cpp
 */

#include <stdbool.h>
#include <stdio.h>
#include "automation.h"
#include "ecan_lib.h"
#include "fsm.h"
#include "servo.h"

#define SENSE1			PORTBbits.RB2
#define SENSE1_DIR		TRISBbits.TRISB2
#define SENSE2			PORTAbits.RA4
#define SENSE2_DIR		TRISAbits.TRISA4

#define SERVO_PINZA_CUP_SX	0
#define SERVO_PINZA_CUP_SX_SX	1
#define SERVO_PINZA_CUP_SX_DX	2
#define SERVO_PINZA_CUP_DX	3
#define SERVO_PINZA_CUP_DX_SX	4
#define SERVO_PINZA_CUP_DX_DX	5
#define SERVO_PALETTA_DISPENSER	7

#define CUPGRAB_ABBASSA_DELAY	30
#define CUPGRAB_RILASCIA_DELAY	40
#define CUPGRAB_SOLLEVA_DELAY	60
#define CUPGRAB_PINZE_DELAY	10
#define CUPGRAB_AFFERRATO_DELAY 20

class CupGrabFSM : public FiniteStateMachine<CupGrabFSM>
{
	public:
		bool isBusy() const;
		bool isNonEmpty() const;

		// API
		void offAPI();
		void ritraiAPI();
		void rilasciaAPI();
		void startGrabAPI();
		void abbassaAPI();
		void sollevaAPI();
		void sollevaPocoAPI();
		void initAPI();
		
	protected:
		// Stati
		void monitoringState();
		void afferrato1State();
		void afferrato2State();
		void afferrato3State();
		void ritraiState();
		void startGrabState();
		void release1State();
		void release2State();
		void release3State();
		void init1State();
		void init2State();
		void init3State();
		void init4State();
		
		// Funzioni dipendenti dal lato
		virtual void abbassa() = 0;
		virtual void abbassa2() = 0;
		virtual void solleva_poco() = 0;
		virtual void solleva() = 0;
		virtual void ritrai() = 0;
		virtual void apri() = 0;
		virtual void chiudi() = 0;
		virtual void off() = 0;
		virtual bool leggi_omron() = 0;

	private:
		int notSeenCounter;
};

class CupGrabSxFSM : public CupGrabFSM
{
	protected:
		void abbassa();
		void abbassa2();
		void solleva_poco();
		void solleva();
		void ritrai();
		void apri();
		void chiudi();
		void off();
		bool leggi_omron();
};

class CupGrabDxFSM : public CupGrabFSM
{
	protected:
		void abbassa();
		void abbassa2();
		void solleva_poco();
		void solleva();
		void ritrai();
		void apri();
		void chiudi();
		void off();
		bool leggi_omron();
};

static CupGrabSxFSM cupgrab_sx;
static CupGrabDxFSM cupgrab_dx;

// --- Funzioni dipendenti dal lato ---

void CupGrabSxFSM::abbassa()
{
	set_servo(SERVO_PINZA_CUP_SX, 560);
}

void CupGrabSxFSM::abbassa2()
{
	set_servo(SERVO_PINZA_CUP_SX, 600);
}

void CupGrabSxFSM::solleva_poco()
{
	set_servo(SERVO_PINZA_CUP_SX, 820);
}

void CupGrabSxFSM::solleva()
{
	set_servo(SERVO_PINZA_CUP_SX, 930);
}

void CupGrabSxFSM::ritrai()
{
	set_servo(SERVO_PINZA_CUP_SX, 1110);
}

void CupGrabSxFSM::apri()
{
	set_servo(SERVO_PINZA_CUP_SX_SX, 790);
	set_servo(SERVO_PINZA_CUP_SX_DX, 810);
}

void CupGrabSxFSM::chiudi()
{
	set_servo(SERVO_PINZA_CUP_SX_SX, 620);
	set_servo(SERVO_PINZA_CUP_SX_DX, 1005);
}

void CupGrabSxFSM::off()
{
	set_servo(SERVO_PINZA_CUP_SX, 0);
	set_servo(SERVO_PINZA_CUP_SX_SX, 0);
	set_servo(SERVO_PINZA_CUP_SX_DX, 0);
}

bool CupGrabSxFSM::leggi_omron()
{
	return (SENSE2 == 0);
}

void CupGrabDxFSM::abbassa()
{
	set_servo(SERVO_PINZA_CUP_DX, 510);
}

void CupGrabDxFSM::abbassa2()
{
	set_servo(SERVO_PINZA_CUP_DX, 550);
}

void CupGrabDxFSM::solleva_poco()
{
	set_servo(SERVO_PINZA_CUP_DX, 780); // corretto -> 590
}

void CupGrabDxFSM::solleva()
{
	set_servo(SERVO_PINZA_CUP_DX, 890); // corretto -> 590
}

void CupGrabDxFSM::ritrai()
{
	set_servo(SERVO_PINZA_CUP_DX, 1060);
}

void CupGrabDxFSM::apri()
{
    set_servo(SERVO_PINZA_CUP_DX_SX, 790 /*900*/); // it was 790
    set_servo(SERVO_PINZA_CUP_DX_DX, 845 /*730*/); // it was 845
}

void CupGrabDxFSM::chiudi()
{
	set_servo(SERVO_PINZA_CUP_DX_SX, 640);
	set_servo(SERVO_PINZA_CUP_DX_DX, 990);
}

void CupGrabDxFSM::off()
{
	set_servo(SERVO_PINZA_CUP_DX, 0);
	set_servo(SERVO_PINZA_CUP_DX_SX, 0);
	set_servo(SERVO_PINZA_CUP_DX_DX, 0);
}

bool CupGrabDxFSM::leggi_omron()
{
	return (SENSE1 == 0);
}

// --- Gestori degli stati ---

void CupGrabFSM::monitoringState()
{
	if (leggi_omron())
	{
		chiudi();
		setNextState(&CupGrabFSM::afferrato1State, CUPGRAB_AFFERRATO_DELAY);
	}
}

void CupGrabFSM::afferrato1State()
{
	solleva_poco();
	setNextState(&CupGrabFSM::afferrato2State, 100);
}

void CupGrabFSM::afferrato2State()
{
	solleva();
	notSeenCounter = 0;
	setNextState(&CupGrabFSM::afferrato3State);
}


void CupGrabFSM::afferrato3State()
{
	if (leggi_omron())
		notSeenCounter = 0;
	else
		notSeenCounter++;

	if (notSeenCounter > 25) // se per più di 500ms non abbiamo più visto l'oggetto usciamo da questo stato
	{
		ritrai();
		setNextState(&CupGrabFSM::nopState);
	}
}

void CupGrabFSM::ritraiState()
{
	ritrai();
	setNextState(&CupGrabFSM::nopState);
}

void CupGrabFSM::startGrabState()
{
	apri();
	abbassa();
	setNextState(&CupGrabFSM::monitoringState, CUPGRAB_ABBASSA_DELAY);
}

void CupGrabFSM::release1State()
{
	apri();
	setNextState(&CupGrabFSM::release2State, CUPGRAB_PINZE_DELAY);
}

void CupGrabFSM::release2State()
{
	ritrai();
	setNextState(&CupGrabFSM::release3State, CUPGRAB_PINZE_DELAY);

	// TODO: inviare messaggio per comunicare che è stato completato il rilascio!
}

void CupGrabFSM::release3State()
{
	chiudi();
	setNextState(&CupGrabFSM::nopState);
}

void CupGrabFSM::init1State()
{
	abbassa();
	setNextState(&CupGrabFSM::init2State, CUPGRAB_PINZE_DELAY*15);
}

void CupGrabFSM::init2State()
{
	apri();
	setNextState(&CupGrabFSM::init3State, CUPGRAB_PINZE_DELAY);
}

void CupGrabFSM::init3State()
{
	chiudi();
	setNextState(&CupGrabFSM::init4State, CUPGRAB_PINZE_DELAY*2);
}

void CupGrabFSM::init4State()
{
	ritrai();
	setNextState(&CupGrabFSM::nopState);
}
// ------------------------------------------------------------------

void init_automation(void)
{
	SENSE1_DIR = SENSE2_DIR = 1;

	cupgrab_sx.start();
	cupgrab_dx.start();
}

void automation(void)
{
	cupgrab_sx.schedule();
	cupgrab_dx.schedule();

	// Invia aggiornamenti via CAN ogni 5*20ms = 100ms
	static int ctr = 0;
	if (ctr++ == 5)
	{
		t_servo_status_grande1 p;
		ctr = 0;

		p.busy_flags = p.nonempty_flags = 0;

		if (cupgrab_sx.isBusy())
			p.busy_flags |= 1;
		if (cupgrab_dx.isBusy())
			p.busy_flags |= 2;

		if (cupgrab_sx.isNonEmpty())
			p.nonempty_flags |= 1;
		if (cupgrab_dx.isNonEmpty())
			p.nonempty_flags |= 2;

		ecan_send(SERVO_STATUS_GRANDE1_CAN_ID, (unsigned char *)&p, 8, 0);
	}
}

// --- Calcolo dei flag ---

bool CupGrabFSM::isBusy() const
{
	StateHandler current_state = getCurrentState();

	if (current_state == &CupGrabFSM::nopState)
		return false;
	else if (current_state == &CupGrabFSM::monitoringState)
		return false;
	else if (current_state == &CupGrabFSM::afferrato3State)
		return false;
	else
		return true;
}

bool CupGrabFSM::isNonEmpty() const
{
	StateHandler current_state = getCurrentState();

	if (current_state == &CupGrabFSM::afferrato1State)
		return true;
	else if (current_state == &CupGrabFSM::afferrato2State)
		return true;
	else if (current_state == &CupGrabFSM::afferrato3State)
		return true;
	else
		return false;
}

// --- API ---

void CupGrabFSM::offAPI()
{
	off();
	setNextState(&CupGrabFSM::nopState);
}

void CupGrabFSM::ritraiAPI()
{
	chiudi();
	setNextState(&CupGrabFSM::ritraiState, CUPGRAB_PINZE_DELAY);
}

void CupGrabFSM::rilasciaAPI()
{
	abbassa();
	setNextState(&CupGrabFSM::release1State, CUPGRAB_ABBASSA_DELAY);
}

void CupGrabFSM::startGrabAPI()
{
	chiudi();
	solleva();
	setNextState(&CupGrabFSM::startGrabState, CUPGRAB_ABBASSA_DELAY);
}

void CupGrabFSM::initAPI()
{
	chiudi();
	setNextState(&CupGrabFSM::init1State, CUPGRAB_ABBASSA_DELAY*3);
}

void CupGrabFSM::abbassaAPI()
{
	abbassa2();
}

void CupGrabFSM::sollevaAPI()
{
	solleva();
	setNextState(&CupGrabFSM::afferrato1State);
}

void CupGrabFSM::sollevaPocoAPI()
{
	solleva_poco();
	setNextState(&CupGrabFSM::afferrato3State);
}

// -----------------------------------------

void automation_command(const t_servo_position * pos)
{
	switch (pos->position)
	{
		case SERVO_POSITION_GRANDE_CUP_OFF_SX:
			cupgrab_sx.offAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_RITRAI_SX:
			cupgrab_sx.ritraiAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_RILASCIA_SX:
			cupgrab_sx.rilasciaAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_INIT_SX:
			cupgrab_sx.initAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_STARTGRAB_SX:
			cupgrab_sx.startGrabAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_ABBASSA_SX:
			cupgrab_sx.abbassaAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_SOLLEVA_SX:
			cupgrab_sx.sollevaAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_SOLLEVA_POCO_SX:
			cupgrab_sx.sollevaPocoAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_OFF_DX:
			cupgrab_dx.offAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_RITRAI_DX:
			cupgrab_dx.ritraiAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_RILASCIA_DX:
			cupgrab_dx.rilasciaAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_INIT_DX:
			cupgrab_dx.initAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_STARTGRAB_DX:
			cupgrab_dx.startGrabAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_ABBASSA_DX:
			cupgrab_dx.abbassaAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_SOLLEVA_DX:
			cupgrab_dx.sollevaAPI();
			break;
		case SERVO_POSITION_GRANDE_CUP_SOLLEVA_POCO_DX:
			cupgrab_dx.sollevaPocoAPI();
			break;
		default:
			break;
	}
}
