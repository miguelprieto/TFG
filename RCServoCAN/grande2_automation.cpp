/*
 * grande2_automation.cpp
 */

#include <stdbool.h>
#include <stdio.h>
#include "automation.h"
#include "bus_interface.h"
#include "ecan_lib.h"
#include "fsm.h"
#include "servo.h"
#include "sensors.h"

#define IR_TEST_SENSOR_RIGHT()    (PORTBbits.RB0 == 0)
#define SET_IR_SENSOR_RIGHT_DIR() { TRISBbits.TRISB0 = 1; }

#define IR_TEST_SENSOR_LEFT()     (PORTBbits.RB1 == 0)
#define SET_IR_SENSOR_LEFT_DIR()  { TRISBbits.TRISB1 = 1; }

#define VALVE_LEFT_ON()     LATAbits.LATA3 = 1
#define VALVE_LEFT_OFF()    LATAbits.LATA3 = 0

#define VALVE_RIGHT_ON()    LATAbits.LATA2 = 1
#define VALVE_RIGHT_OFF()   LATAbits.LATA2 = 0

#define VACUUM_ON()  LATBbits.LATB3 = 1
#define VACUUM_OFF() LATBbits.LATB3 = 0

#define SET_VALVES() {\
    TRISAbits.TRISA2 = 0;\
    TRISAbits.TRISA3 = 0;\
    TRISBbits.TRISB3 = 0;\
    ODCAbits.ODCA2 = 0;\
    ODCAbits.ODCA3 = 0;\
    ODCBbits.ODCB3 = 0;\
    }

#define SERVO_DITO_LEFT    (10 - NUM_SERVO)
#define SERVO_DITO_RIGHT   (18 - NUM_SERVO)
#define SERVO_PARETE_LEFT  (11 - NUM_SERVO)
#define SERVO_PICK_UP      (12 - NUM_SERVO)
#define SERVO_PICK_DOWN    (17 - NUM_SERVO)
#define SERVO_PARETE_RIGHT (19 - NUM_SERVO)

#define POSIZIONE_UP		80
#define POSIZIONE_MID		22
#define POSIZIONE_DOWN		0

class StandGrabFSM : public FiniteStateMachine<StandGrabFSM>
{
	public:
		StandGrabFSM();

		bool isBusy() const;
		int getStandCount() const;
		bool isBallPresent() const;

		// API
		void offAPI();
		void homeAPI();
		void startAPI();
		void stopAPI();
		void loadAPI();
		void positionMidAPI();
		void releaseAPI();
		void releaseDownAPI();
		void pareteHomeAPI();
		void pinzaNBAAPI();

	protected:
		// Stati
		void monitoring1State();
		void monitoring2State();
		void monitoring3AState();
		void monitoring3BState();
		void apriPinzaState();
		void abbassaPinzaState();
		void chiudiPinzaState();
		void alzaPinzaState();
		void sollevandoState();
		void rilascia1State();
		void rilascia1DownState();
		void rilascia2State();
		void ball_monitoringState();
		void ball_apriPinzaState();
		void ball_abbassaPinza1State();
		void ball_abbassaPinza2State();
		void ball_chiudiPinzaState();
		void ball_alzaPinzaState();
		void pinza_nba1State();
		void pinza_nba2State();

		virtual void off() = 0;
		virtual void pinza_home() = 0;
		virtual bool leggi_omron() = 0;
		virtual bool leggi_standSensor() = 0;
		virtual void home_dito() = 0;
		virtual void chiudi_dito() = 0;
		virtual void apri_dito() = 0;
		virtual void apri_pinza() = 0;
		virtual void chiudi_pinza() = 0;
		virtual void pinza_up() = 0;
		virtual void pinza_position_release() = 0;
		virtual void pinza_mid() = 0;
		virtual void pinza_very_down() = 0;
		virtual void pinza_down() = 0;
		virtual void apri_parete() = 0;
		virtual void chiudi_parete() = 0;
		virtual void parete_NBA() = 0;
		virtual void lift_NBA() = 0;
		virtual void home_parete() = 0;
                virtual void reset_sensor() = 0;

	private:
		int altezza_torre;
		bool ball_presa;
};

class StandGrabSxFSM : public StandGrabFSM
{
	friend class HomeFSM;
	friend class StandGrabFSM;

	protected:
		void off();
		void pinza_home();
		bool leggi_standSensor();
		bool leggi_omron();
		void home_dito();
		void chiudi_dito();
		void apri_dito();
		void apri_pinza();
		void chiudi_pinza();
		void pinza_up();
		void pinza_position_release();
		void pinza_mid();
		void pinza_down();
		void pinza_very_down();
		void apri_parete();
		void chiudi_parete();
		void parete_NBA();
		void lift_NBA();
		void home_parete();
		void reset_sensor();
};

class StandGrabDxFSM : public StandGrabFSM
{
	friend class HomeFSM;
	friend class StandGrabFSM;

	protected:
		void off();
		void pinza_home();
		bool leggi_standSensor();
		bool leggi_omron();
		void home_dito();
		void chiudi_dito();
		void apri_dito();
		void apri_pinza();
		void chiudi_pinza();
		void pinza_up();
		void pinza_position_release();
		void pinza_mid();
		void pinza_down();
		void pinza_very_down();
		void apri_parete();
		void chiudi_parete();
		void parete_NBA();
		void lift_NBA();
		void home_parete();
		void reset_sensor();
};

class CupFrontGrabFSM : public FiniteStateMachine<CupFrontGrabFSM>
{
	public:
		CupFrontGrabFSM();

		bool isBusy() const;
		bool isNonEmpty() const;

		void homeAPI();
		void readyAPI();
		void pickAPI();
		void releaseAPI();
		void NBAAPI();
		void offAPI();
		
	protected:
		void pickState();
		void release1State();
		void release2State();
		void release3State();

	private:
		bool afferrato;
};

class HomeFSM : public FiniteStateMachine<HomeFSM>
{
	public:
		// API
		void offAPI();
		void homeAPI();

	protected:
		// Stati
		void home1State();
		void home2State();
};

static StandGrabSxFSM automation_left;
static StandGrabDxFSM automation_right;
static HomeFSM home_automation;
static CupFrontGrabFSM automation_center;

StandGrabFSM::StandGrabFSM()
: altezza_torre(0), ball_presa(false)
{
}

// --- Funzioni dipendenti dal lato ---

void StandGrabSxFSM::off()
{
	set_servo(SERVO_DITO_LEFT, 0);
	set_servo(SERVO_PARETE_LEFT, 0);
	lift_stop(0);
}

void StandGrabDxFSM::off()
{
	set_servo(SERVO_DITO_RIGHT, 0);
	set_servo(SERVO_PARETE_RIGHT, 0);
	lift_stop(1);
}

void StandGrabSxFSM::pinza_home()
{
	lift_home(0);
}

void StandGrabDxFSM::pinza_home()
{
	lift_home(1);
}

bool StandGrabSxFSM::leggi_omron()
{
    return read_sensor1();
}

bool StandGrabDxFSM::leggi_omron()
{
    return read_sensor2();
}

bool StandGrabSxFSM::leggi_standSensor()
{
	return IR_TEST_SENSOR_LEFT();
}

bool StandGrabDxFSM::leggi_standSensor()
{
	return IR_TEST_SENSOR_RIGHT();
}

void StandGrabSxFSM::home_dito()
{
	set_servo(SERVO_DITO_LEFT, 160);
}

void StandGrabDxFSM::home_dito()
{
	set_servo(SERVO_DITO_RIGHT, 1060);
}

void StandGrabSxFSM::chiudi_dito()
{
	set_servo(SERVO_DITO_LEFT, 850);
}

void StandGrabDxFSM::chiudi_dito()
{
	set_servo(SERVO_DITO_RIGHT, 340);
}

void StandGrabSxFSM::apri_dito()
{
    set_servo(SERVO_DITO_LEFT, 530);
}

void StandGrabDxFSM::apri_dito()
{
    set_servo(SERVO_DITO_RIGHT, 610);
}

void StandGrabSxFSM::apri_pinza()
{
	VALVE_LEFT_OFF();
}

void StandGrabDxFSM::apri_pinza()
{
	VALVE_RIGHT_OFF();
}

void StandGrabSxFSM::chiudi_pinza()
{
	VALVE_LEFT_ON();
}

void StandGrabDxFSM::chiudi_pinza()
{
	VALVE_RIGHT_ON();
}

void StandGrabSxFSM::pinza_up()
{
	send_lift_position(0, 90);
}

void StandGrabDxFSM::pinza_up()
{
	send_lift_position(1, 90);
}

void StandGrabSxFSM::pinza_position_release()
{
	send_lift_position(0, 45);
}

void StandGrabDxFSM::pinza_position_release()
{
	send_lift_position(1, 45);
}

void StandGrabSxFSM::pinza_mid()
{
	send_lift_position(0, 18);
}

void StandGrabDxFSM::pinza_mid()
{
	send_lift_position(1, 18);
}

void StandGrabSxFSM::pinza_down()
{
	send_lift_position(0, 10);
}

void StandGrabDxFSM::pinza_down()
{
    // CS: L'altezza differente rispetto alla sx e' dovuta al fatto che le due pinze
    // hanno altezze meccaniche differenti
    send_lift_position(1, 7);
}

void StandGrabSxFSM::pinza_very_down()
{
	send_lift_position(0, 0);
}

void StandGrabDxFSM::pinza_very_down()
{
	send_lift_position(1, 0);
}

void StandGrabSxFSM::chiudi_parete()
{
	set_servo(SERVO_PARETE_LEFT, 770);
}

void StandGrabDxFSM::chiudi_parete()
{
	set_servo(SERVO_PARETE_RIGHT, 675);
}

void StandGrabSxFSM::apri_parete()
{
	set_servo(SERVO_PARETE_LEFT, 1100);
}

void StandGrabDxFSM::apri_parete()
{
	set_servo(SERVO_PARETE_RIGHT, 350);
}

void StandGrabSxFSM::parete_NBA()
{
	set_servo(SERVO_PARETE_LEFT, 990);
}

void StandGrabDxFSM::parete_NBA()
{
	set_servo(SERVO_PARETE_RIGHT, 450);
}

void StandGrabSxFSM::lift_NBA()
{
	VALVE_LEFT_ON();

	int stand_count = getStandCount();
	if (stand_count == 0)
		send_lift_position(0, 90);
	else if (stand_count == 1)
		send_lift_position(0, 80);
	else if (stand_count == 2)
		send_lift_position(0, 13);
}

void StandGrabDxFSM::lift_NBA()
{
	VALVE_RIGHT_ON();

	int stand_count = getStandCount();
	if (stand_count == 0)
		send_lift_position(1, 90);
	else if (stand_count == 1)
		send_lift_position(1, 80);
	else if (stand_count == 2)
		send_lift_position(1, 13);
}

void StandGrabSxFSM::home_parete()
{
	set_servo(SERVO_PARETE_LEFT, 550);
}

void StandGrabDxFSM::home_parete()
{
	set_servo(SERVO_PARETE_RIGHT, 850);
}

void StandGrabSxFSM::reset_sensor()
{
        clear_sensor1();
}

void StandGrabDxFSM::reset_sensor()
{
        clear_sensor2();
}

// --- Gestori degli stati ---

CupFrontGrabFSM::CupFrontGrabFSM()
: afferrato(false)
{
}

void StandGrabFSM::monitoring1State()
{
	if (leggi_omron())
	{
		chiudi_dito();
		setNextState(&StandGrabFSM::monitoring2State);
	}
}

void StandGrabFSM::monitoring2State()
{
	if (leggi_standSensor()) // Abbiamo davvero preso uno stand?
	{
		altezza_torre++;
		setNextState(&StandGrabFSM::apriPinzaState);
	}
	else
		setNextState(&StandGrabFSM::monitoring3AState, 15);
}

void StandGrabFSM::monitoring3AState()
{
	if (leggi_standSensor()) // Abbiamo davvero preso uno stand?
	{
		chiudi_dito();
		altezza_torre++;
		setNextState(&StandGrabFSM::apriPinzaState, 10);
	}
	else
	{
		apri_dito();
		setNextState(&StandGrabFSM::monitoring3BState, 2);
	}
}

void StandGrabFSM::monitoring3BState()
{
	if (leggi_standSensor()) // Abbiamo davvero preso uno stand?
	{
		chiudi_dito();
		altezza_torre++;
		setNextState(&StandGrabFSM::apriPinzaState);
	}
	else
	{
		chiudi_dito();
		setNextState(&StandGrabFSM::monitoring3AState, 18);
	}
}

void StandGrabFSM::apriPinzaState()
{
	apri_pinza();
	setNextState(&StandGrabFSM::abbassaPinzaState, 10);
}

void StandGrabFSM::abbassaPinzaState()
{
	pinza_down();
	setNextState(&StandGrabFSM::chiudiPinzaState, 40);
}

void StandGrabFSM::chiudiPinzaState()
{
	chiudi_pinza();
	apri_dito();
	setNextState(&StandGrabFSM::alzaPinzaState, 100);
}

void StandGrabFSM::alzaPinzaState()
{
	if (altezza_torre == 4)
		pinza_mid();
	else
		pinza_up();
	
	setNextState(&StandGrabFSM::sollevandoState, 100);
}

void StandGrabFSM::sollevandoState()
{
	setNextState(&StandGrabFSM::nopState);
}

void StandGrabFSM::ball_monitoringState()
{
	if (leggi_omron())
	{
		chiudi_dito();
		setNextState(&StandGrabFSM::ball_apriPinzaState, 20);
	}
}

void StandGrabFSM::ball_apriPinzaState()
{
	apri_pinza();
	setNextState(&StandGrabFSM::ball_abbassaPinza1State, 4);
}

void StandGrabFSM::ball_abbassaPinza1State()
{
	pinza_down();
	setNextState(&StandGrabFSM::ball_abbassaPinza2State, 40);
}

void StandGrabFSM::ball_abbassaPinza2State()
{
	apri_dito();
	pinza_very_down();
	setNextState(&StandGrabFSM::ball_chiudiPinzaState, 40);
}

void StandGrabFSM::ball_chiudiPinzaState()
{
	chiudi_pinza();
	automation_right.home_dito();
	setNextState(&StandGrabFSM::ball_alzaPinzaState, 50);
}

void StandGrabFSM::ball_alzaPinzaState()
{
	pinza_mid();
	home_parete();
	ball_presa = true;
	automation_left.home_dito();
	setNextState(&StandGrabFSM::nopState);
}

void StandGrabFSM::rilascia1DownState()
{
	pinza_down();
	setNextState(&StandGrabFSM::rilascia2State, 40);
}

void StandGrabFSM::rilascia1State()
{
	pinza_mid();
	setNextState(&StandGrabFSM::rilascia2State, 40);
}

void StandGrabFSM::rilascia2State()
{
	apri_pinza();
	apri_parete();
	setNextState(&StandGrabFSM::nopState);
}

void CupFrontGrabFSM::pickState()
{
	set_servo(SERVO_PICK_DOWN, 1280);
	set_servo(SERVO_PICK_UP, 1170);

	afferrato = true;

	setNextState(&CupFrontGrabFSM::nopState);
}

void CupFrontGrabFSM::release1State()
{
	VACUUM_OFF();
	setNextState(&CupFrontGrabFSM::release2State, 25);
}

void CupFrontGrabFSM::release2State()
{
	set_servo(SERVO_PICK_UP, 1070); // Alzo la pinza e continuo a tenere il bicchiere così non cade
	setNextState(&CupFrontGrabFSM::release3State, 50);
}

void CupFrontGrabFSM::release3State()
{
	set_servo(SERVO_PICK_DOWN, 825);
	set_servo(SERVO_PICK_UP, 470); // home -> corretto prima 410
	setNextState(&CupFrontGrabFSM::nopState);
}


// ----------------------------------------------------------------

void HomeFSM::home1State()
{
	automation_left.pinza_down();
	automation_right.pinza_down();

	automation_right.home_dito();
	setNextState(&HomeFSM::home2State, 40);
}

void HomeFSM::home2State()
{
	automation_left.home_dito();

	set_servo(SERVO_PARETE_LEFT, 550);
	set_servo(SERVO_PARETE_RIGHT, 850);

	setNextState(&HomeFSM::nopState);
}

// ------------------------------------------------------------------

void init_automation(void)
{
	init_sensors(SENSOR_INTERRUPT, EDGE_FALLING, SENSOR_INTERRUPT, EDGE_FALLING);

	SET_IR_SENSOR_LEFT_DIR();
	SET_IR_SENSOR_RIGHT_DIR();
	SET_VALVES();

	VALVE_LEFT_OFF();
	VALVE_RIGHT_OFF();
        VACUUM_OFF();

	automation_left.start();
	automation_right.start();
	home_automation.start();
	automation_center.start();
}

void automation(void)
{
	automation_left.schedule();
	automation_right.schedule();
	home_automation.schedule();
	automation_center.schedule();

	// Invia aggiornamenti via CAN ogni 5*20ms = 100ms
	static int ctr = 0;
	if (ctr++ == 5)
	{
		t_servo_status_grande2 p;
		ctr = 0;

		p.busy_flags = p.nonempty_flags = 0;

		if (automation_center.isBusy())
			p.busy_flags |= 1;
		if (automation_left.isBusy())
			p.busy_flags |= 2;
		if (automation_right.isBusy())
			p.busy_flags |= 4;

		if (automation_center.isNonEmpty())
			p.nonempty_flags |= 1;
		if (automation_left.isBallPresent())
			p.nonempty_flags |= 2;
		if (automation_right.isBallPresent())
			p.nonempty_flags |= 4;

		p.left_stand_count = automation_left.getStandCount();
		p.right_stand_count = automation_right.getStandCount();

		ecan_send(SERVO_STATUS_GRANDE2_CAN_ID, (unsigned char *)&p, 8, 0);
	}
}

// --- Calcolo dei flag ---

bool StandGrabFSM::isBusy() const
{
	StateHandler current_state = getCurrentState();

	if (current_state == &StandGrabFSM::nopState)
		return false;
	else if (current_state == &StandGrabFSM::monitoring1State)
		return false;
	else if (current_state == &StandGrabFSM::monitoring2State)
		return false;
	else if (current_state == &StandGrabFSM::monitoring3AState)
		return false;
	else if (current_state == &StandGrabFSM::monitoring3BState)
		return false;
	else if (current_state == &StandGrabFSM::ball_monitoringState)
		return false;
	else
		return true;
}

int StandGrabFSM::getStandCount() const
{
	return altezza_torre;
}

bool StandGrabFSM::isBallPresent() const
{
	return ball_presa;
}

bool CupFrontGrabFSM::isBusy() const
{
	StateHandler current_state = getCurrentState();

	if (current_state == &CupFrontGrabFSM::nopState)
		return false;
	else
		return true;
}

bool CupFrontGrabFSM::isNonEmpty() const
{
	return afferrato;
}

// --- API ---

/*
 * - OFF
 * - HOME: sale a posizione alta + reset contatore
 * - START_SX
 * - START_DX
 * - STOP_SX
 * - STOP_DX
 * - LOAD_SX: carica un oggetto e resetta anche il contatore
 * - LOAD_DX: carica un oggetto e resetta anche il contatore
 */

void StandGrabFSM::offAPI()
{
	off();
	apri_pinza();
	setNextState(&StandGrabFSM::nopState);
}

void StandGrabFSM::homeAPI()
{
	apri_pinza();
	pinza_home();
	apri_dito();
	altezza_torre = 0;
	ball_presa = false;
}

void StandGrabFSM::startAPI()
{
        reset_sensor();
	if (altezza_torre == 4)
		return;

	pinza_up();
	automation_left.apri_dito();
	automation_right.apri_dito();
	chiudi_parete();
	setNextState(&StandGrabFSM::monitoring1State, 20);
}

void StandGrabFSM::stopAPI()
{
	apri_dito();
	setNextState(&StandGrabFSM::nopState);
}

void StandGrabFSM::loadAPI()
{
        reset_sensor();
	pinza_up();
	automation_left.apri_dito();
	automation_right.apri_dito();
	altezza_torre = 0;
	chiudi_parete();
	setNextState(&StandGrabFSM::ball_monitoringState, 20);
}

void StandGrabFSM::positionMidAPI()
{
	pinza_position_release();
}

void StandGrabFSM::releaseDownAPI()
{
	altezza_torre = 0;
	ball_presa = false;
	setNextState(&StandGrabFSM::rilascia1DownState);
}

void StandGrabFSM::releaseAPI()
{
	altezza_torre = 0;
	ball_presa = false;
	setNextState(&StandGrabFSM::rilascia1State);
}

void StandGrabFSM::pareteHomeAPI()
{
	home_parete();
	setNextState(&StandGrabFSM::nopState, 20);
}

void StandGrabFSM::pinzaNBAAPI()
{
	ball_presa = true;
	setNextState(&StandGrabFSM::pinza_nba1State);
}

void StandGrabFSM::pinza_nba1State()
{
	if (getStandCount() < 3)
	{
		lift_NBA();
		setNextState(&StandGrabFSM::pinza_nba2State, 10);
	}
	else
		setNextState(&StandGrabFSM::nopState);
}

void StandGrabFSM::pinza_nba2State()
{
	parete_NBA();
	setNextState(&StandGrabFSM::nopState);
}

void CupFrontGrabFSM::homeAPI()
{
	VACUUM_OFF();
	set_servo(SERVO_PICK_DOWN, 825);
	set_servo(SERVO_PICK_UP, 440); // home
	setNextState(&CupFrontGrabFSM::nopState);
}

void CupFrontGrabFSM::readyAPI()
{
	afferrato = false;
	set_servo(SERVO_PICK_DOWN, 860);
	set_servo(SERVO_PICK_UP, 850);
	VACUUM_ON();
	setNextState(&CupFrontGrabFSM::nopState);
}

void CupFrontGrabFSM::pickAPI()
{
	afferrato = false;
	set_servo(SERVO_PICK_UP, 1360); // was 1320
	setNextState(&CupFrontGrabFSM::pickState, 70);
}

void CupFrontGrabFSM::releaseAPI()
{
	afferrato = false;
	set_servo(SERVO_PICK_DOWN, 940);
	set_servo(SERVO_PICK_UP, 1250);
	setNextState(&CupFrontGrabFSM::release1State, 25);
}

void CupFrontGrabFSM::NBAAPI()
{
	if (afferrato)
	{
		set_servo(SERVO_PICK_DOWN, 940);
		set_servo(SERVO_PICK_UP, 1250);
	}
}

void CupFrontGrabFSM::offAPI()
{
	afferrato = false;
	VACUUM_OFF();
	set_servo(SERVO_PICK_DOWN, 0);
	set_servo(SERVO_PICK_UP, 0);
	setNextState(&CupFrontGrabFSM::nopState);
}

void HomeFSM::offAPI()
{
	setNextState(&HomeFSM::nopState);
}

void HomeFSM::homeAPI()
{
	setNextState(&HomeFSM::home1State, 200);
}

void automation_command(const t_servo_position * pos)
{
	switch (pos->position)
	{
		case SERVO_POSITION_GRANDE_STANDGRAB_OFF:
			automation_left.offAPI();
			automation_right.offAPI();
			home_automation.offAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_HOME:
			automation_left.stopAPI();
			automation_left.homeAPI();
			automation_right.stopAPI();
			automation_right.homeAPI();
			home_automation.homeAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_START_SX:
			automation_left.startAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_START_DX:
			automation_right.startAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_STOP_SX:
			automation_left.stopAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_STOP_DX:
			automation_right.stopAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_LOAD_SX:
			automation_left.loadAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_LOAD_DX:
			automation_right.loadAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_POSITIONMID_SX:
			automation_left.positionMidAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_POSITIONMID_DX:
			automation_right.positionMidAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_RELEASE_SX:
			automation_left.releaseAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_RELEASEDOWN_SX:
			automation_left.releaseDownAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_RELEASE_DX:
			automation_right.releaseAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_RELEASEDOWN_DX:
			automation_right.releaseDownAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_CHIUDIPARETE_SX:
			automation_left.pareteHomeAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_CHIUDIPARETE_DX:
			automation_right.pareteHomeAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_NBA_POSITION_SX:
			automation_left.pinzaNBAAPI();
			if (automation_center.isNonEmpty())
				automation_center.NBAAPI();
			break;
		case SERVO_POSITION_GRANDE_STANDGRAB_NBA_POSITION_DX:
			automation_right.pinzaNBAAPI();
			if (automation_center.isNonEmpty())
				automation_center.NBAAPI();
			break;
		case SERVO_POSITION_GRANDE_VALVOLA_SX_OFF:
			VALVE_LEFT_OFF();
			break;
		case SERVO_POSITION_GRANDE_VALVOLA_SX_ON:
			VALVE_LEFT_ON();
			break;
		case SERVO_POSITION_GRANDE_VALVOLA_DX_OFF:
			VALVE_RIGHT_OFF();
			break;
		case SERVO_POSITION_GRANDE_VALVOLA_DX_ON:
			VALVE_RIGHT_ON();
			break;
		case SERVO_POSITION_GRANDE_VACUUM_OFF:
			VACUUM_OFF();
			break;
		case SERVO_POSITION_GRANDE_VACUUM_ON:
			VACUUM_ON();
			break;
		case SERVO_POSITION_GRANDE_CUPGRAB_HOME:
			automation_center.homeAPI();
			break;
		case SERVO_POSITION_GRANDE_CUPGRAB_READY:
			automation_center.readyAPI();
			break;
		case SERVO_POSITION_GRANDE_CUPGRAB_PICK:
			automation_center.pickAPI();
			break;
		case SERVO_POSITION_GRANDE_CUPGRAB_RELEASE:
			automation_center.releaseAPI();
			break;
		case SERVO_POSITION_GRANDE_CUPGRAB_OFF:
			automation_center.offAPI();
			break;
	}
}
