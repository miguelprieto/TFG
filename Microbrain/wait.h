/*
 * wait.h
 */

#ifndef __WAIT_H
#define __WAIT_H

#include "defines.h"
#include "geometry.h"
#include "gpio.h"

enum WaitResult
{
	NOT_OCCURRED = -1, // solo per uso interno, non viene mai restituito dalla wait

	PATH_DONE,
	MOTOR_LOCKED,
	TIMEOUT_EXPIRED,
	OBSTACLE,
	BUMPERS,
	IR_SENSOR,
	POSITION_REACHED,
        POSITION_LOCKED,
	NEAR_DISTANCE,
	GRABBED,
	OMRON_SENSOR,
	TILTED
};

class AbstractWaitableEvent
{
	public:
		virtual ~AbstractWaitableEvent();

		virtual void printDescription() const = 0;
		virtual WaitResult testOccurred() const = 0;
};

/* Attende completamento dei motion commands precedentemente inviati oppure la
 * segnalazione di un ostacolo da parte della obstacle avoidance.
 * Valori restituiti: PATH_DONE, MOTOR_LOCKED, OBSTACLE */
class MotionEvent : public AbstractWaitableEvent
{
	public:
		MotionEvent();
		void printDescription() const;
		WaitResult testOccurred() const;
};

/* Segnala la presenza di un ostacolo rilevato dalla obstacle avoidance.
 * Valori restituiti: OBSTACLE */
class ObstacleEvent : public AbstractWaitableEvent
{
	public:
		ObstacleEvent();
		void printDescription() const;
		WaitResult testOccurred() const;
};

/* Attende al massimo il numero di quarti di secondo specificati, a partire
 * dall'istante corrente oppure da quello indicato
 * Valori restituiti: TIMEOUT_EXPIRED */
class TimeoutEvent : public AbstractWaitableEvent
{
	public:
		explicit TimeoutEvent(int timeout_qsecs, int initial_game_timer = game_timer);
		void printDescription() const;
		WaitResult testOccurred() const;

	private:
		int initial_game_timer, timeout_qsecs;
};

enum PlateBumperSide
{
	BUMPERS_BOTH,
#if defined(ROBOT_GRANDE)
	BUMPERS_LEFT,
	BUMPERS_RIGHT
#else
	BUMPERS_BACK,
	BUMPERS_FRONT
#endif
};

/* Attende la pressione dei _plate_ bumpers oppure un ostacolo
 * Valori restituiti: BUMPERS, MOTOR_LOCKED, OBSTACLE */
class PlateBumperEvent : public AbstractWaitableEvent
{
	public:
		explicit PlateBumperEvent(PlateBumperSide side = BUMPERS_BOTH);
		void printDescription() const;
		WaitResult testOccurred() const;

	private:
		PlateBumperSide side;
};


enum PositionTresholdType
{
	X_LESS_THAN,
	X_GREATER_THAN,
	Y_LESS_THAN,
	Y_GREATER_THAN
};

/* Attende il superamento di una soglia sulla coordinata x o y
 * Valori restituiti: POSITION_REACHED */
class PositionTresholdEvent : public AbstractWaitableEvent
{
	public:
		PositionTresholdEvent(PositionTresholdType type, double reference_value);
		void printDescription() const;
		WaitResult testOccurred() const;

	private:
		PositionTresholdType type;
		double reference_value;
};

/* Verifica che la posizione del robot non sia cambiata dopo un certo numero di campionamenti
  (significa che il robot e' bloccato)
 * Valori restituiti: POSITION_LOCKED */
class PositionLockedEvent : public AbstractWaitableEvent
{
	public:
		PositionLockedEvent();
		void printDescription() const;
		WaitResult testOccurred() const;
		friend WaitResult checkMotion(PositionLockedEvent * t);

	private:
                double x, y, theta;
                double dx, dy, dtheta;
                int lock_counter, lock_max;
};

/* Attende che la distanza del robot da (x, y) diventi minore di r
 * Valori restituiti: NEAR_DISTANCE */
class DistanceTresholdEvent : public AbstractWaitableEvent
{
	public:
		DistanceTresholdEvent(const Point &pos, double r);
		void printDescription() const;
		WaitResult testOccurred() const;

	private:
		Point pos;
		double r, r_squared;
};

#ifdef ROBOT_PICCOLO
/* Attende che il sensore Omron laterale passi nello stato specificato
 * Valori restituiti: OMRON_SENSOR */
class OmronSensorEvent : public AbstractWaitableEvent
{
	public:
		OmronSensorEvent(int side, bool status);
		void printDescription() const;
		WaitResult testOccurred() const;

	private:
		int side;
		bool status;
};

/* Attende che il sensore Baumer front o rear passi nello stato specificato
 * Valori restituiti: OBSTACLE */
class BaumerSensorEvent : public AbstractWaitableEvent
{
	public:
		BaumerSensorEvent(int side, bool status);
		void printDescription() const;
		WaitResult testOccurred() const;

	private:
		int side;
		bool status;
};

#endif

/* wait(...) generica, per aspettare che accada un evento.
 *
 * Ad esempio, per aspettare che i comandi motion precedentemente inviati siano
 * stati completati:
 *	wait(MotionEvent()) -- attende che vengano completati i motion commands
 *
 * È possibile aspettare più di un evento. La wait() ritornerà al verificarsi di
 * almeno uno tra gli eventi specificati.
 * Ad esempio:
 *	wait(MotionEvent(), TimeoutEvent(8)) -- attende che vengano
 *		completati i motion commands oppure che siano passati due secondi
 *
 * Valore restituito:
 *	wait(...) restituisce il codice corrispondente all'evento verificatosi.
 *
 * Nota:
 * Nella maggior parte dei casi, il valore restituito dalla wait(...) può
 * segnalare condizioni impreviste, e per tale motivo deve essere sempre
 * controllato. Il compilatore segnala un warning se il codice che richiama
 * wait(...) ignora il valore restituito.
 *
 */
WaitResult waitImpl(const AbstractWaitableEvent *events[], unsigned int events_count); // per uso interno

template<typename ...Args>
inline WaitResult unchecked_wait(const Args&... args)
{
	// esegue unchecked_wait_array passando un vettori di puntatori a
	// ciascun parametro ricevuto in input
	const AbstractWaitableEvent *list[] = { &args... };
	return waitImpl(list, sizeof(list) / sizeof(list[0]));
};

template<typename ...Args> __attribute__((warn_unused_result))
inline WaitResult wait(const Args&... args)
{
	return unchecked_wait(args...);
};

#endif
