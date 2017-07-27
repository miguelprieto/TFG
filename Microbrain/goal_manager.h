
/*
 * goal_manager.h
 */

#ifndef __GOAL_MANAGER
#define __GOAL_MANAGER

#include <stdbool.h>
#include <limits.h>

typedef enum
{
	FAIL,	// 0
	DONE	// 1
} success_t;

class Goal
{
	friend class GoalIterator;
	friend void resetAllGoals();
	friend void runGoalSystem();
	friend Goal* findGoalByName(const char *name);

       	public:
		Goal(const char *name);
 
		void run(); // esegue il goal (richiama la execute() internamente)
		const char* name() const { return m_name; }

		bool isAchieved() const { return m_achieved; }

	protected:
		// Funzioni implementate da ciascun goal
		virtual void reset();
		virtual int feasible() = 0;
		virtual success_t execute() = 0;

		// Gli oggetti di tipo Goal non possono essere copiati
		Goal(const Goal&) = delete;
		Goal &operator=(const Goal&) = delete;

	private:
		const char* m_name;	// nome del goal
		bool m_achieved;	// il goal è stato già eseguito con successo?
		Goal *m_goalListNext;	// puntatore a goal successivo in lista goal

		// Funzione ausiliaria utilizzata dallo scheduler
		static Goal* findHighestPriorityGoal();
};

class GoalIterator
{
	public:
		GoalIterator();

		void rewind();
		Goal *next();

	private:
		Goal *nextGoal;
};

// Funzioni globali dello scheduler
void resetAllGoals();
void resetRunningGoal();
void runGoalSystem() __attribute__((noreturn));
Goal* findGoalByName(const char *name);

#endif
