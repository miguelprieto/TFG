/*
 * goal_manager.c
 */

#include "defines.h"

#include <libpic30++.h>
#include <stdio.h>
#include <string.h>
#include "console.h"
#include "goal_manager.h"
#include "goals.h"
#include "bus_interface.h"
#include "routing.h"

static Goal *goalListHead = NULL;	// primo elemento della lista dei goal
static Goal *runningGoal = NULL;	// ultimo goal eseguito

void resetRunningGoal()
{
    runningGoal = NULL;
}

Goal::Goal(const char *name)
: m_name(name)
{
	// inserimento in coda (potremmo inserire in modo più efficiente in
	// testa, ma in questo modo preserviamo l'ordine di inserimento)
	Goal **previousGoalPointer = &goalListHead;
	while (*previousGoalPointer != NULL)
		previousGoalPointer = &(*previousGoalPointer)->m_goalListNext;
	*previousGoalPointer = this;
	m_goalListNext = NULL;
}

void Goal::reset()
{
	// I goal possono reimplementare questa funzione per resettare lo stato interno
}

Goal* Goal::findHighestPriorityGoal()
{
	int current_fitness, selected_fitness = INT_MAX;
	Goal *selected = NULL;

	for (Goal *g = goalListHead; g != NULL; g = g->m_goalListNext)
	{
		if (g->m_achieved == false)
		{
			// -1 = impossibile, 0 = priorità massima, INT_MAX = priorità minima
			current_fitness = g->feasible();

			if (current_fitness < 0)
				continue;

			// Abbassiamo la priorità dell'ultimo goal eseguito, in modo da evitare di rischedularlo, se possibile
			if (g == runningGoal)
				current_fitness = INT_MAX;

			printf("%s %d\n", g->m_name, current_fitness);

			if (current_fitness <= selected_fitness)
			{
				selected_fitness = current_fitness;
				selected = g;
			}
		}
	}

	return selected;
}

void Goal::run()
{
	printf("Executing goal %s...\n", m_name);
	m_achieved = execute() == DONE;
	printf("Executed %s : Exited with: %s\n\n", m_name, m_achieved ? "DONE" : "FAIL");
}

GoalIterator::GoalIterator()
{
	rewind();
}

void GoalIterator::rewind()
{
	nextGoal = goalListHead;
}

Goal* GoalIterator::next()
{
	Goal *result = nextGoal;
	if (result != NULL)
		nextGoal = nextGoal->m_goalListNext;
	return result;
}

void resetAllGoals()
{
	for (Goal *g = goalListHead; g != NULL; g = g->m_goalListNext)
	{
		g->reset();
		g->m_achieved = false;
	}
}

void runGoalSystem()
{
	resetAllGoals();

	while (true)
	{
		/* A volte capita che, anche se la wait(CHECK_OBSTACLE()) ha già
		 * gestito l'ostacolo, l'ObstacleAvoidance invii immediatamente
		 * un nuovo stop.
		 * Effettuiamo quindi un unlock per precauzione prima di avviare
		 * il nuovo goal */
		enable_xbee_console();
		double obstdir;
		if (check_obstacle_detected(&obstdir))
		{
			printf("Unlock obstacle avoidance di emergenza (l'ostacolo era in direzione %f)\n", obstdir);
			closePointsInDirection(obstdir);
		}
		
		printf("SCHEDULING (tempo trascorso = %.2f).......\n", (game_timer - start_time) * .25);
		runningGoal = Goal::findHighestPriorityGoal();
		if (runningGoal)
		{
			runningGoal->run();
		}
		else
		{
			printf("Nessun goal da eseguire, aspetto un secondo e riprovo\n");
			__delay_ms(1000);
		}

		__delay_ms(50);
		openEdgesToBeOpened(); // riapre archi chiusi da troppo tempo
	}
}

Goal* findGoalByName(const char *name)
{
	for (Goal *g = goalListHead; g != NULL; g = g->m_goalListNext)
	{
		if (strcmp(g->name(), name) == 0)
			return g;
	}

	return NULL;
}
