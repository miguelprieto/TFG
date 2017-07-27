/*
 *	goals.h
 */


#ifndef __GOALS_H
#define __GOALS_H

#include <stdbool.h>

#include "goal_manager.h"

class GoalStart : public Goal
{
	public:
		GoalStart(const char *name);
		int feasible();
		success_t execute();
};

class GoalCratere1 : public Goal
{
	public:
		GoalCratere1(const char *name);
		int feasible();
		success_t execute();
};

class GoalCratere2 : public Goal
{
	public:
		GoalCratere2(const char *name);
		int feasible();
		success_t execute();
};

class GoalMegaCratere : public Goal
{
	public:
		GoalMegaCratere(const char *name);
		int feasible();
		success_t execute();
};

class GoalScarica : public Goal
{
	public:
		GoalScarica(const char *name);
		int feasible();
		success_t execute();
};


void FunnyAction(void);

// FUNZIONI

void palle(void);
void chiusura(int n, int controllo);
void scarica(void);

#endif
