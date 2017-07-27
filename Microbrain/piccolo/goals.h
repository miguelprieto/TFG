/*
 *	goals.h
 */


#ifndef __GOALS_H
#define __GOALS_H

#include <stdbool.h>

#include "goal_manager.h"
#include "servos.h"

typedef enum {
    BOTTOM_DISPENSER,
    YELLOW_SIDE_DISPENSER,
    BLUE_SIDE_DISPENSER
} t_dispenser;


/* void capture_from_dispenser(t_dispenser dispenser_side); */
/* bool capture_module_from_dispenser(t_side robot_side, t_dispenser dispenser_side, bool up); */

class GoalStart : public Goal
{
	public:
		GoalStart(const char *name);
		int feasible();
		success_t execute();
};

class GoalModule5 : public Goal
{
       public:
                GoalModule5(const char *name);
                int feasible();
                success_t execute();
};

class GoalModule3 : public Goal
{
       public:
                GoalModule3(const char *name);
                int feasible();
                success_t execute();
};

class GoalModule2 : public Goal
{
       public:
                GoalModule2(const char *name);
                int feasible();
                success_t execute();
};

class GoalRelease1 : public Goal
{
       public:
                GoalRelease1(const char *name);
                int feasible();
                success_t execute();
};

class GoalDispenser1 : public Goal
{
	public:
		GoalDispenser1(const char *name);
		int feasible();
		success_t execute();
};

class GoalRelease2 : public Goal
{
        public:
		GoalRelease2(const char *name);
		int feasible();
		success_t execute();
};

class GoalRelease3 : public Goal
{
	public:
		GoalRelease3(const char *name);
		int feasible();
		success_t execute();
};
void FunnyAction(void);

#endif
