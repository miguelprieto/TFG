/*
 * fsm.c
 */

#include <stdio.h>
#include "fsm.h"


void init_fsm(t_fsm * fsm, void * data, int first_state)
{
    int i;
    for (i = 0; i < MAX_STATES;i++)
        fsm->proc[i] = NULL;
    fsm->data = data;
    fsm->running = 0;
    fsm->current_state = first_state;
}


void add_fsm(t_fsm * fsm, int state, state_proc proc)
{
    fsm->proc[state] = proc;
}


void start_fsm(t_fsm * fsm)
{
    fsm->counter = fsm->max_counter = 0;
    fsm->running = 1;
}


void stop_fsm(t_fsm * fsm)
{
    fsm->running = 0;
}


void next_fsm(t_fsm * fsm, int next_state)
{
    fsm->current_state = next_state;
    fsm->max_counter = 0;
}

void next_fsm_after(t_fsm * fsm, int next_state, int c)
{
    fsm->current_state = next_state;
    fsm->max_counter = c;
}

void schedule_fsm(t_fsm * fsm)
{
    if (fsm->running) {
        ++fsm->counter;
        if (fsm->counter >= fsm->max_counter) {
            fsm->counter = 0;
            fsm->proc[fsm->current_state](fsm);
        }
    }
}

