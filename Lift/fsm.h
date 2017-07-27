/*
 * fsm.h
 */

#ifndef __FSM_H
#define __FSM_H

struct t_fsm;

typedef void   (*state_proc)(struct t_fsm *);

#define MAX_STATES   32

typedef struct t_fsm {
    int running;
    int current_state;
    int counter;
    int max_counter;
    state_proc proc[MAX_STATES];
    void * data;
} t_fsm;

void init_fsm(t_fsm * fsm, void * data, int first_state);
void add_fsm(t_fsm * fsm, int state, state_proc proc);
void start_fsm(t_fsm * fsm);
void stop_fsm(t_fsm * fsm);
void next_fsm(t_fsm * fsm, int next_state);
void next_fsm_after(t_fsm * fsm, int next_state, int c);
void schedule_fsm(t_fsm * fsm);


#endif
