/*
 * menu.h
 */

#ifndef __MENU_H
#define __MENU_H

#include "defines.h"

typedef struct
{
    const char *nome_comando, *help_text;
    void (*handler)(int argc, const char **argv);
    int num_args;
} menu_command_t;

// Comandi specifici di ciascun robot (i.e. non condivisi da entrambi i robot)
extern const menu_command_t comandi_robot[];
extern const int num_comandi_robot;


#endif
