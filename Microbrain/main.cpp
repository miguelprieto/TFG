/*
 * main.c
 */

#include "defines.h"

#include <xc.h>
#include <libpic30++.h>

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "clocking.h"
#include "color.h"
#include "console.h"
#include "led.h"
#include "serial.h"
#include "ecan_lib.h"
#include "bus_interface.h"
#include "bus_objects.h"
#include "goals.h"
#include "goal_manager.h"
#include "i2c.h"
#include "routing.h"
#include "srf08.h"
#include "gpio.h"
#include "wait.h"
#include "menu.h"
#include "servos.h"
#include "simulator_interface.h"
#include "simple_var_output.h"

#if defined(MICROBRAIN_BOARD_2013)
#pragma config FPWRT = PWR1   // Power-on timer: Disabled
#pragma config ALTI2C = OFF   // I2C mapped to SDA1/SCL1 pins
#pragma config ICS = PGD2     // Communicate on PGC2/EMUC2 and PGD2/EMUD2
#pragma config PWMPIN = ON    // PWM module pins controlled by I/O module at device Reset
#elif defined(MICROBRAIN_BOARD_2015)
#pragma config ALTI2C1 = OFF  // No I2C1
#pragma config ALTI2C2 = ON   // I2C2 mapped to ASDA2/ASCL2 pins
#pragma config ICS = PGD1     // Communicate on PGC1/EMUC1 and PGD1/EMUD1
#pragma config PWMLOCK = OFF  // Don't protect PWM registers
#endif

#pragma config FNOSC = FRC    // Internal Fast RC (FRC)
#pragma config FCKSM = CSECMD // Clock switching is enabled, Fail-Safe Clock Monitor is disabled
#pragma config OSCIOFNC = ON  // OSC2 pin has digital I/O function
//#pragma config IOL1WAY = OFF  // Allow multiple PPS reconfigurations
#pragma config JTAGEN = OFF   // JTAG is Disabled
#pragma config FWDTEN = OFF   // Watchdog timer controlled by user software

color_t color = BLUE;

int start_time; // istante di inizio gara
bool game_started = false;

volatile int game_timer;

static void init_timer(void)
{
    T2CONbits.TON = 0; // disabilito il timer2

    T2CONbits.TSIDL = 0;

    T2CONbits.T32 = 0; //32 bit mode off
    T2CONbits.TCS = 0;
    T2CONbits.TGATE = 0;

    T2CONbits.TCKPS = 0b11; // 1:256 prescaler

    TMR2 = 0;

    PR2 = 37780; // periodo_timer(s)*((Fosc)/(2*prescaler))

    IPC1bits.T2IP = 1; // priorita' a 1
    IEC0bits.T2IE = 1; // abilito Timer2 interrupt
    IFS0bits.T2IF = 0; // reset del dell' interrupt flag

    T2CONbits.TON = 1; // abilito il timer2

}

static void start_game_timer(unsigned int elapsed_time)
{
    start_time = game_timer - elapsed_time; // memorizzo istante inizio gara
    game_started = true;
}


extern "C" void __attribute__((__interrupt__, __auto_psv__, __shadow__)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // reset del dell' interrupt flag
    ++game_timer;
    led_toggle();
    if (game_started && game_timer - start_time > END_GAME) { // trascorsi 90 secondi - fine gara
	IEC0bits.T2IE = 0; // disattiva ulteriori interrupt del Timer2

	motion_stop();
	motion_stop();  // il primo potrebbe fallire

	servos_off();

	printf("\nFINE GARA\n\n");

	FunnyAction();
	printf ("\nFUNNY ACTION\n\n");

        for (;;);
    }
}

void initialize_peripherals(void) {
    init_clock();

    IEC0 = 0; // DISABLE ALL USER INTERRUPT
    IEC1 = 0;
    IEC2 = 0;
    IEC3 = 0;
    IEC4 = 0;

    RCONbits.SWDTEN = 0; // disable Watchdog Timer

    // tutte le porte abilitate in digitale, inizializzazione can
#if defined(MICROBRAIN_BOARD_2013)
    AD1PCFGL = 0xFFFF;
    ecan_initialize(6, 5, true, true);
#elif defined(MICROBRAIN_BOARD_2015)
    ANSELA = ANSELB = 0;
    ecan_initialize(10, 9, true, true);
#endif

    serial2_start(0, B115200);
    //__delay_ms(500);
    gpio_init();
    //i2c_setup();
    init_timer();

    set_led();

    init_can_objects();

    reset_obstacle_avoidance();
    //srf08_initialize();
}

void tune_servo_position(int num) {
    char c;
    int value = 10;
    int on_off = 0;

    printf("-, +   --> decr/incr by 10\n\r");
    printf("1, 2   --> decr/incr by 100\n\r");
    printf("SPACE  --> on/off\n\r");
    printf("ESC    --> End\n\r");
    printf("\n\r");

    for (;;) {
        printf("ON_OFF = %d    ---- Value = %5d\r", on_off, value);
        if (on_off)
            set_servo_position(num, value);
        else
            set_servo_position(num, 0);
        c = read_char();
        switch (c) {
            case 0x1b:
                set_servo_position(num, 0);
                printf("\n\r\n\r");
                return;
            case ' ':
                on_off = 1 - on_off;
                break;
            case '1':
                value -= 100;
                break;
            case '2':
                value += 100;
                break;
            case '-':
                value -= 10;
                break;
            case '+':
                value += 10;
                break;
        }
    }

}

/*
static bool wait_starter(bool wait_in)
{
	if (wait_in)
		set_status_display(CAN_ROBOT_STATUS_UPDATE_STATUS_WAIT_STARTER_IN);
	else
		set_status_display(CAN_ROBOT_STATUS_UPDATE_STATUS_WAIT_STARTER_OUT);
	
	while (STARTER != (wait_in ? 1 : 0))
	{
		unsigned char cmd, flags;
		unsigned int elapsed_time;
		if (receive_strategy_command(&cmd, &flags, &elapsed_time) && cmd == STRATEGY_COMMAND_DISABLE_STARTER)
		{
			color = ((flags & STRATEGY_FLAG_COLOR) == 0) ? YELLOW : BLUE;
			printf("\nRicevuto comando (color=%s): DISABLE_STARTER\n", (color == YELLOW ? "YELLOW" : "BLUE"));
			set_status_display(CAN_ROBOT_STATUS_UPDATE_STATUS_IDLE);
			return false;
		}
	}

	return true;
}
*/

static void game(unsigned int elapsed_time)
{
	set_status_display(CAN_ROBOT_STATUS_UPDATE_STATUS_RUNNING);
	printf("game started! Color: %s\n", color == YELLOW ? "YELLOW" : "BLUE");
	start_game_timer(elapsed_time);
	runGoalSystem();
	while(1);
}

static void do_set_k(int argc, const char **argv)
{
    set_k(atof(argv[1]), atoi(argv[2]));
}

static void do_set_pid(int argc, const char **argv)
{
    speed_pid_parameters(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), 0, 0);
}

static void do_stop(int argc, const char **argv)
{
    motion_stop();
}

static void do_pwm(int argc, const char **argv)
{
    int left, right, r;
    left = atoi(argv[1]);
    right = atoi(argv[2]);
    r = set_pwm(left, right);
    if (r == 1)
        puts("OK");
    else
        puts("NOOK");
}

static void do_radius(int argc, const char **argv)
{
    int r;
    r = set_wheel_radius(atof(argv[1]), atof(argv[2]));
    if (r == 1)
        puts("OK");
    else
        puts("NOOK");
}

static void do_wd(int argc, const char **argv)
{
    int r;
    r = set_wheel_distance(atof(argv[1]));
    if (r == 1)
        puts("OK");
    else
        puts("NOOK");
}

static void do_speed(int argc, const char **argv)
{
    int left, right, r;
    left = atoi(argv[1]);
    right = atoi(argv[2]);
    r = set_speed(left, right);
    if (r == 1)
        puts("OK");
    else
        puts("NOOK");
}

static void do_servo(int argc, const char **argv)
{
    int num, val, r;
    num = atoi(argv[1]);
    val = atoi(argv[2]);
    r = set_servo_position(num, val);
    if (r == 1)
        puts("OK");
    else
        puts("NOOK");
}

static void do_servo_tune(int argc, const char **argv)
{
    int num;
    num = atoi(argv[1]);
    tune_servo_position(num);
}


static void do_ax12(int argc, const char **argv)
{
    int num, val, r;
    num = atoi(argv[1]);
    val = atoi(argv[2]);
    r = set_ax12_servo_position(num, val);
    if (r == 1)
        puts("OK");
    else
        puts("NOOK");
}


static void do_pos(int argc, const char **argv)
{
    ecan_update_object(ROBOT_POSITION_OBJECT);
    printf("(%f, %f, %f)\n", (double)robot_position.x, (double)robot_position.y, (double)robot_position.theta);
#if defined(ROBOT_GRANDE)
    printf("left_bumpers=%d, right_bumpers=%d\n", robot_position.left_bumpers, robot_position.right_bumpers);
#else
    printf("back_bumpers=%d, front_bumpers=%d\n", robot_position.back_bumpers, robot_position.front_bumpers);
#endif
    printf("path done=%d, motor locked=%d, position_valid=%d\n", robot_position.path_done,
        robot_position.block_detection, robot_position.position_valid);
}

static void do_set_pos(int argc, const char **argv)
{
    int r;
    r = set_position(Point(atoi(argv[1]), atoi(argv[2])), atof(argv[3]));
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_invalidate_pos(int argc, const char **argv)
{
    int r;
    r = set_position_valid_flag(false);
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_fwd(int argc, const char **argv)
{
    int r;
    r = forward_to_distance(atoi(argv[1]));
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_rotr(int argc, const char **argv)
{
    int r;
    r = rotate_relative(atoi(argv[1]));
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_rota(int argc, const char **argv)
{
    int r;
    r = rotate_absolute(atoi(argv[1]));
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_rotc(int argc, const char **argv)
{
    int r;
    r = rotate_circular(atoi(argv[1]), atoi(argv[2]));
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_go(int argc, const char **argv)
{
    int r;
    r = forward_to_point(Point(atoi(argv[1]), atoi(argv[2])));
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_go_retry(int argc, const char **argv)
{
    forward_to_point_with_retry(Point(atoi(argv[1]), atoi(argv[2])));
}

static void do_lp(int argc, const char **argv)
{
    int r;
    r = line_to_point(Point(atoi(argv[1]), atoi(argv[2])));
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_lpb(int argc, const char **argv)
{
    int r;
    r = line_to_point(Point(atoi(argv[1]), atoi(argv[2])), true);
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_hdg(int argc, const char **argv)
{
    int r;
    r = heading_to(Point(atoi(argv[1]), atoi(argv[2])), HEADING_FRONT);
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_hdgr(int argc, const char **argv)
{
    int r;
    r = heading_to(Point(atoi(argv[1]), atoi(argv[2])), HEADING_RIGHT);
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_hdgb(int argc, const char **argv)
{
    int r;
    r = heading_to(Point(atoi(argv[1]), atoi(argv[2])), HEADING_BACK);
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_hdgl(int argc, const char **argv)
{
    int r;
    r = heading_to(Point(atoi(argv[1]), atoi(argv[2])), HEADING_LEFT);
    if (r == 1) puts("OK");
    else puts("NOOK");
}

static void do_cc(int argc, const char **argv)
{
	if (color == YELLOW)
	{
		color = BLUE;
		puts("Now Color Is: BLUE");
	}
	else
	{
		color = YELLOW;
		puts("Now Color Is: YELLOW");
	}
}

static void do_dpxy(int argc, const char **argv)
{
	doPathXY(Point(atoi(argv[1]), atoi(argv[2])));
}

static void do_dpl(int argc, const char **argv)
{
	GraphNode *n = findNodeByName(argv[1]);
	if (n != NULL)
	{
		if (doPath(n) != IMPOSSIBLE)
			unchecked_wait(MotionEvent());
	}
	else
	{
		printf("Node not found\n");
	}
}

static void do_cpl(int argc, const char **argv)
{
	GraphNode *n = findNodeByName(argv[1]);
	if (n != NULL)
		n->closeAllEdges();
	else
		printf("Node not found\n");
}

static void autocomplete_dpl_and_cpl(char *buffer, int argidx)
{
	// Generatore che restituisce i nomi di tutti i nodi dotati di nome
	class NodeNameCompletionGenerator : public CompletionGenerator
	{
		public:
			void rewind()
			{
				it.rewind();
			}

			const char* next()
			{
				for (GraphNode *r; (r = it.next()) != NULL; )
				{
					if (r->name != NULL)
						return r->name;
				}
				return NULL;
			}

		private:
			GraphNodeIterator it;
	};

	NodeNameCompletionGenerator nodeIterator;
	autocomplete(buffer, &nodeIterator);
}

static void do_cd(int argc, const char **argv)
{
	closePointsInDirection(atof(argv[1]));
}

static void do_op(int argc, const char **argv)
{
	openAllEdges();
}

static void do_points(int argc, const char **argv)
{
	showNodes();
}

static void do_sched_all(int argc, const char **argv)
{
	start_game_timer(0);
	runGoalSystem();
}

static void do_goal(int argc, const char **argv)
{
	Goal *g = findGoalByName(argv[1]);
	if (g != NULL)
		g->run();
	else
		printf("Goal not found\n");
}

static void autocomplete_goal(char *buffer, int argidx)
{
	// Generatore che restituisce i nomi di tutti i goal
	class GoalNameCompletionGenerator : public CompletionGenerator
	{
		public:
			void rewind()
			{
				it.rewind();
			}

			const char* next()
			{
				Goal *r = it.next();
				return r ? r->name() : NULL;
			}

		private:
			GoalIterator it;
	};

	GoalNameCompletionGenerator goalIterator;
	autocomplete(buffer, &goalIterator);
}

static void do_obstcheck(int argc, const char **argv)
{
	double angle;
	get_pos(&angle);
	angle += argv[1][0] == 'f' ? 0 : 180;

	if (check_obstacle_direction(&angle))
		printf("Trovato ostacolo: %f\n", (double)angle);
	else
		printf("Nessun ostacolo\n");
}

static void do_obstnearonly(int argc, const char **argv)
{
	bool v = argv[1][0] == '0' ? false : true;
	set_obstacle_avoidance_nearonly(v);
}

static void do_unlock(int argc, const char **argv)
{
	double obstdir;

	if (check_obstacle_detected(&obstdir))
		printf("Sbloccato (l'ostacolo era in direzione %f)\n", obstdir);
	else
		printf("Non ero bloccato\n");
}

static void do_simcheck(int argc, const char **argv)
{
	printf("In esecuzione su: ");

	if (in_simulator())
		printf("Simulatore\n");
	else
		printf("Hardware reale\n");
}

#if 0
static void do_sonar(int argc, const char **argv)
{
	srf08_start_ranging(RANGE_MODE_MICROSECS);
	srf08_set_filter(1); // 1 = no soppressione rumore

	while (!kbhit() || toupper(read_char()) != 'Q')
	{
		float v;
		__delay_ms(100);
		if (srf08_check_ranging_complete())
		{
			if (srf08_read_range(&v))
				printf("%f -> %f cm\n", v, v * 10 / 7);
			else
				printf("ERROR\n");

			srf08_start_ranging(RANGE_MODE_MICROSECS);
		}
	}
}
#endif

static void do_help(int argc, const char **argv);

static const menu_command_t comandi_condivisi[] =
{
    { "help", "Show this message", do_help, 1 },
    { "stop", "Stop robot", do_stop, 1 },
    { "pwm <left> <right>", "Set wheel pwm", do_pwm, 3 },
    { "set_k", "Configure K", do_set_k, 3 },
    { "set_pid KPm KPe KIm KIe", "Configure PID", do_set_pid, 5 },
    { "radius <left> <right>", "Set wheel radius", do_radius, 3 },
    { "wd <d>", "Set wheel distance", do_wd, 2 },
    { "speed <left> <right>", "Set wheel speed", do_speed, 3 },
    { "ax12 <servo_num> <value>", "Set AX12 servo position", do_ax12, 3 },
    { "servo <servo_num> <value>", "Set servo position", do_servo, 3 },
    { "servo_tune <servo_num>", "Tune servo position", do_servo_tune, 2 },
    { "pos", "Get robot position", do_pos, 1 },
    { "set_pos <x> <y> <theta>", "Set robot position", do_set_pos, 4 },
    { "invalidate_pos", "Invalidate robot position", do_invalidate_pos, 1 },
    { "fwd <dist>", "Forward to distance", do_fwd, 2 },
    { "rotr <deg>", "Rotate relative", do_rotr, 2 },
    { "rota <deg>", "Rotate absolute", do_rota, 2 },
    { "rotc <deg> <relative_x>", "Rotate circular", do_rotc, 3 },
    { "go <x> <y>", "Go to point", do_go, 3 },
    { "go_retry <x> <y>", "Go to point with retry", do_go_retry, 3 },
    { "lp <x> <y>", "Line to point (in avanti)", do_lp, 3 },
    { "lpb <x> <y>", "Line to point (all'indietro)", do_lpb, 3 },
    { "hdg <x> <y>", "Heading to", do_hdg, 3 },
    { "hdgr <x> <y>", "Heading to with right", do_hdgr, 3 },
    { "hdgb <x> <y>", "Heading to with back", do_hdgb, 3 },
    { "hdgl <x> <y>", "Heading to with left", do_hdgl, 3 },
    { "cc", "Change color", do_cc, 1 },
    { "dpxy <x> <y>", "Execute a path to given point", do_dpxy, 3 },
    { "dpl <label>", "Execute a path to given node", do_dpl, 2, autocomplete_dpl_and_cpl },
    { "cpl <p>", "Close point", do_cpl, 2, autocomplete_dpl_and_cpl },
    { "cd <dir>", "Close direction", do_cd, 2 },
    { "op", "Open all points", do_op, 1 },
    { "points", "Show info about graph points", do_points, 1 },
    { "sched_all", "Schedule all goals", do_sched_all, 1 },
    { "goal <name>", "Run goal <name>", do_goal, 2, autocomplete_goal },
    { "obstcheck <fwd|rev>", "Controlla presenza ostacoli davanti o dietro", do_obstcheck, 2, AUTOCOMPLETE_LIST("fwd", "rev", NULL) },
    { "obstnearonly <0|1>", "Attiva/disattiva filtro ostacoli rilevati", do_obstnearonly, 2, AUTOCOMPLETE_LIST("0", "1", NULL) },
    { "unlock", "Rimuove blocco imposto da obstacle avoidance", do_unlock, 1 },
    { "simcheck", "Controlla se siamo all'interno del simulatore", do_simcheck, 1 },
    //{ "sonar", "Leggi sensore distanza", do_sonar, 1 }
};

static const int num_comandi_condivisi = sizeof(comandi_condivisi) / sizeof(comandi_condivisi[0]);

static void do_help(int argc, const char **argv)
{
	static const char help_fmt[] = "%-30s %s\n";
	printf("== COMANDI CONDIVISI ==\n");
	for (int i = 0; i < num_comandi_condivisi; i++)
		printf(help_fmt, comandi_condivisi[i].nome_comando, comandi_condivisi[i].help_text);
	printf("\n== COMANDI ROBOT ==\n");
	for (int i = 0; i < num_comandi_robot; i++)
		printf(help_fmt, comandi_robot[i].nome_comando, comandi_robot[i].help_text);
}

static const menu_command_t* find_command(const char *name)
{
	// Cerca tra comandi_condivisi
	for (int i = 0; i < num_comandi_condivisi; i++)
	{
		// Confronta stringhe fino al primo spazio
		const char *sA = comandi_condivisi[i].nome_comando;
		const char *sB = name;
		while (*sA && *sA == *sB)
			sA++, sB++;

		const bool nome_match = ((*sA == ' ' || *sA == '\0') && *sB == '\0');
		if (nome_match)
			return &comandi_condivisi[i];
	}

	// Cerca tra comandi_robot
	for (int i = 0; i < num_comandi_robot; i++)
	{
		// Confronta stringhe fino al primo spazio
		const char *sA = comandi_robot[i].nome_comando;
		const char *sB = name;
		while (*sA && *sA == *sB)
			sA++, sB++;

		const bool nome_match = ((*sA == ' ' || *sA == '\0') && *sB == '\0');
		if (nome_match)
			return &comandi_robot[i];
	}

	// Comando non trovato
	return NULL;
}

int main(void)
{
	char line[90];

	initialize_peripherals();

	for (int i = 0; i < 10; i++)
	{
		__delay_ms(200);
		led_on();
		__delay_ms(200);
		led_off();
	}

	printf("Starting\n");

	setup_geometry_and_controllers();
	openAllEdges();
	resetAllGoals();
	//servo_reset();

        color = (COLOR_SEL) ? YELLOW : BLUE;
#if defined(ROBOT_GRANDE)
        GPIO_1_DIR = 0;
        if (color == BLUE)
            RGPIO_1 = 1;
        else
            RGPIO_1 = 0;
#endif

        if (STARTER == 1) {
            align();
            while (STARTER == 1) {}
            game(0);
            for (;;) {}
        }

	for (;;)
	{
		const char *argv[20];
		int argc;

		// immissione della stringa da parte dell'utente
		begin_read_line(line);
		do
		{
			while (!kbhit())
			{
				unsigned char cmd, flags;
				unsigned int elapsed_time;

                                color = (COLOR_SEL) ? YELLOW : BLUE;

#if defined(ROBOT_GRANDE)
                                if (color == BLUE)
                                    RGPIO_1 = 1;
                                else
                                    RGPIO_1 = 0;
#endif
				simulator_relax();
			}
		}
		while (!continue_read_line());

		// separazione sottostringhe per ciascun parametro
		argc = 0;
		argv[0] = strtok(line, " ");
		argc++;
		while ((argv[argc] = strtok(NULL, " ")) != NULL) argc++;

		if (strlen(argv[0]) == 0)
			continue;

		const menu_command_t *command = find_command(argv[0]);
		if (command != NULL && argc == command->num_args)
			command->handler(argc, argv);
		else
			puts("ERROR");
	}
}

static void autocomplete_command(char *buffer)
{
	// Generatore che restituisce i nomi di tutti i comandi
	class CommandNameCompletionGenerator : public CompletionGenerator
	{
		public:
			void rewind()
			{
				source = SHARED;
				idx = -1;
			}

			const char* next()
			{
				idx++;

				if (source == SHARED && idx == num_comandi_condivisi)
				{
					source = ROBOT;
					idx = 0;
				}

				if (source == ROBOT && idx == num_comandi_robot)
				{
					idx = num_comandi_robot - 1;
					return NULL;
				}

				if (source == SHARED)
					return comandi_condivisi[idx].nome_comando;
				else
					return comandi_robot[idx].nome_comando;
			}

		private:
			enum { SHARED, ROBOT } source;
			int idx;
	};

	CommandNameCompletionGenerator commandIterator;
	autocomplete(buffer, &commandIterator);
}

void autocompletion_callback(char *linebuffer)
{
	char *command_name_start, *command_name_end;

	// salta spazi iniziali
	while (*linebuffer == ' ') linebuffer++;

	// identifica nome comando, se presente
	command_name_start = linebuffer;
	while (*linebuffer != ' ' && *linebuffer != '\0') linebuffer++;
	command_name_end = linebuffer;

	if (*command_name_end == '\0') // dobbiamo completare il nome comando
	{
		autocomplete_command(command_name_start);
		return;
	}

	// se siamo arrivati qui vuol dire che l'utente vuole completare un
	// parametro. identifichiamo il comando (i.e. argv[0])
	*command_name_end = '\0';
	const menu_command_t *command = find_command(command_name_start);
	*command_name_end = ' ';

	if (command == NULL || !command->autocomplete.isValid())
		return; // comando non trovato oppure non dotato di completamento automatico

	for (int argidx = 1; argidx < command->num_args; argidx++)
	{
		char *arg_value_start;

		// salta spazi da parametro precedente
		while (*linebuffer == ' ') linebuffer++;

		// identifica eventuale valore già presente
		arg_value_start = linebuffer;
		while (*linebuffer != ' ' && *linebuffer != '\0') linebuffer++;

		if (*linebuffer == '\0') // dobbiamo completare l'ultimo parametro
		{
			command->autocomplete(arg_value_start, argidx);
			return;
		}
	}
}
