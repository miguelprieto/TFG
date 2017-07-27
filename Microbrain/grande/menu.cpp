#include <stdbool.h>

#include "defines.h"
#include <xc.h>
#include <dsp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libpic30++.h>

#include "adc.h"
#include "bus_interface.h"
#include "bus_objects.h"
#include "color.h"
#include "console.h"
#include "controller_presets.h"
#include "goals.h"
#include "gpio.h"
#include "menu.h"
#include "pwm.h"
#include "servos.h"
#include "routing.h"
#include "ecan_lib.h"
#include "wait.h"

static void do_staccastacca(int argc, const char **argv);

void align(void)
{
	// DEBUG
	do_staccastacca (0, NULL);

	puts("Aligning...");

	disable_obstacle_avoidance();

	// Abbassiamo temporaneamente la velocità massima per avere un maggiore
	// controllo, dato che dobbiamo strisciare sul bordo
	AutoControllerParamsRestorer<DistanceControllerParameters> autorestorer_p;
	distance_slow.apply();

	const double align_x = symmXifBlue(DIM_H_BACK);
	const double align_y = 22+160;
	const int align_t = symmTifBlue(0);
	
	// APERTURA PARETIE
	set_servo_position (PAR_SX, PAR_SX_OPEN);
    	__delay_ms(80);
	set_servo_position (PAR_DX, PAR_DX_OPEN);

	// ASCENSORE
	lift_home(0);
    	lift_wait_home();
	puts("Ascensore resettato");

    	__delay_ms(500);

	set_position(Point(align_x, align_y), align_t);
	bump_and_set_x(align_x, align_t);
	unchecked_wait(MotionEvent());

	forward_to_distance (70);
	unchecked_wait(MotionEvent());	

	set_position_valid_flag(true);

	enable_obstacle_avoidance();
	
	// SERVO RESET
	servo_reset();
	puts("Servo resettati");

	lift_go_to(LIFT_RUN);

	__delay_ms(1000);

	// CHIUSURA PARETIE
	set_servo_position (PAR_SX, PAR_SX_CLOSED);
    	__delay_ms(80);
	set_servo_position (PAR_DX, PAR_DX_CLOSED);

	rotate_absolute(symmTifBlue(0));

	puts("Done");


}

void setup_geometry_and_controllers(void)
{
	int r;

        init_adc();

	motion_stop();

	printf("GEOMETRY:");
	r = set_wheel_radius(-20.62, -20.62);
	r = r & set_wheel_distance(232.8);		// MISURA FRA GLI ENCODER
	r = r & set_k(0.0, 0.0);
	if (r == 1) puts("OK");
	else puts("NOOK");

	printf("SPEED PID:");
        r = speed_pid_parameters(20, 0, 120, 0, 0, 0);
	if (r == 1) puts("OK");
	else puts("NOOK");

	printf("Line controller:");
	if (line_default.apply())
		puts("OK");
	else
		puts("NOOK");

	printf("Distance controller:");
	if (distance_default.apply())
		puts("OK");
	else
		puts("NOOK");

	printf("Heading controller:");
	if (heading_default.apply())
		puts("OK");
	else
		puts("NOOK");

	printf("Circular rotation controller:");
	if (circular_rotation_default.apply())
		puts("OK");
	else
		puts("NOOK");
	
	set_servo_position (0, 0);
	set_servo_position (2, 0);
	set_servo_position (7, 0);
	set_servo_position (8, 0);
	set_servo_position (9, 0);
	
	lift_stop (0);

	// printf("Minimal speed:");
	// r = set_minimal_speed(40, 5); // 40 mm/s, 5 deg/s
	// if (r == 1) puts("OK");
	// else puts("NOOK");

	// printf("Error tolerance:");
	// r = set_error_tolerance(3, 0.5);  // 3 mm, 0.5 deg
	// if (r == 1) puts("OK");
	// else puts("NOOK");

	// printf("Anticipaton gain:");
	// r = set_anticipation_gain(0, 0);
	// if (r == 1) puts("OK");
	// else puts("NOOK");

	// printf("Error to minimal speed:");
	// r = set_error_to_minimal_speed(20, 2.5); // 20mm, 2.5 deg
	// if (r == 1) puts("OK");
	// else puts("NOOK");

}


static void do_align(int argc, const char **argv)
{
	align();
}

static void do_servo_reset(int argc, const char **argv)
{
	servo_reset();
}

static void do_servo_off(int argc, const char **argv)
{
	servos_off();
}

static void do_servo_status(int argc, const char **argv)
{
}

static void do_gpio(int argc, const char **argv)
{
    printf("STARTER = %d, COLOR = %d\n", STARTER, COLOR_SEL);
}

static void do_lift(int argc, const char **argv)
{
    if (!strcmp(argv[1], "off"))
        lift_stop(0);
    else if (!strcmp(argv[1], "home")) {
        lift_home(0);
        lift_wait_home();
    }
    else if (!strcmp(argv[1], "pwm"))
        lift_pwm(0, atoi(argv[2]));
    else if (!strcmp(argv[1], "speed"))
        lift_speed(0, atoi(argv[2]));
    else if (!strcmp(argv[1], "tel"))
        send_lift_telemetry_enable(true);
    else if (!strcmp(argv[1], "pos")) {
        printf("LIFT POSITION = %f\n", get_lift_position());
    }
    else if (!strcmp(argv[1], "setpos")) {
        send_lift_position(0, atoi(argv[2]));
    }
    else
        printf("invalid command\n");
}


void do_servo_run(int argc, const char **argv)
{

	__delay_ms(100);
	set_servo_position (UPD_SX, (UPD_SX_DOWN));
	set_servo_position (UPD_DX, (UPD_DX_DOWN));
	__delay_ms(100);
	set_servo_position (BAF_SX, (BAF_SX_UPD));
	set_servo_position (BAF_DX, (BAF_DX_UPD));
	__delay_ms(100);
	set_servo_position (UPD_SX, (UPD_SX_UP));
	set_servo_position (UPD_DX, (UPD_DX_UP));
}


static void do_chiusura(int argc, const char **argv)
{
	int n = atoi(argv[1]);
	int controllo = atoi(argv[2]);

	chiusura(n, controllo);
}


static void do_palle(int argc, const char **argv)
{
    palle();
}

static void do_scarica(int argc, const char **argv)
{

	lift_go_to(LIFT_CASA);
	__delay_ms(10000);

	
	// SCARICO
	set_servo_position(UPD_DX, UPD_DX_DOWN);
	set_servo_position(UPD_SX, UPD_SX_DOWN);
	__delay_ms(100);

	set_servo_position(BAF_DX, BAF_DX_CLOSED+100);
	__delay_ms(60);
	set_servo_position(BAF_SX, BAF_SX_CLOSED-100);
	__delay_ms(100);

	set_servo_position(PAR_SX, PAR_SX_OPEN);
	__delay_ms(60);
	set_servo_position(PAR_DX, PAR_DX_OPEN);
	__delay_ms(250);

	scarica();
} 

static void do_automation(int argc, const char **argv)
{
    set_servo_position(255, 1);

    while (!kbhit()) {
        __delay_ms(500);
        printf("status %d\n\r", automation_busy());
    }

}

static void do_funny(int argc, const char **argv)
{
    FunnyAction();
}


static void do_staccastacca(int argc, const char **argv)
{

	// ASCENSORE
	lift_stop(0);	

	// SERVO
	do_servo_off(0, NULL);

	// MOTORI
	motion_stop();

	// SERVO
	set_servo_position (255, 0);

}



const menu_command_t comandi_robot[] =
{
	{ "align", "Align the robot - entrambe le parti", do_align, 1 },
	{ "servo_reset", "Riporta servo in posizione iniziale", do_servo_reset, 1 },
	{ "servo_off", "Libera i servo", do_servo_off, 1 },
	{ "servo_status", "Print some FSMs' statuses", do_servo_status, 1 },
	{ "gpio", "Leggi starter e selettore", do_gpio, 1 },
	{ "lift", "", do_lift, 3 },
	{ "chiusura", "Run the algorithm *Chiusura*", do_chiusura, 3 },
	{ "palle", "Run the algorithm *Palle*", do_palle, 1 },
	{ "staccastacca", "Power Off everything (Ascensore, Servi e Motori)", do_staccastacca, 1 },
	{ "auto", "", do_automation, 1 },
	{ "scarica", "Run the algorithm *Scarica*", do_scarica, 1},
	{ "funny", "Funny action", do_funny, 1},
};

const int num_comandi_robot = sizeof(comandi_robot) / sizeof(comandi_robot[0]);
