#include <stdbool.h>

#include "defines.h"
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libpic30++.h>

#include "color.h"
#include "controller_presets.h"
#include "goals.h"
#include "gpio.h"
#include "menu.h"
#include "servos.h"
#include "../wait.h"
#include "routing.h"
#include "bus_interface.h"
#include "bus_objects.h"
#include "ecan_lib.h"
#include "wait.h"
#include "adc.h"
#include "console.h"

static void do_staccastacca(int argc, const char **argv);

void align(void)
{
    do_staccastacca (0, NULL);

    puts("Aligning...");
    disable_obstacle_avoidance();

    const double align_x = symmXifBlue(DIM_H_BACK + 710 + 23);
    const double align_y = 23 + DIM_H_BACK;
    const int align_tx = symmTifBlue(0);
    const int align_ty = 90;

    set_position(Point(align_x, align_y), align_tx);

    bump_and_set_x(align_x, align_tx);
    unchecked_wait(MotionEvent());

    forward_to_distance (120);
    rotate_absolute(symmTifBlue(90));

    bump_and_set_y(align_y, align_ty);
    unchecked_wait(MotionEvent());

    //forward_to_distance (180);
    forward_to_distance(100);
    unchecked_wait(MotionEvent());

    set_position_valid_flag(true);

    //enable_obstacle_avoidance();

    rotate_absolute(symmTifBlue(90));

    puts("Done");
}

void setup_geometry_and_controllers(void)
{
	int r;

        //i2c_setup();
        // set i2c pin as input
        TRISBbits.TRISB8 = 1;
        TRISBbits.TRISB9 = 1;

	motion_stop();

        servos_off();

	printf("GEOMETRY:");
	r = set_wheel_radius(-20.66, -20.66);
	r = r & set_wheel_distance(136.22); //137.47
	r = r & set_k(0.0, 0.0);
	if (r == 1) puts("OK");
	else puts("NOOK");

        // parametri PID copiati da ROCCO (2014)
        // ma KP da 13 e' diventata 18 (la dinamica era un po' troppo lenta...)
        printf("SPEED PID:");
        r = speed_pid_parameters(18, 0, 72, 0, 0, 0);
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

static void do_gpio(int argc, const char **argv)
{
	printf("STARTER = %d, COLOR_SEL = %d, OMRON_LEFT = %d, OMRON_RIGHT = %d, OBSTACLE_FRONT = %d, OBSTACLE_REAR = %d\n",
               STARTER, COLOR_SEL, OMRON_LEFT, OMRON_RIGHT, OBSTACLE_FRONT, OBSTACLE_REAR);
}

static void do_sucker_front(int argc, const char **argv)
{
    sucker_front(atoi(argv[1]));
}


static void do_sucker_rear(int argc, const char **argv)
{
    sucker_rear(atoi(argv[1]));
}

static void do_staccastacca(int argc, const char **argv)
{

    // MOTORI
    motion_stop();

    // SERVO
    servo_reset();

}

static void do_sensors (int argc, const char **argv)
{
    while (!kbhit()) {
        printf("OMRON F=%d, R=%d\n", omron_front_status(), omron_rear_status());
        __delay_ms(100);
    }
}


const menu_command_t comandi_robot[] =
{
	{ "align", "Align the robot", do_align, 1 },
	{ "servo_reset", "Riporta servo in posizione iniziale", do_servo_reset, 1 },
	{ "gpio", "Leggi starter e selettore", do_gpio, 1 },
	{ "sf 0|1", "sucker front", do_sucker_front, 2 },
	{ "sr 0|1", "sucker rear", do_sucker_rear, 2 },
	{ "sensors", "", do_sensors, 1 },
	{ "staccastacca", "", do_staccastacca, 1 },
};

const int num_comandi_robot = sizeof(comandi_robot) / sizeof(comandi_robot[0]);


