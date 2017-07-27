/*
 * main.cpp
 */
#define L1 75
#define L2 95
#define L3 40

#include "defines.h"

#include <p33FJ128MC802.h>
#include <libpic30++.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "clocking.h"
#include "led.h"
#include "ax12.h"
#include "manipulator.h"
#include "menu.h"

_FPOR(PWMPIN_ON & FPWRT_PWR1 & ALTI2C_ON);

_FICD(ICS_PGD2 & JTAGEN_OFF);

_FWDT(FWDTEN_OFF & WDTPRE_PR128 & WDTPOST_PS2 & WINDIS_OFF);

UART_1 uart1;
UART_2 console;
Dynamixel ax12(uart1);
Manipulator arm(ax12, 4);

void initialize_peripherals()
{
    init_clock();

    IEC0 = 0;				// DISABLE ALL USER INTERRUPT
    IEC1 = 0;				
    IEC2 = 0;
    IEC3 = 0;	
    IEC4 = 0;	

    AD1PCFGL = 0x1ff; // all input ports in digital mode (analog disabled)

    RCONbits.SWDTEN   = 0;	// disable Watchdog Timer

    set_led();

    // TX = RP12
    // RX = RP7
    console.open(B115200, 12, 7, false, false, false);

}


void flashing_at_startup()
{
    led_on();
    for (int i = 0;i < 10;i++) {
        led_toggle();
        __delay_ms(100);
    }
}


void change_speed_from_1M_to_57600()
{
    led_on();
    ax12.setup(AX12_PIC_SPEED_1M);
    __delay_ms(2000);
    ax12.set_servo_led(0xfe, true);
    __delay_ms(2000);
    ax12.set_new_speed(0xfe, AX12_PIC_SPEED_57600);
    __delay_ms(2000);
    ax12.set_servo_led(0xfe, false);
    led_off();
    for (;;) ;
}


void change_speed_from_57600_to_20408()
{
    led_on();
    ax12.setup(AX12_PIC_SPEED_57600);
    __delay_ms(2000);
    ax12.set_servo_led(0xfe, true);
    __delay_ms(2000);
    ax12.set_new_speed(0xfe, AX12_PIC_SPEED_20408);
    __delay_ms(2000);
    ax12.set_servo_led(0xfe, false);
    led_off();
    for (;;) ;
}

void change_speed_from_to(int from, int to)
{
    led_on();
    ax12.setup(from);
    __delay_ms(2000);
    ax12.set_servo_led(0xfe, true);
    __delay_ms(2000);
    ax12.set_new_speed(0xfe, to);
    __delay_ms(2000);
    ax12.set_servo_led(0xfe, false);
    led_off();
    for (;;) ;
}

void servo_led_flash(uint8_t id)
{
    for (;;) {
        ax12.set_servo_led(id, true);
        __delay_ms(1000);
        ax12.set_servo_led(id, false);
        __delay_ms(1000);
    }
}

void servo_test(uint8_t id)
{
    ax12.torque(id, true);
    for (;;) {
        ax12.set_servo_led(id, true);
        ax12.degrees(id, -45);
        __delay_ms(1000);
        ax12.set_servo_led(id, false);
        ax12.degrees(id, 45);
        __delay_ms(1000);
    }
}

void setup_manipulator()
{
    arm.set_joint_id(0, 0x26);
    arm.set_joint_id(1, 0x25);
    arm.set_joint_id(2, 0x13);
    arm.set_joint_id(3, 0x20);
}

void manipulator_test()
{
    arm.set_joint_id(0, 0x26);
    arm.set_joint_id(1, 0x25);
    arm.set_joint_id(2, 0x13);

    float target_angles[] = { 0x0, 0x0, 0x0 };
    arm.linear(target_angles, 10);
    arm.reset();

    while (!arm.step()) {
        __delay_ms(10);
    }

    for (;;) ;
}


static void do_servo(int argc, const char **argv)
{
    ax12.degrees(atoi(argv[1]), atoi(argv[2]));
}

static void do_get_servo(int argc, const char **argv)
{
    float current;
    ax12.current_angle(atoi(argv[1]), current);
    printf("Value = %f\n\r", current);
}

static void do_m_get(int argc, const char **argv)
{
    float current[10];
    arm.reset();
    arm.get_current_angles(current);
    for (int i = 0; i < arm.joints();i++) {
        printf("Joint %d = %f\n\r", i, current[i]);
    }
}

static void do_inv_kinematics(int argc, const char **argv)
{
    float angles [4];
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    float z = atof(argv[3]);
    float pi = 3.141592;
    //angolo di attacco dell' end effector rispetto al terreno
    float a_ee= atof(argv[4]);
    //calcolo il modulo della distanza tra il fulcro del manipolatore ed il punto da raggiungere
    float r = sqrtf((x * x) + (y * y));
    r = r + L3 * cos(a_ee);
    z = z + L3 * sin(a_ee);
    angles[0]= atan2(y,x); //a 0° è dritto
    angles[2]= - acos( (r * r + z * z - L1 * L1 - L2 * L2 ) / (2 * L1 * L2) );
    //printf("angolo 2 in radianti: %f ", angles[2]);
    float c = cos(angles[2]);
    float s = sin(angles[2]);
    angles[2] = (pi/2) + angles[2];
    angles[1] = (pi/2) + asin ( ( (z * ( L1 + L2 * c)) - L2 * s * r) /( ( (L1 + L2 * c) * (L1 + L2 * c)) + ((L2 * s) * (L2 * s))));
    //converto in gradi
    for (int i=0; i<3; i++){
        angles[i] = (angles[i] * 180 / pi);
        printf("joint %f \n", angles[i]);
    }
    angles[3] = 180 - angles[1] - angles[2] - a_ee;
    printf("joint 4 %f \n", angles[3]);
    arm.reset();
    arm.linear(angles, 15);
    while (!arm.step()) ;
}
static void do_m_reset(int argc, const char **argv)
{
    float target_angles[] = { 0, 60, -50, 80 };

    arm.reset();
    arm.linear(target_angles, 15);

    while (!arm.step()) ;
}

static void do_m_pick(int argc, const char **argv)
{
    float target_angles[4][4] = { { 0, 97, 0, 0 },
                                  { 0, 67, 26, 0 },
                                  { -83, 75, -32, 53},
                                  { -83, 83, -21, 29} };

    int ticks = 10;

    arm.reset();
    arm.linear(&target_angles[0][0], ticks);
    while (!arm.step()) ;

    __delay_ms(200);

    arm.linear(&target_angles[1][0], ticks);
    while (!arm.step()) ;

    __delay_ms(500);

    arm.linear(&target_angles[2][0], ticks);
    while (!arm.step()) ;

    arm.linear(&target_angles[3][0], ticks);
    while (!arm.step()) ;

}

// ------------------------------------------------------------------
ArmTypeManipulator main_arm(ax12, 0x26, 0x25, 0x20, 0x13,
                            107, 130, 40);


static void show_arm_position(int argc, const char **argv)
{
    Point3D p;
    float a[4];
    main_arm.direct_kinematics(p);
    main_arm.get_current_angles(a);

    printf("X = %f, Y = %f. Z = %f\n", p.x(), p.y(), p.z());
    printf("a = %f, b = %f. c = %f, d = %f\n", a[0], a[1], a[2], a[3]);
}


static void arm_go_to_position(int argc, const char **argv)
{
    Point3D p(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    float t[4];

    main_arm.inverse_kinematics(p, atof(argv[4]), t);

    printf("%f %f %f %f\n", t[0], t[1], t[2], t[3]);

    main_arm.set_angle(0, t[0]);
    main_arm.set_angle(1, t[1]);
    main_arm.set_angle(2, t[2]);
    main_arm.set_angle(3, t[3]);

}

static void arm_scan(int argc, const char **argv)
{
    Point3D p(50, 0, -160);
    float t[4];
    int i;

    for (i = 0;i < 50;i+=4) {
        p.y(i);
        main_arm.inverse_kinematics(p, -90, t);
        printf("%f %f %f %f\n", t[0], t[1], t[2], t[3]);

        main_arm.set_angle(0, t[0]);
        main_arm.set_angle(1, t[1]);
        main_arm.set_angle(2, t[2]);
        main_arm.set_angle(3, t[3]);
        __delay_ms(10);
    }

    for (i = 50;i >= -50;i-=4) {
        p.y(i);
        main_arm.inverse_kinematics(p, -90, t);
        printf("%f %f %f %f\n", t[0], t[1], t[2], t[3]);

        main_arm.set_angle(0, t[0]);
        main_arm.set_angle(1, t[1]);
        main_arm.set_angle(2, t[2]);
        main_arm.set_angle(3, t[3]);
        __delay_ms(10);
    }

    for (i = -50;i < 0;i+=4) {
        p.y(i);
        main_arm.inverse_kinematics(p, -90, t);
        printf("%f %f %f %f\n", t[0], t[1], t[2], t[3]);

        main_arm.set_angle(0, t[0]);
        main_arm.set_angle(1, t[1]);
        main_arm.set_angle(2, t[2]);
        main_arm.set_angle(3, t[3]);
        __delay_ms(10);
    }

}


// ------------------------------------------------------------------
static void do_help(int argc, const char **argv);

static const menu_command_t comandi_condivisi[] =
{
    { "help", "Show this message", do_help, 1 },
    { "servo <num> <-150/+150>", "Move a servo", do_servo, 3},
    { "get_servo <num>", "Get the current angle of a servo", do_get_servo, 2},
    { "m_get", "Get the current angles of the manipulator", do_m_get, 1},
    { "m_reset", "Reset the manipulator", do_m_reset, 1},
    { "m_pick", "Pick an object", do_m_pick, 1},
    { "pos", "Get current position", show_arm_position, 1},
    { "go", "Go to position", arm_go_to_position, 5},
    { "scan", "Arm scan", arm_scan, 1},
    { "inv_kinematics <x> <y> <z> <ang>", "Operate invers kinematics", do_inv_kinematics, 5}
};

static const int num_comandi_condivisi = sizeof(comandi_condivisi) / sizeof(comandi_condivisi[0]);

static void do_help(int argc, const char **argv)
{
	static const char help_fmt[] = "%-30s %s\n";
	printf("== COMANDI CONDIVISI ==\n");
	for (int i = 0; i < num_comandi_condivisi; i++)
		printf(help_fmt, comandi_condivisi[i].nome_comando, comandi_condivisi[i].help_text);
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

	// Comando non trovato
	return NULL;
}


void menu()
{
    char line[90];

    for (;;) {
        const char *argv[20];
        int argc;


        printf("DYNAMIXEL>");
        console.read_line(line);
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


int main()
{
    __C30_UART = 2;

    initialize_peripherals();

    flashing_at_startup();

    //change_speed_from_to(AX12_PIC_SPEED_1M, AX12_PIC_SPEED_58K);

    ax12.setup(AX12_PIC_SPEED_58K);

    setup_manipulator();

    printf("STARTING....\n\r");

    ax12.set_servo_led(0xfe, true);

    __delay_ms(1000);

    main_arm.set_angle_factor(0, -1);
    main_arm.set_angle_offset(1, 90);
    main_arm.set_angle_factor(2, -1);
    main_arm.set_angle_factor(3, -1);

    menu();


    //ax12.set_new_id(0xfe, 0x30);
    //servo_test(0x20);

    ax12.degrees(0x25, 0);
    ax12.degrees(0x26, 0);
    ax12.degrees(0x13, 0);

    ax12.torque(0x25, true);
    ax12.torque(0x26, true);
    ax12.torque(0x13, true);

    //yellow_led(LED_OFF);

    __delay_ms(1000);

    float sign = -1.0;

    for (;;) {

        int angle;

        //__delay_ms(1000);

        for (angle = 0; angle < 60 ;angle+=5) {
            ax12.degrees(0x25, angle);
            ax12.degrees(0x13, -angle);
            ax12.degrees(0x26, sign * angle);
            __delay_ms(10);
        }

        //__delay_ms(1000);

        for (; angle >= 0 ;angle-=5) {
            ax12.degrees(0x25, angle);
            ax12.degrees(0x13, -angle);
            ax12.degrees(0x26, sign * angle);
            __delay_ms(10);
        }

        sign = - sign;
    }


    return 0;
}

