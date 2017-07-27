/*
 * main.cpp
 */

#include "defines.h"

#include <p33FJ128MC802.h>
#include <libpic30++.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "clocking.h"
#include "serial.h"
#include "led.h"
#include "ax12.h"
#include "timers.h"

_FPOR(PWMPIN_ON & FPWRT_PWR1 & ALTI2C_OFF);

_FICD(ICS_PGD1 & JTAGEN_OFF);

_FWDT(FWDTEN_OFF);

UART_1 uart1;
UART_2 console;
Dynamixel ax12(uart1);

#define ENABLE_485() { LATBbits.LATB7 = 1; }
#define DISABLE_485() { LATBbits.LATB7 = 0; }


extern "C"  int    write(int handle, void *buffer, unsigned int len)
{
  int i;
  char * buf = (char*)buffer;
   switch (handle)
  {
      case 0:        // handle 0 corresponds to stdout
      case 1:        // handle 1 corresponds to stdin
      case 2:        // handle 2 corresponds to stderr
      default:
          ENABLE_485();
          for (i=0; i<len; i++) {
              char c = *buf;
              buf++;
              console.put_char_no_fifo(c);
          }
          __delay_ms(12);
          DISABLE_485();
  }
  return(len);
}

extern "C"  int  fputc(int c, FILE * f)
{
    ENABLE_485();
    console.put_char_no_fifo(c);
    __delay_ms(12);
    DISABLE_485();
}

void initialize_peripherals()
{
    init_clock();

    IEC0 = 0;				// DISABLE ALL USER INTERRUPT
    IEC1 = 0;				
    IEC2 = 0;
    IEC3 = 0;	
    IEC4 = 0;	

    AD1PCFGL = 0x1ff; // all input ports in digital mode (analog disabled)

    RCONbits.SWDTEN = 0; // disable Watchdog Timer

    set_led();

    // ENABLE = pin 16, RB7
    // TX = pin 22, RP11
    // RX = pin 21, RP10
    console.open(B9600, 11, 10, true, false, false);
    TRISBbits.TRISB7 = 0;
    ENABLE_485();
}


void flashing_at_startup()
{
    led_on();
    for (int i = 0;i < 10;i++) {
        led_toggle();
        __delay_ms(100);
    }
    led_off();
}


void change_speed_from_1M_to_57600()
{
    led_on();
    ax12.setup(AX12_PIC_SPEED_1M, 8, 9, true, false, true);
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
    ax12.setup(AX12_PIC_SPEED_57600, 8, 9, true, false, true);
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
    ax12.setup(from, 8, 9, true, false, true);
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


static void do_flash_ax12_led(int argc, const char **argv)
{
    int i;
    for (i = 0;i < 2;i ++) {
        ax12.set_servo_led(0xfe, true);
        __delay_ms(500);
        ax12.set_servo_led(0xfe, false);
        __delay_ms(500);
    }
    ax12.set_servo_led(0xfe, true);
}


static void do_set_ax12_speed(int argc, const char **argv)
{
    if (!strcmp(argv[1],"1m")) ax12.setup_uart_speed(AX12_PIC_SPEED_1M);
    else if (!strcmp(argv[1],"58k")) ax12.setup_uart_speed(AX12_PIC_SPEED_58K);
    else if (!strcmp(argv[1],"57k")) ax12.setup_uart_speed(AX12_PIC_SPEED_57600);
    else if (!strcmp(argv[1],"20k")) ax12.setup_uart_speed(AX12_PIC_SPEED_20408);
    else if (!strcmp(argv[1],"13k")) ax12.setup_uart_speed(AX12_PIC_SPEED_13071);
    else
        puts("speed error\r");
}

static void do_read_ax12(int argc, const char **argv)
{
    uint8_t value;
    uint8_t id = atoi(argv[1]);
    int reg = atoi(argv[2]);
    t_dymx_error err;
    if ((err = ax12.read_register_8(id, reg, value)) == ERR_NO_ERROR) {
        printf("Register 0x%02x, value = %d\n\r", reg, value);
    }
    else {
        printf("Communication error %02x\n\r", err);
    }
}

static void do_write_ax12(int argc, const char **argv)
{
    uint8_t id = atoi(argv[1]);
    int reg = atoi(argv[2]);
    uint8_t value = atoi(argv[3]);
    t_dymx_error err;
    if ((err = ax12.write_register_8(id, reg, value)) == ERR_NO_ERROR) {
        printf("Register 0x%02x, value = %d\n\r", reg, value);
    }
    else {
        printf("Communication error %02x\n\r", err);
    }
}

static void do_write16_ax12(int argc, const char **argv)
{
    uint8_t id = atoi(argv[1]);
    int reg = atoi(argv[2]);
    uint16_t value = atoi(argv[3]);
    t_dymx_error err;
    if ((err = ax12.write_register_16(id, reg, value)) == ERR_NO_ERROR) {
        printf("Register 0x%02x, value = %d\n\r", reg, value);
    }
    else {
        printf("Communication error %02x\n\r", err);
    }
}

static void do_set_pos(int argc, const char **argv)
{
    uint8_t id = atoi(argv[1]);
    int pos = atof(argv[2]);
    t_dymx_error err;
    if ((err = ax12.move(id, pos)) == ERR_NO_ERROR) {
        printf("Ok\n\r");
    }
    else {
        printf("Communication error %02x\n\r", err);
    }
}

static void do_torque(int argc, const char **argv)
{
    uint8_t id = atoi(argv[1]);
    int t = atoi(argv[2]);
    t_dymx_error err;
    if ((err = ax12.torque(id, t)) == ERR_NO_ERROR) {
        printf("Ok\n\r");
    }
    else {
        printf("Communication error %02x\n\r", err);
    }
}

static void do_find(int argc, const char **argv)
{
    int i;
    for (i = 1; i < 253;i++) {
        t_dymx_error err;
        uint8_t value;
        if ((err = ax12.read_register_8(i, 3, value)) == ERR_NO_ERROR) {
            printf("Register 3, value = %d\n\r", value);
            return;
        }
    }
}

typedef struct
{
    const char *nome_comando, *help_text;
    void (*handler)(int argc, const char **argv);
    int num_args;
} menu_command_t;


static void do_help(int argc, const char **argv);

static const menu_command_t comandi_condivisi[] =
{
    { "help", "Show this message", do_help, 1 },
    { "speed <1m|58k|57k|20k|13k>", "Set AX12 Speed port", do_set_ax12_speed, 2 },
    { "led", "Flash AX12 Led in broadcast", do_flash_ax12_led, 1 },
    { "read <id> <reg>", "Read AX12 Register", do_read_ax12, 3 },
    { "write <id> <reg> <value>", "Write AX12 8-bit Register", do_write_ax12, 4 },
    { "write16 <id> <reg> <value>", "Write AX12 16-bit Register", do_write16_ax12, 4 },
    { "set_pos <id> <degrees>", "Set angle", do_set_pos, 3 },
    { "torque <id> <0|1>", "Set torque", do_torque, 3 },
    { "find", "Find ID of a servo", do_find, 1 },
};

static const int num_comandi_condivisi = sizeof(comandi_condivisi) / sizeof(comandi_condivisi[0]);

static void do_help(int argc, const char **argv)
{
	static const char help_fmt[] = "%-30s %s\n\r";
	printf("== COMANDI ==\n\r");
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

int main()
{
    int count = 0;

    //__C30_UART = 2;

    initialize_peripherals();
    flashing_at_startup();

    // TXPIN = RB8/RP8
    // RXPIN = RB8/RP9
    // open drain configuration: yes
    // TX interrupt: no
    // RX interrupt: yes
    ax12.setup(AX12_PIC_SPEED_58K, 8, 9, true, false, true);

    printf("\n\r\n\r\n\rSTARTING....\n\r");

    ax12.set_servo_led(0xfe, true);

    for (;;) {
        char line[128];
        const char *argv[20];
        int argc;

        printf("AX12>");
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
            puts("ERROR\r");

    }

    return 0;
}

