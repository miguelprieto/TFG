/*
 * gpio.h
 */

#ifndef __GPIO_H
#define __GPIO_H

#define GPIO_1             LATAbits.LATA0
#define GPIO_2             LATAbits.LATA1
#define GPIO_3             LATBbits.LATB0
#define GPIO_4             LATBbits.LATB1
#define GPIO_5             LATBbits.LATB2
#define GPIO_6             LATBbits.LATB3
#define GPIO_7             LATAbits.LATA2
#define GPIO_8             LATAbits.LATA3
#define GPIO_9             LATBbits.LATB4
#define GPIO_10            LATBbits.LATB15
#define GPIO_11            LATBbits.LATB14
#define GPIO_12            LATBbits.LATB13

#define RGPIO_1             PORTAbits.RA0
#define RGPIO_2             PORTAbits.RA1
#define RGPIO_3             PORTBbits.RB0
#define RGPIO_4             PORTBbits.RB1
#define RGPIO_5             PORTBbits.RB2
#define RGPIO_6             PORTBbits.RB3
#define RGPIO_7             PORTAbits.RA2
#define RGPIO_8             PORTAbits.RA3
#define RGPIO_9             PORTBbits.RB4
#define RGPIO_10            PORTBbits.RB15
#define RGPIO_11            PORTBbits.RB14
#define RGPIO_12            PORTBbits.RB13

#define GPIO_1_DIR         TRISAbits.TRISA0
#define GPIO_2_DIR         TRISAbits.TRISA1
#define GPIO_3_DIR         TRISBbits.TRISB0
#define GPIO_4_DIR         TRISBbits.TRISB1
#define GPIO_5_DIR         TRISBbits.TRISB2
#define GPIO_6_DIR         TRISBbits.TRISB3
#define GPIO_7_DIR         TRISAbits.TRISA2
#define GPIO_8_DIR         TRISAbits.TRISA3
#define GPIO_9_DIR         TRISBbits.TRISB4
#define GPIO_10_DIR        TRISBbits.TRISB15
#define GPIO_11_DIR        TRISBbits.TRISB14
#define GPIO_12_DIR        TRISBbits.TRISB13

#define COLOR_SEL          RGPIO_4
#define COLOR_SEL_DIR      GPIO_4_DIR
#define STARTER            (!RGPIO_6)
#define STARTER_DIR        GPIO_6_DIR

void gpio_init(void);
void sucker_front(int v);
void sucker_rear(int v);


/*
 * Connessioni scheda sensori   ----> MICROBRAIN
 *       S1 = blue              ----> S3 = RB0
 *       S2 = pink              ----> S5 = RB2
 *       S3 = yellow            ----> S7 = RA2
 *       S4 = green             ----> S8 = RA3
 */

#define OBSTACLE_FRONT_DIR    TRISBbits.TRISB0
#define OBSTACLE_REAR_DIR     TRISBbits.TRISB2
#define OBSTACLE_FRONT        PORTBbits.RB0
#define OBSTACLE_REAR         PORTBbits.RB2

#define OMRON_LEFT_DIR    TRISAbits.TRISA3
#define OMRON_RIGHT_DIR   TRISAbits.TRISA2
#define OMRON_LEFT        PORTAbits.RA3
#define OMRON_RIGHT       PORTAbits.RA2

#endif
