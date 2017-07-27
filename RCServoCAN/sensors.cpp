/*
 * sensors.cpp
 */

#include "defines.h"
#include <p33FJ128MC802.h>
#include "sensors.h"


#define PORT_SENSOR_1     PORTBbits.RB2
#define PORT_SENSOR_2     PORTAbits.RA4

#define DIR_SENSOR_1      TRISBbits.TRISB2
#define DIR_SENSOR_2      TRISAbits.TRISA4

#define CN_SENSOR_1       CNEN1bits.CN6IE
#define CN_SENSOR_2       CNEN1bits.CN0IE


t_sensor sensor1, sensor2;


#define CHECK_SENSOR_INTERRUPT(var,port) {      \
        if (var.type == SENSOR_INTERRUPT) {     \
            if (port == var.edge) var.value = true;     \
        }                                               \
    }


#define SETUP_SENSOR(var,t,e,dir,cn) {          \
        dir = 1;                                \
        var.type = t;                           \
        var.edge = e;                           \
        var.value = false;                       \
        if (t == SENSOR_INTERRUPT)              \
            cn = 1;                             \
    }




extern "C" void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void)
{
    CHECK_SENSOR_INTERRUPT(sensor1, PORT_SENSOR_1);
    CHECK_SENSOR_INTERRUPT(sensor2, PORT_SENSOR_2);

    IFS1bits.CNIF = 0;
}


void init_sensors(t_sensor_type s1_type, t_sensor_edge s1_edge, t_sensor_type s2_type, t_sensor_edge s2_edge)
{
    CNPU1 = 0;
    CNPU2 = 0;
    SETUP_SENSOR(sensor1, s1_type, s1_edge, DIR_SENSOR_1, CN_SENSOR_1);
    SETUP_SENSOR(sensor2, s2_type, s2_edge, DIR_SENSOR_2, CN_SENSOR_2);

    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;

    IEC1bits.CNIE = 1;
    IFS1bits.CNIF = 0;
}


#define READ(num,var,port)                      \
    bool read_sensor##num(void) {               \
        if (var.type == SENSOR_INTERRUPT) {     \
            bool ret_val = var.value;           \
            if (ret_val)                        \
                var.value = false;              \
            return ret_val;                     \
        }                                       \
        else                                    \
            return port == 0;                   \
    }


#define CLEAR(num,var)                      \
    void clear_sensor##num(void) {              \
        if (var.type == SENSOR_INTERRUPT) {     \
            var.value = false;                  \
        }                                       \
    }


READ(1,sensor1,PORT_SENSOR_1);
READ(2,sensor2,PORT_SENSOR_2);

CLEAR(1,sensor1);
CLEAR(2,sensor2);

