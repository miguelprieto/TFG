/*
 * sensors.h
 */

#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdbool.h>

typedef enum {
    SENSOR_POLLING = 0,
    SENSOR_INTERRUPT = 1
} t_sensor_type;

typedef enum {
    EDGE_FALLING = 0,
    EDGE_RISING = 1
} t_sensor_edge;

typedef struct {
    t_sensor_type  type;
    t_sensor_edge   edge;
    bool value;
} t_sensor;

void init_sensors(t_sensor_type s1_type, t_sensor_edge s1_edge, t_sensor_type s2_type, t_sensor_edge s2_edge);
bool read_sensor1(void);
bool read_sensor2(void);
void clear_sensor1(void);
void clear_sensor2(void);


#endif
