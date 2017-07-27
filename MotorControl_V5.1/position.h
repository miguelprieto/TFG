/*
 *		position.h
 */


#ifndef __POSITION_H
#define __POSITION_H

#include <stdbool.h>
#include "controller.h"
#include "timers.h"
#include "velocity.h"
#include "odometry.h"

//#define P_CHECK_TIME		8							//parametro a moltiplicare il PERTMR2 per parametrizzare la temporizzazione del controllo in posizione
#define P_CHECK_TIME		4							//parametro a moltiplicare il PERTMR2 per parametrizzare la temporizzazione del controllo in posizione
#define U_CHECK_TIME		2							//parametro a moltiplicare il PERTMR2 per parametrizzare la temporizzazione dell'update della cinematica del robot
//U_CHECK_TIME deve essere uguale a V_CHECK_TIME !
#define P_TIMER_INTERVAL	(PERTMR2*P_CHECK_TIME) 		//tempo controllo in posizione	
#define U_TIMER_INTERVAL	(PERTMR2*U_CHECK_TIME)		//tempo update cinematica

typedef struct {
	
    float x, y, theta;
    //coordinate del centro interasse (espresse nel sistema di riferimento inerziale)

    float wheel_radius_l, wheel_radius_r;
    // raggio delle ruote passive (in mm)

    float wheel_factor_l, wheel_factor_r;
    // fattore di conversione da tick a mm

    float K_x, K_y;
    // fattori di correzione per la compensazione della deriva

    float wheel_distance;
    // distanza interasse ruote passive (in mm)

    float tick_per_revolution;
    // numero di tick per giro ruota passiva

    int  tic_L, tic_R;
    //variazione del conteggio deglio encoder in un tempo dt (da aggiornare ogni dt)

    int delta_tic_L;
    int delta_tic_R;
    /* spostamento del centro di interasse in un intervallo dt */

    float delta_linear;
    float delta_angular;
    /* spostamento totale del centro di interasse da quando è partito il controllo: lineare e angolare */
	
    float linear;
    float angular;		
    /* spostamenti lineari dei punti di contatto delle ruote col piano */

    float rotation_radius;
    /* raggio di istantanea rotazione */
    int rotation_radius_inf;
    /* 1 quando il raggio di ist rotazione e' infinito (VL == VR) */
	
    float delta_R, delta_L;				
    /* flags controllo in posizione */
		
    short threshold_flag;
    short continuous_flag;	
    /* variabili per il cross coupling */
		
    float cross_coupling_L;
    float cross_coupling_R;		
    /* altre variabili per il controllo in posizione */
	
    float position_read;

    int path_done, position_valid;
    int target_got, target_got_count, brake;
    int motor_locked_count, motor_locked;

    int position_control_state, bumper_count, bump_timer_count;

} robot;
// contiene tutte le informazioni necessarie per calcolare la posizione del robot (del centro dell'interasse tra le due ruote)


typedef struct {
    controller c;
    float accel;
    float decel;
    float decel_distance;
    float max_speed;
    float max_speed_2; // max_speed*max_speed
    float target;
} t_position_controller;

typedef struct {
    controller c;
    int cx;
    int cy;
    int target_radius;
    int radius;
    float Kdist;
    float Khdg;
    float decel_distance;

    float linear_accel;
    float linear_decel;
    float linear_max_speed;

    float angular_accel;
    float angular_decel;
    float angular_max_speed;
    float angular_max_speed_2;

    float target;
} t_circular_rotation_controller;

typedef struct {
    float a,b,c, den;
    float change_threshold;
    float current_change_threshold;
    bool do_not_brake;
    bool backward;
    bool has_right_heading;
    float target_heading;
    float kp_heading;
    float kp_line_distance;
} t_line_controller;


typedef struct {
    float current_robot_speed;
    float accel;
    float decel;
    float max_speed;
} t_trajectory_profile;


typedef struct {
    int set_x;
    float value;
    float heading;
} t_bump_and_set;

/* variabili esterne: main.c */


extern robot robot_pos;
extern t_position_controller heading_controller, distance_controller;
extern float target_x, target_y;
extern t_line_controller line_profile;


#define CONTROL_OFF                         0
#define CONTROL_STOP                        1
//#define CONTROL_ON_DISTANCE                 2
//#define CONTROL_ON_HEADING                  3
//#define CONTROL_ON_HEADING_AND_DISTANCE     4
#define CONTROL_ON_POINT                    5
#define CONTROL_ON_LINE                     6
#define CONTROL_ON_RELATIVE_ROTATION        7
#define CONTROL_ON_CIRCULAR_ROTATION        8
#define CONTROL_ON_BUMP                     9
#define CONTROL_ON_SIMPLE_BUMP              10
#define CONTROL_ON_POINT_WITH_OFFSET        11
#define CONTROL_ON_HEADING_TO               12
#define CONTROL_ON_HEADING_WITH_OFFSET      13

#define CONTROL_SKIP                        99

/* funzioni */

void init_position(void);
void set_robot_geometry(float radius_left, float radius_right, float distance, float ticks);
void set_wheel_radius(float radius_left, float radius_right);
void set_wheel_distance(float distance);
void set_wheel_radius_left(float radius_left);
void set_wheel_radius_right(float radius_right);
float normalize_angle(float a);

void set_heading_speed(float max_speed, float accel, float decel, float kd);
void set_distance_speed(float max_speed, float accel, float decel, float kd);



void update_go_with_offset_targets(float *target, float *target_h);


void reset_position_controllers(controller*pos, controller* cc_ctrl);
//funzione per il reset dei controller e dello spostamento lineare e angolare del robot


void set_point_control_speed(float max_speed, float accel, float decel, float kd,
                             float max_speed_h, float decel_h, float kd_h);
int distance_control(void);
void position_control(void);
void set_line_control_parameters(float kp_heading, float kp_line_distance, float change_threshold);
void set_circular_rotation_speed(float max_speed, float accel, float decel, float kd, float Kdist, float Khdg);

// ---------------------------------------------------------------
// Interface
void rotate_absolute(float heading);
void rotate_relative(float heading);
void forward_to_distance(float distance);
void forward_to_point(int x, int y);
void line_to_point(int x1, int y1,int x2, int y2, bool backward);
void circular_rotation(int center_delta_x, float angle);
void heading_to (int headX, int headY, int with_back);
void bump_and_set_x(int x, int hdg);
void bump_and_set_y(int x, int hdg);
void simple_bump(void);
void heading_to_with_offset(float x, float y, float offs_x, float offs_y);
void go_with_offset(float x, float y, float offs_x, float offs_y);

void check_motor_locked(void);

#endif
