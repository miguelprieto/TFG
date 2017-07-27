/*
 * bus_interface.h
 */

#ifndef __BUS_INTERFACE
#define __BUS_INTERFACE

#include "defines.h"
#include <stdbool.h>

#include "bus_objects.h"
#include "geometry.h"

#define CAN_DELAY_MS   10

#define ROBOT_POSITION_OBJECT			0
#define REMOTE_STDIO_RX_OBJECT			1
#define OBSTACLE_AVOIDANCE_RX_OBJECT		2
#define OBSTACLE_AVOIDANCE_QUERY_RX_OBJECT	3
#define LIFT_TELEMETRY_OBJECT			4
#define SERVO_GPIO_DATA_OBJECT			5
#define STRATEGY_COMMAND_OBJECT			6
#define NBA_COORDINATION_COMMAND_OBJECT		7
#define NBA_COORDINATION_STATUS_OBJECT		8
#define OTHER_ROBOT_POSITION_OBJECT		9

#ifdef ROBOT_GRANDE
#define SERVO_STATUS_GRANDE_OBJECT		10
//#define SERVO_STATUS_GRANDE2_OBJECT		11
#else
#define SERVO_STATUS_PICCOLO_OBJECT		10
#endif

typedef enum   //terzo parametro heading_to, indica il lato del robot che si deve rivolgere al punto
{
	HEADING_FRONT = 0,
	HEADING_RIGHT = 1,
	HEADING_BACK = 2,
	HEADING_LEFT = 3,
	HEADING_FRONT_OR_BACK = 4 // sceglie lato che comporta la rotazione minore
} heading_pos;

typedef struct {
    float x, y, theta;
    bool path_done;
    bool block_detection;
    bool position_valid;
#if defined (ROBOT_GRANDE)
    bool left_bumpers;
    bool right_bumpers;
#else
    bool back_bumpers;
    bool front_bumpers;
#endif
} t_robot_position;

class DistanceControllerParameters
{
	public:
		bool apply() const;
		static inline const DistanceControllerParameters& getCurrent() { return currentParameters; }

		int max_speed, accel, decel, kdm, kde;

	private:
		static DistanceControllerParameters currentParameters;
};

class LineControllerParameters
{
	public:
		bool apply() const;
		static inline const LineControllerParameters& getCurrent() { return currentParameters; }

		int max_speed, accel, decel, kdm, kde;
		int kp_h_m, kp_h_e, kp_line_m, kp_line_e;
		int change_threshold;

	private:
		static LineControllerParameters currentParameters;
};

class PointControllerParameters
{
	public:
		bool apply() const;
		static inline const PointControllerParameters& getCurrent() { return currentParameters; }

		int max_speed, accel, decel, kdm, kde;
		int max_speed_h, decel_h, kdm_h, kde_h;

	private:
		static PointControllerParameters currentParameters;
};

class HeadingControllerParameters
{
	public:
		bool apply() const;
		static inline const HeadingControllerParameters& getCurrent() { return currentParameters; }

		int max_speed, accel, decel, kdm, kde;

	private:
		static HeadingControllerParameters currentParameters;
};

class CircularRotationControllerParameters
{
	public:
		bool apply() const;
		static inline const CircularRotationControllerParameters& getCurrent() { return currentParameters; }

		int max_speed, accel, decel, kdm, kde;
		int kp_dist_m, kp_dist_e, kp_hdg_m, kp_hdg_e;

	private:
		static CircularRotationControllerParameters currentParameters;
};

template <class T>
class AutoControllerParamsRestorer
{
	public:
		explicit AutoControllerParamsRestorer(const T &oldParams = T::getCurrent())
		: oldParams(oldParams), armed(true)
		{
		}

		// Impedisce la copia di oggetti di questo tipo
		AutoControllerParamsRestorer(const AutoControllerParamsRestorer&) = delete;
		AutoControllerParamsRestorer &operator=(const AutoControllerParamsRestorer&) = delete;

		void restoreNow()
		{
			oldParams.apply();
			armed = false;
		}

		~AutoControllerParamsRestorer()
		{
			if (armed)
				oldParams.apply();
		}

	private:
		const T oldParams;
		bool armed;
};

void init_can_objects(void);

void send_lift_telemetry_enable(bool on);
float get_lift_position(void);
bool lift_go_to(int pos);
void send_lift_position(int lift, int pos);
void lift_home(int lift);
void lift_wait_home(void);
void lift_stop(int lift);
void lift_pwm(int lift, int pwm);
void lift_speed(int lift, int s);

t_servo_gpio * get_servo_gpio(void);
int set_servo_position(int servo_num, int servo_value);
int set_ax12_servo_position(int servo_num, int servo_value);

int set_wheel_radius(float left, float right);
int set_wheel_distance(float d);
int set_k(float kx, float ky);
int speed_pid_parameters(int kpm, int kpe, int kim, int kie, int kdm, int kde);
int set_speed(int vl,int vr);
int set_speed_no_lock(int vl,int vr);
int set_pwm(int vl, int vr);
int motion_stop(void);
int motion_brake(void);
int set_position(const Point &pos, float theta);
int set_position_valid_flag(bool position_valid);

int forward_to_distance(int distance);
int forward_to_point(double x, double y);
int forward_to_point(const Point &pos);
int line_to_point(const Point &pos, bool backward = false);
int rotate_relative(int degrees);
int rotate_absolute(int degrees);
int rotate_circular(float degrees, int relative_x); // x=scostamento rispetto a centro robot
int heading_to(const Point &pos, heading_pos with_back);
int go_with_offset(const Point &pos, int offset_x, int offset_y);
int heading_to_with_offset(const Point &pos, int offset_x, int offset_y);
int set_status_display(t_can_robot_status_update_status status);
Point get_pos(double *t = NULL);
Point get_pos_other_robot(double *t = NULL);
bool is_motor_locked();

int bump_and_set_y(int y, int hdg);
int bump_and_set_x(int x, int hdg);
int bump(void);

void reset_obstacle_avoidance(void);
void disable_obstacle_avoidance(void);
void enable_obstacle_avoidance(void);
void set_obstacle_avoidance_nearonly(bool nearonly);
bool check_obstacle_detected(double *dir);

bool check_obstacle_direction(double *dir);

int set_minimal_speed(int v, float w);
int set_error_tolerance(float lin, float rot);
int set_anticipation_gain(float lin, float rot);
int set_error_to_minimal_speed(float v, float w);

void send_start_piccolo(char flags);
bool receive_strategy_command(unsigned char *out_cmd, unsigned char *out_flags, unsigned int *out_elapsed_time);

void nba_command_send();
bool nba_command_receive();

void nba_status_send(char status);
bool nba_status_receive(char *out_status);

extern t_robot_position robot_position;
extern int running_goal;

#endif
