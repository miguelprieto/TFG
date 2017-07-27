#include "defines.h"

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <libpic30++.h>
#include <string.h>
#include <math.h>

#include "adc.h"
#include "bus_interface.h"
#include "canstdio_endpoint.h"
#include "ecan_lib.h"
#include "goals.h"
#include "servos.h"
#include "routing.h"
#include "wait.h"

// Max speed -- da decommentare solo in fase di omologazione
// omologazione2014=500, startOK=700, rischioso=800
//#define MAX_SPEED 700 -- è safe 2015
// -- #define MAX_SPEED 500 // omologazione 2015 (grande)
#define MAX_SPEED 900 // test 2015
#ifdef MAX_SPEED
#define MAX_SPEED_CAP(s) ((s) < MAX_SPEED ? (s) : MAX_SPEED)
#else
#define MAX_SPEED_CAP(s) (s)
#endif

t_robot_position robot_position, other_robot_position;

// ultimi parametri per ciascun controller
DistanceControllerParameters DistanceControllerParameters::currentParameters;
LineControllerParameters LineControllerParameters::currentParameters;
PointControllerParameters PointControllerParameters::currentParameters;
HeadingControllerParameters HeadingControllerParameters::currentParameters;
CircularRotationControllerParameters CircularRotationControllerParameters::currentParameters;

static bool obstacle_detected = false;
static bool obstacle_processed_ack;
static double obstacle_direction;

static float lift_position;
static int lift_status;

static t_servo_gpio servo_gpio;

static enum
{
	WAITING,
	GOT_TRUE,
	GOT_FALSE
} obstacle_query_status;
static double obstacle_query_result_dir;

static int last_strategy_command = -1;
static unsigned char strategy_command_flags;
static unsigned int strategy_command_elapsed_time;

static bool nba_command_received = false, nba_status_received = false;
static char nba_status_value;

static void update_robot_position(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_robot_position *m = (const t_can_robot_position*)data;

    robot_position.x = m->x;
    robot_position.y = m->y;
    robot_position.theta = m->deg100 / 100.0;
    robot_position.path_done = (bool)(m->flags & 1);
    robot_position.block_detection = (bool)((m->flags & 2) >> 1);
    robot_position.position_valid = (bool)((m->flags & 4) >> 2);
#if defined(ROBOT_GRANDE)
    robot_position.right_bumpers = (bool)((m->bumpers & 1));
    robot_position.left_bumpers = (bool)((m->bumpers & 2) >> 1);
#else
    robot_position.front_bumpers = (bool)((m->bumpers & 1));
    robot_position.back_bumpers = (bool)((m->bumpers & 2) >> 1);
#endif
}

static void update_obstacle_avoidance(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_obstacle_avoidance *m = (const t_can_obstacle_avoidance*)data;

    if (m->msg_type == CAN_OBSTACLE_AVOIDANCE_MSG_OBSTACLEDETECTED)
    {
        obstacle_direction = m->obstacle_direction;
        obstacle_detected = true;
    }
    else if (m->msg_type == CAN_OBSTACLE_AVOIDANCE_MSG_ACKOBSTACLEPROCESSED)
    {
        obstacle_processed_ack = true;
    }
}

static void update_obstacle_avoidance_query(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_obstacle_avoidance *m = (const t_can_obstacle_avoidance*)data;

    if (m->msg_type == CAN_OBSTACLE_AVOIDANCE_QUERY_MSG_OBSTFOUND)
    {
        obstacle_query_status = GOT_TRUE;
	obstacle_query_result_dir = m->obstacle_direction;
    }
    else if (m->msg_type == CAN_OBSTACLE_AVOIDANCE_QUERY_MSG_OBSTNOTFOUND)
    {
        obstacle_query_status = GOT_FALSE;
    }
}

static void update_lift_telemetry(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_lift_telemetry_encoder_data *m = (const t_can_lift_telemetry_encoder_data*)data;

    if (m->frame_id == 0x10 && m->sub_id == 0x01)
    {
        lift_position = m->value;
        lift_status = m->bumper;
    }
}

static void update_servo_gpio(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_servo_gpio *m = (const t_servo_gpio*)data;
    servo_gpio = *m;
}

static void update_other_robot_position(const uint8_t *data, unsigned int len, void *user_ptr)
{
    const t_can_robot_position *m = (const t_can_robot_position*)data;

    other_robot_position.x = m->x;
    other_robot_position.y = m->y;
    other_robot_position.theta = m->deg100 / 100.0;
}

void init_can_objects(void)
{
    ecan_set_rx_object(ROBOT_POSITION_OBJECT, ROBOT_POSITION_CAN_ID, update_robot_position, NULL, 0);
    ecan_set_rx_object(REMOTE_STDIO_RX_OBJECT, REMOTE_STDIO_CAN_ID(CAN_STDIO_AND_RTSP_NODE_ID),
                       canstdio_endpoint_process_can_frame, NULL, ECAN_RX_FLAG_ASYNC);
    ecan_set_rx_object(OBSTACLE_AVOIDANCE_RX_OBJECT, OBSTACLE_AVOIDANCE_CAN_ID, update_obstacle_avoidance, NULL, 0);
    ecan_set_rx_object(OBSTACLE_AVOIDANCE_QUERY_RX_OBJECT, OBSTACLE_AVOIDANCE_QUERY_CAN_ID,
                       update_obstacle_avoidance_query, NULL, 0);
    ecan_set_rx_object(LIFT_TELEMETRY_OBJECT, LIFT_TELEMETRY_DATA_CAN_ID, update_lift_telemetry, NULL, 0);
    ecan_set_rx_object(SERVO_GPIO_DATA_OBJECT, SERVO_GPIO_CAN_ID, update_servo_gpio, NULL, 0);
    // ecan_set_rx_object(STRATEGY_COMMAND_OBJECT, STRATEGY_COMMAND_CAN_ID, update_strategy_command, NULL, 0);
    // ecan_set_rx_object(NBA_COORDINATION_COMMAND_OBJECT, NBA_COORDINATION_COMMAND_ID, update_nba_command, NULL, 0);
    // ecan_set_rx_object(NBA_COORDINATION_STATUS_OBJECT, NBA_COORDINATION_STATUS_ID, update_nba_status, NULL, 0);
    ecan_set_rx_object(OTHER_ROBOT_POSITION_OBJECT, OTHER_ROBOT_POSITION_CAN_ID, update_other_robot_position, NULL, 0);
#ifdef ROBOT_GRANDE
    ecan_set_rx_object(SERVO_STATUS_GRANDE_OBJECT, SERVO_STATUS_GRANDE_CAN_ID, update_servo_status_grande, NULL, 0);
    //ecan_set_rx_object(SERVO_STATUS_GRANDE2_OBJECT, SERVO_STATUS_GRANDE2_CAN_ID, update_servo_status_grande2, NULL, 0);
#else
    ecan_set_rx_object(SERVO_STATUS_PICCOLO_OBJECT, SERVO_STATUS_PICCOLO_CAN_ID, update_servo_status_piccolo, NULL, 0);
#endif
}

void send_lift_telemetry_enable(bool enable)
{
    t_command_position_generic p;

    p._cmd = POSITION_TELEMETRY;
    p.axis = enable ? 1 : 0;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}

float get_lift_position()
{
    ecan_update_object(LIFT_TELEMETRY_OBJECT);
    return lift_position;
}

t_servo_gpio * get_servo_gpio(void)
{
    ecan_update_object(SERVO_GPIO_DATA_OBJECT);
    return &servo_gpio;
}


bool lift_go_to(int pos)
{
	int start_lift = game_timer;

	send_lift_position(0, pos);
	do
	{
		__delay_ms(100);
		if (game_timer - start_lift > 8)
			return false;
	}
	while (fabs(get_lift_position() - pos) > 5);

	return true;
}

void send_lift_position(int lift, int pos)
{
    t_command_position_generic p;

    p._cmd = POSITION_GO;
    p.axis = lift;
    p.position = pos;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}


void lift_home(int lift)
{
    t_command_position_generic p;

    p._cmd = POSITION_HOME;
    p.axis = lift;
    p.position = 0;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}


void lift_wait_home(void)
{
    do {
        ecan_update_object(LIFT_TELEMETRY_OBJECT);
        __delay_ms(100);
    } while ((lift_status & 0x7f) != 3);
}



void lift_pwm(int lift, int pwm)
{
    t_command_position_generic p;

    p._cmd = POSITION_PWM;
    p.axis = lift;
    p.position = pwm;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}


void lift_speed(int lift, int s)
{
    t_command_position_generic p;

    p._cmd = POSITION_SPEED;
    p.axis = lift;
    p.position = s;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}


void lift_stop(int lift)
{
    t_command_position_generic p;

    p._cmd = POSITION_OFF;
    p.axis = lift;
    p.position = 0;
    ecan_send(POSITION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}


int set_servo_position(int servo_num, int servo_value)
{
    t_servo_position pos;
    pos.servo_num = servo_num;
    pos.position = servo_value;
    return ecan_send(SERVO_POSITION_CAN_ID, (unsigned char *)&pos, 8, 0);
}


int set_ax12_servo_position(int servo_num, int servo_value)
{
    t_servo_position pos;
    pos.servo_num = servo_num;
    pos.position = servo_value;
    return ecan_send(AX12_SERVO_POSITION_CAN_ID, (unsigned char *)&pos, 8, 0);
}


int set_servo_shake_position(int servo_num, int servo_value_1, int servo_value_2)
{
    t_servo_position pos;
    pos.servo_num = 0xff; // automation
    pos.position = 0x01; // set shake positions
    pos._padding[0] = servo_num;
    *(short *)(pos._padding + 1) = (short)servo_value_1;
    *(short *)(pos._padding + 3) = (short)servo_value_2;
    return ecan_send(SERVO_POSITION_CAN_ID, (unsigned char *)&pos, 8, 0);
}


int servo_shake_on(void)
{
    t_servo_position pos;
    pos.servo_num = 0xff; // automation
    pos.position = 0x02; // shake on
    return ecan_send(SERVO_POSITION_CAN_ID, (unsigned char *)&pos, 8, 0);
}


int set_k(float kx, float ky)
{
    t_command_set_K p;
    int r;

    p._cmd = MOTION_COMMAND_SET_KX;
    p.K = kx;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    p._cmd = MOTION_COMMAND_SET_KY;
    p.K = ky;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    return r;
}


int set_wheel_radius(float left, float right)
{
    t_command_set_wheel_radius p;
    int r;
    p._cmd = MOTION_COMMAND_SET_WHEEL_RADIUS_LEFT;
    p.wheel_radius = left;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    p._cmd = MOTION_COMMAND_SET_WHEEL_RADIUS_RIGHT;
    p.wheel_radius = right;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}


int set_wheel_distance(float d)
{
    t_command_set_wheel_distance p;
    int r;
    p._cmd = MOTION_COMMAND_SET_WHEEL_DISTANCE;
    p.wheel_distance = d;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}


int speed_pid_parameters(int kpm,int kpe,int kim,int kie,int kdm,int kde)
{
    t_command_speed_pid p;
    int r;
    p._cmd = MOTION_COMMAND_SPEED_PID;
    p.kp_m = kpm;
    p.kp_e = kpe;
    p.ki_m = kim;
    p.ki_e = kie;
    p.kd_m = kdm;
    p.kd_e = kde;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}


int set_speed(int vl,int vr)
{
    t_command_set_pwm_speed p;
    int r;
    p._cmd = MOTION_COMMAND_SET_SPEED;
    p.left = vl;
    p.right = vr;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int set_speed_no_lock(int vl,int vr)
{
    t_command_set_pwm_speed p;
    int r;
    p._cmd = MOTION_COMMAND_SET_SPEED_NO_LOCK;
    p.left = vl;
    p.right = vr;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int set_pwm(int vl, int vr)
{
    t_command_set_pwm_speed p;
    int r;
    p._cmd = MOTION_COMMAND_SET_PMW;
    p.left = vl;
    p.right = vr;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int motion_stop(void)
{
    t_can_motion_command p;
    int r;
    p._cmd = MOTION_COMMAND_STOP_AND_FREE;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int motion_brake(void)
{
    t_can_motion_command p;
    int r;
    p._cmd = MOTION_COMMAND_STOP_AND_BRAKE;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int set_position(const Point &pos, float theta)
{
    t_command_set_current_position p;
    int r;
    p._cmd = MOTION_COMMAND_SET_CURRENT_POSITION;
    p.x = (short)pos.x;
    p.y = (short)pos.y;
    p.deg100 = (short)(theta * 100);
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int set_position_valid_flag(bool position_valid)
{
    t_command_set_current_position p;
    int r;
    if (position_valid)
        p._cmd = MOTION_COMMAND_SET_POSITION_VALID_FLAG;
    else
        p._cmd = MOTION_COMMAND_CLEAR_POSITION_VALID_FLAG;
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

bool DistanceControllerParameters::apply() const
{
    currentParameters = *this;

    t_command_distance_controller_1 c1;
    t_command_distance_controller_2 c2;
    int r;

    c1._cmd = MOTION_COMMAND_DISTANCE_CONTROLLER_1;
    c1.max_speed = MAX_SPEED_CAP(max_speed);
    c1.accel = accel;

    c2._cmd = MOTION_COMMAND_DISTANCE_CONTROLLER_2;
    c2.decel = decel;
    c2.kd_m = kdm;
    c2.kd_e = kde;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c1, 8, 0);
    if (r != 1) return false;

    __delay_ms(CAN_DELAY_MS);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c2, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    return true;
}

bool HeadingControllerParameters::apply() const
{
    currentParameters = *this;

    t_command_heading_controller_1 c1;
    t_command_heading_controller_2 c2;
    int r;

    c1._cmd = MOTION_COMMAND_HEADING_CONTROLLER_1;
    c1.max_speed = max_speed;
    c1.accel = accel;

    c2._cmd = MOTION_COMMAND_HEADING_CONTROLLER_2;
    c2.decel = decel;
    c2.kd_m = kdm;
    c2.kd_e = kde;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c1, 8, 0);
    if (r != 1) return false;

    __delay_ms(CAN_DELAY_MS);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c2, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    return true;
}

bool CircularRotationControllerParameters::apply() const
{
    currentParameters = *this;

    t_command_circular_rotation_controller_1 c1;
    t_command_circular_rotation_controller_2 c2;
    int r;

    c1._cmd = MOTION_COMMAND_CIRCULAR_ROTATION_CONTROLLER_1;
    c1.max_speed = max_speed;
    c1.accel = accel;
    c1.decel = decel;

    c2._cmd = MOTION_COMMAND_CIRCULAR_ROTATION_CONTROLLER_2;
    c2.kd_m = kdm;
    c2.kd_e = kde;
    c2.kp_dist_m = kp_dist_m;
    c2.kp_dist_e = kp_dist_e;
    c2.kp_h_m = kp_hdg_m;
    c2.kp_h_e = kp_hdg_e;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c1, 8, 0);
    if (r != 1) return false;

    __delay_ms(CAN_DELAY_MS);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c2, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    return true;
}

bool LineControllerParameters::apply() const
{
    currentParameters = *this;

    t_command_line_controller_1 c1;
    t_command_line_controller_2 c2;
    t_command_line_controller_3 c3;
    int r;

    c1._cmd = MOTION_COMMAND_LINE_CONTROLLER_1;
    c1.max_speed = MAX_SPEED_CAP(max_speed);
    c1.accel = accel;
    c1.decel = decel;

    c2._cmd = MOTION_COMMAND_LINE_CONTROLLER_2;
    c2.kd_m = kdm;
    c2.kd_e = kde;
    c2.kp_h_m = kp_h_m;
    c2.kp_h_e = kp_h_e;
    c2.kp_line_m = kp_line_m;
    c2.kp_line_e = kp_line_e;

    c3._cmd = MOTION_COMMAND_LINE_CONTROLLER_3;
    c3.change_threshold = change_threshold;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c1, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c2, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c3, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    return true;
}

bool PointControllerParameters::apply() const
{
    currentParameters = *this;

    t_command_point_controller_1 c1;
    t_command_point_controller_2 c2;
    t_command_point_controller_3 c3;
    int r;

    c1._cmd = MOTION_COMMAND_POINT_CONTROLLER_1;
    c1.max_speed = MAX_SPEED_CAP(max_speed);
    c1.accel = accel;
    c1.decel = decel;

    c2._cmd = MOTION_COMMAND_POINT_CONTROLLER_2;
    c2.max_speed_h = max_speed_h;
    c2.accel_h = 0;
    c2.decel_h = decel_h;

    c3._cmd = MOTION_COMMAND_POINT_CONTROLLER_3;
    c3.kd_m = kdm;
    c3.kd_e = kde;
    c3.kd_h_m = kdm_h;
    c3.kd_h_e = kde_h;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c1, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c2, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c3, 8, 0);
    if (r != 1) return false;
    __delay_ms(CAN_DELAY_MS);

    return true;
}

int set_minimal_speed(int v, float w)
{
    t_command_set_minimal_speed c;
    int r;

    c._cmd = MOTION_COMMAND_SET_MINIMAL_SPEED;
    c.linear_speed = v;
    c.rotation_10_speed =(int)(w * 10);
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int set_error_tolerance(float lin, float rot)
{
    t_command_set_error_tolerance c;
    int r;

    c._cmd = MOTION_COMMAND_SET_ERROR_TOLERANCE;
    c.linear_10_error = (int)(lin * 10);
    c.heading_10_error =(int)(rot * 10);
    c.heading_10_to_minimal_speed = 0;  // not used
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int set_anticipation_gain(float lin, float rot)
{
    t_command_set_anticipation_gain c;
    int r;

    c._cmd = MOTION_COMMAND_SET_ANTICIPATION_GAIN;
    c.linear_gain = (int)(lin * 100);
    c.heading_gain =(int)(rot * 100);
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int set_error_to_minimal_speed(float lin, float rot)
{
    t_command_set_error_tolerance c;
    int r;

    c._cmd = MOTION_COMMAND_SET_ERROR_TO_MINIMAL_SPEED;
    c.linear_10_error = (int)(lin * 10);
    c.heading_10_error =(int)(rot * 10);
    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&c, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int forward_to_distance(int distance)
{
    t_command_forward_to_distance p;
    int r;

    p._cmd = MOTION_COMMAND_FORWARD_TO_DISTANCE;
    p.distance = distance;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int forward_to_point(const Point &pos)
{
    t_command_forward_to_point p;
    int r;

    p._cmd = MOTION_COMMAND_FORWARD_TO_POINT;
    p.x = (int)pos.x;
    p.y = (int)pos.y;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int forward_to_point(double x, double y)
{
    t_command_forward_to_point p;
    int r;

    p._cmd = MOTION_COMMAND_FORWARD_TO_POINT;
    p.x = (int)x;
    p.y = (int)y;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int line_to_point(const Point &pos, bool backward)
{
    t_command_line_to_point p;
    int r;

    p._cmd = MOTION_COMMAND_LINE_TO_POINT;
    p.x = (int)pos.x;
    p.y = (int)pos.y;
    p.backward = backward;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int rotate_relative(int degrees)
{
    t_command_rotate p;
    int r;

    p._cmd = MOTION_COMMAND_ROTATE_RELATIVE;
    p.degrees = degrees;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int rotate_absolute(int degrees)
{
    t_command_rotate p;
    int r;

    p._cmd = MOTION_COMMAND_ROTATE_ABSOLUTE;
    p.degrees = degrees;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int rotate_circular(float degrees, int relative_x)
{
    t_command_rotate_circular p;
    int r;

    p._cmd = MOTION_COMMAND_ROTATE_CIRCULAR;
    p.degrees = (short)(degrees*10);
    p.x = relative_x;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}


int heading_to(const Point &pos, heading_pos with_back)
{
    t_command_heading_to p;
    int r;

    p._cmd = MOTION_COMMAND_HEADING_TO;
    p.x = (int)pos.x;
    p.y = (int)pos.y;
    p.with_back = with_back;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);
    return r;
}

int bump_and_set_x(int x, int hdg)
{
    t_command_bump_and_set_x p;
    int r;

    p._cmd = MOTION_COMMAND_BUMP_AND_SET_X;
    p.x = x;
    p.hdg = hdg;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    return r;
}

int bump_and_set_y(int y, int hdg)
{
    t_command_bump_and_set_y p;
    int r;

    p._cmd = MOTION_COMMAND_BUMP_AND_SET_Y;
    p.y = y;
    p.hdg = hdg;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    return r;
}

/* ATTENTION!!! THE BUMP COMMAND IGNORES MOTOR LOCKED! */
int bump(void)
{
    t_command_simple_bump p;
    int r;

    p._cmd = MOTION_COMMAND_SIMPLE_BUMP;

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    return r;
}

int go_with_offset(const Point &pos, int offset_x, int offset_y)
{
    t_command_go_with_offset p;
    int r;

    p._cmd = MOTION_COMMAND_GO_WITH_OFFSET;
    p.x = (int)pos.x;
    p.y = (int)pos.y;
    p.high_offs_x = offset_x >> 4;
    p.high_offs_y = offset_y >> 4;
    p.lowbits_offs_xy = (offset_x & 0xf) | ((offset_y & 0xf) << 4);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    return r;
}

int heading_to_with_offset(const Point &pos, int offset_x, int offset_y)
{
    t_command_go_with_offset p;
    int r;

    p._cmd = MOTION_COMMAND_HEADING_TO_WITH_OFFSET;
    p.x = (int)pos.x;
    p.y = (int)pos.y;
    p.high_offs_x = offset_x >> 4;
    p.high_offs_y = offset_y >> 4;
    p.lowbits_offs_xy = (offset_x & 0xf) | ((offset_y & 0xf) << 4);

    r = ecan_send(MOTION_COMMAND_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    return r;
}

int set_status_display(t_can_robot_status_update_status status)
{
    t_can_robot_status_update p;
    int r;

    p.status_display = (int)status;
#ifdef ROBOT_GRANDE
    p.robot_selected = (int)CAN_ROBOT_STATUS_UPDATE_ROBOT_GRANDE;
#else
    p.robot_selected = (int)CAN_ROBOT_STATUS_UPDATE_ROBOT_PICCOLO;
#endif

    r = ecan_send(ROBOT_STATUS_UPDATE_CAN_ID, (unsigned char *)&p, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    return r;
}

Point get_pos(double *t)
{
    ecan_update_object(ROBOT_POSITION_OBJECT);
    if (t != NULL)
        *t = robot_position.theta;
    return Point(robot_position.x, robot_position.y);
}

Point get_pos_other_robot(double *t)
{
    ecan_update_object(OTHER_ROBOT_POSITION_OBJECT);
    if (t != NULL)
        *t = other_robot_position.theta;
    return Point(other_robot_position.x, other_robot_position.y);
}

bool is_motor_locked()
{
	ecan_update_object(ROBOT_POSITION_OBJECT);
	return robot_position.block_detection;
}

void reset_obstacle_avoidance(void)
{
    t_can_obstacle_avoidance m;
    m.msg_type = CAN_OBSTACLE_AVOIDANCE_MSG_RESET;
    ecan_send(OBSTACLE_AVOIDANCE_CAN_ID, (unsigned char *)&m, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}

void disable_obstacle_avoidance(void)
{
    t_can_obstacle_avoidance m;
    m.msg_type = CAN_OBSTACLE_AVOIDANCE_MSG_DISABLE;
    ecan_send(OBSTACLE_AVOIDANCE_CAN_ID, (unsigned char *)&m, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}

void enable_obstacle_avoidance(void)
{
    t_can_obstacle_avoidance m;
    m.msg_type = CAN_OBSTACLE_AVOIDANCE_MSG_ENABLE;
    ecan_send(OBSTACLE_AVOIDANCE_CAN_ID, (unsigned char *)&m, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}

void set_obstacle_avoidance_nearonly(bool nearonly)
{
    t_can_obstacle_avoidance m;
    m.msg_type = nearonly ? CAN_OBSTACLE_AVOIDANCE_MSG_SET_NEARONLY_FLAG : CAN_OBSTACLE_AVOIDANCE_MSG_CLR_NEARONLY_FLAG;
    ecan_send(OBSTACLE_AVOIDANCE_CAN_ID, (unsigned char *)&m, 8, 0);
    __delay_ms(CAN_DELAY_MS);
}

bool check_obstacle_detected(double *dir)
{
    ecan_update_object(OBSTACLE_AVOIDANCE_RX_OBJECT);

    if (obstacle_detected == false)
        return false;

    *dir = obstacle_direction;
    obstacle_detected = false;

    obstacle_processed_ack = false;

    t_can_obstacle_avoidance m;
    m.msg_type = CAN_OBSTACLE_AVOIDANCE_MSG_OBSTACLEPROCESSED;
    ecan_send(OBSTACLE_AVOIDANCE_CAN_ID, (unsigned char *)&m, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    while (!obstacle_processed_ack)
        ecan_update_object(OBSTACLE_AVOIDANCE_RX_OBJECT);

    return true;
}

bool check_obstacle_direction(double *dir)
{
    obstacle_query_status = WAITING;

    t_can_obstacle_avoidance m;
    m.msg_type = CAN_OBSTACLE_AVOIDANCE_QUERY_MSG_REQUEST;
    m.obstacle_direction = *dir;
    ecan_send(OBSTACLE_AVOIDANCE_QUERY_CAN_ID, (unsigned char *)&m, 8, 0);
    __delay_ms(CAN_DELAY_MS);

    while (obstacle_query_status == WAITING)
        ecan_update_object(OBSTACLE_AVOIDANCE_QUERY_RX_OBJECT);

    if (obstacle_query_status == GOT_TRUE)
    {
        *dir = obstacle_query_result_dir;
        return true;
    }
    else
    {
        return false;
    }
}

void send_start_piccolo(char flags)
{
    t_can_strategy_command c;
    c.cmd = STRATEGY_COMMAND_START_PICCOLO;
    c.flags = flags;
    c.elapsed_time = 0;
    ecan_send(STRATEGY_COMMAND_CAN_ID, (unsigned char*)&c, sizeof(t_can_strategy_command), 0);
}

bool receive_strategy_command(unsigned char *out_cmd, unsigned char *out_flags, unsigned int *out_elapsed_time)
{
    ecan_update_object(STRATEGY_COMMAND_OBJECT);

    if (last_strategy_command == -1)
        return false;

    *out_cmd = last_strategy_command;
    *out_flags = strategy_command_flags;
    *out_elapsed_time = strategy_command_elapsed_time;

    last_strategy_command = -1;
    return true;
}

void nba_command_send()
{
	t_can_nba_coordination_command m;
	ecan_send(NBA_COORDINATION_COMMAND_ID, (unsigned char*)&m, sizeof(t_can_nba_coordination_command), 0);
}

bool nba_command_receive()
{
	ecan_update_object(NBA_COORDINATION_COMMAND_OBJECT);

	if (nba_command_received == false)
		return false;

	nba_command_received = false;
	return true;
}

void nba_status_send(char status)
{
	t_can_nba_coordination_status m;
	m.status = status;
	ecan_send(NBA_COORDINATION_STATUS_ID, (unsigned char*)&m, sizeof(t_can_nba_coordination_status), 0);
}

bool nba_status_receive(char *out_status)
{
	ecan_update_object(NBA_COORDINATION_STATUS_OBJECT);

	if (nba_status_received == false)
		return false;

	*out_status = nba_status_value;

	nba_status_received = false;
	return true;
}
