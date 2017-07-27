/*
 * path.h
 */

#ifndef __PATH_H
#define __PATH_H


#define MAX_PATH     32


typedef struct {
    int control_type;
    union {
        struct {
            float x, y, heading, distance;
            int flag;
        } geometry;
        struct {
            float x, y, offs_x, offs_y;
        } gwo; // parametri go with offset
        struct {
            float max_speed, accel, decel, kd, kp_correction;
            float max_speed_h, accel_h, decel_h, kd_h;
            float kp_heading, kp_line_distance, change_threshold;
        } control;
    } data;
} t_path_element;


typedef struct {
    t_path_element   el[MAX_PATH];
    int elements, current, stopped;
} t_path;

extern t_path path;

#define FORWARD_TO_DISTANCE       1
#define FORWARD_TO_POINT          2
#define ROTATE_ABSOLUTE           3
#define LINE_TO_POINT             4
#define ROTATE_RELATIVE           5
#define ROTATE_CIRCULAR           6
#define HEADING_TO                7
#define BUMP_SET_X                8
#define BUMP_SET_Y                9
#define SIMPLE_BUMP               10
#define GO_WITH_OFFSET            11
#define HEADING_WITH_OFFSET       12


#define SET_DISTANCE_CONTROLLER           100
#define SET_HEADING_CONTROLLER            101
#define SET_POINT_CONTROLLER              102
#define SET_LINE_CONTROLLER               103
#define SET_CIRCULAR_ROTATION_CONTROLLER  104

void init_path(void);

void add_forward_to_distance(float distance);
void add_forward_to_point(float x, float y);
void add_rotate_absolute(float target);
void add_rotate_relative(float target);
void add_line_to_point(float x, float y, unsigned char backward);
void add_rotate_circular(float angle, float x);
void put_heading_to(int pos, int x, int y, int with_back);
void add_heading_to(int x, int y, int with_back);
void add_bump_set_x(int y, int angle);
void add_bump_set_y(int y, int angle);
void add_simple_bump(void);
void add_go_with_offset(int x, int y, int offs_x, int offs_y);
void add_heading_to_with_offset(int x, int y, int offs_x, int offs_y);

void add_set_distance_controller(float max_speed, float accel, float decel, float kd);
void add_set_heading_controller(float max_speed, float accel, float decel, float kd);
void add_set_circular_rotation_controller(float max_speed, float accel, float decel, float kd, float Kdist, float Khdg);
void add_set_point_controller(float max_speed, float accel, float decel, float kd,
                              float max_speed_h, float decel_h, float kd_h);
void add_set_line_controller(float max_speed, float accel, float decel, float kd,
                             float kp_heading, float kp_line_distance, float change_threshold);

void push_path_element(int position);
void path_control(void);
bool get_next_line_to_point_heading(float * hdg);
int get_current_path_position(void);


#endif
