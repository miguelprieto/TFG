/*
 * path.c
 */

#include "defines.h"
#include <stdbool.h>

#include <p33FJ128MC802.h>

#include "position.h"
#include "path.h"
#include "gpio.h"

t_path path;

void init_path(void)
{
    path.elements = path.current = path.stopped = 0;
}

void add_forward_to_distance(float distance)
{
    int i = path.elements;
    path.el[i].control_type = FORWARD_TO_DISTANCE;
    path.el[i].data.geometry.distance = distance;
    path.elements++;
    robot_pos.path_done = 0;
}

void add_forward_to_point(float x, float y)
{
    int i = path.elements;
    path.el[i].control_type = FORWARD_TO_POINT;
    path.el[i].data.geometry.x = x;
    path.el[i].data.geometry.y = y;
    path.elements++;
    robot_pos.path_done = 0;
}

void add_rotate_absolute(float target)
{
    int i = path.elements;
    path.el[i].control_type = ROTATE_ABSOLUTE;
    path.el[i].data.geometry.heading = target;
    path.elements++;
    robot_pos.path_done = 0;
}

void add_rotate_relative(float target)
{
    int i = path.elements;
    path.el[i].control_type = ROTATE_RELATIVE;
    path.el[i].data.geometry.heading = target;
    path.elements++;
    robot_pos.path_done = 0;
}

void add_line_to_point(float x, float y, unsigned char backward)
{
    int i = path.elements;
    path.el[i].control_type = LINE_TO_POINT;
    path.el[i].data.geometry.x = x;
    path.el[i].data.geometry.y = y;
    path.el[i].data.geometry.flag = backward;
    path.elements++;
    robot_pos.path_done = 0;
}

void put_heading_to(int pos, int x, int y, int with_back)
{
    path.el[pos].control_type = HEADING_TO;
    path.el[pos].data.geometry.x = x;
    path.el[pos].data.geometry.y = y;
    path.el[pos].data.geometry.flag = with_back;
}

void add_heading_to(int x, int y, int with_back)
{
    put_heading_to(path.elements, x, y, with_back);
    path.elements++;
    robot_pos.path_done = 0;
}

void add_rotate_circular(float angle, float x)
{
    int i = path.elements;
    path.el[i].control_type = ROTATE_CIRCULAR;
    path.el[i].data.geometry.heading = angle;
    path.el[i].data.geometry.x = x;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_bump_set_x(int x, int angle)
{
    int i = path.elements;
    path.el[i].control_type = BUMP_SET_X;
    path.el[i].data.geometry.heading = angle;
    path.el[i].data.geometry.x = x;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_bump_set_y(int y, int angle)
{
    int i = path.elements;
    path.el[i].control_type = BUMP_SET_Y;
    path.el[i].data.geometry.heading = angle;
    path.el[i].data.geometry.y = y;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_simple_bump(void)
{
    int i = path.elements;
    path.el[i].control_type = SIMPLE_BUMP;
    path.elements++;
    robot_pos.path_done = 0;
}

void add_go_with_offset(int x, int y, int offs_x, int offs_y)
{
    int i = path.elements;

    path.el[i].control_type = HEADING_WITH_OFFSET;
    path.el[i].data.gwo.x = x;
    path.el[i].data.gwo.y = y;
    path.el[i].data.gwo.offs_x = offs_x;
    path.el[i].data.gwo.offs_y = offs_y;

    i++;

    path.el[i].control_type = GO_WITH_OFFSET;
    path.el[i].data.gwo.x = x;
    path.el[i].data.gwo.y = y;
    path.el[i].data.gwo.offs_x = offs_x;
    path.el[i].data.gwo.offs_y = offs_y;

    path.elements += 2;

    robot_pos.path_done = 0;
}


void add_heading_to_with_offset(int x, int y, int offs_x, int offs_y)
{
    int i = path.elements;

    path.el[i].control_type = HEADING_WITH_OFFSET;
    path.el[i].data.gwo.x = x;
    path.el[i].data.gwo.y = y;
    path.el[i].data.gwo.offs_x = offs_x;
    path.el[i].data.gwo.offs_y = offs_y;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_set_distance_controller(float max_speed, float accel, float decel, float kd)
{
    int i = path.elements;
    path.el[i].control_type = SET_DISTANCE_CONTROLLER;
    path.el[i].data.control.max_speed = max_speed;
    path.el[i].data.control.accel = accel;
    path.el[i].data.control.decel = decel;
    path.el[i].data.control.kd = kd;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_set_heading_controller(float max_speed, float accel, float decel, float kd)
{
    int i = path.elements;
    path.el[i].control_type = SET_HEADING_CONTROLLER;
    path.el[i].data.control.max_speed = max_speed;
    path.el[i].data.control.accel = accel;
    path.el[i].data.control.decel = decel;
    path.el[i].data.control.kd = kd;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_set_circular_rotation_controller(float max_speed, float accel, float decel, float kd, float Kdist, float Khdg)
{
    int i = path.elements;
    path.el[i].control_type = SET_CIRCULAR_ROTATION_CONTROLLER;
    path.el[i].data.control.max_speed = max_speed;
    path.el[i].data.control.accel = accel;
    path.el[i].data.control.decel = decel;
    path.el[i].data.control.kd = kd;
    path.el[i].data.control.kp_line_distance = Kdist;
    path.el[i].data.control.kp_heading = Khdg;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_set_point_controller(float max_speed, float accel, float decel, float kd,
                              float max_speed_h, float decel_h, float kd_h)
{
    int i = path.elements;
    path.el[i].control_type = SET_POINT_CONTROLLER;
    path.el[i].data.control.max_speed = max_speed;
    path.el[i].data.control.accel = accel;
    path.el[i].data.control.decel = decel;
    path.el[i].data.control.kd = kd;
    path.el[i].data.control.max_speed_h = max_speed_h;
    path.el[i].data.control.decel_h = decel_h;
    path.el[i].data.control.kd_h = kd_h;
    path.elements++;
    robot_pos.path_done = 0;
}


void add_set_line_controller(float max_speed, float accel, float decel, float kd,
                             float kp_heading, float kp_line_distance, float change_threshold)
{
    int i = path.elements;
    path.el[i].control_type = SET_LINE_CONTROLLER;
    path.el[i].data.control.max_speed = max_speed;
    path.el[i].data.control.accel = accel;
    path.el[i].data.control.decel = decel;
    path.el[i].data.control.kd = kd;
    path.el[i].data.control.kp_heading = kp_heading;
    path.el[i].data.control.kp_line_distance = kp_line_distance;
    path.el[i].data.control.change_threshold = change_threshold;
    path.elements++;
    robot_pos.path_done = 0;
}

int get_current_path_position(void)
{
    return path.current;
}

void push_path_element(int position)
{
    int i;

    for (i = path.elements - 1; i >= position;i--)
        path.el[i+1] = path.el[i];
    path.elements++;
}


void execute_next_path_element(void)
{
    int i;

 redo:

    i = path.current;

    robot_pos.path_done = 0;

    switch (path.el[i].control_type) {
    case FORWARD_TO_DISTANCE:
        forward_to_distance(path.el[i].data.geometry.distance);
        break;
    case FORWARD_TO_POINT:
        forward_to_point(path.el[i].data.geometry.x, path.el[i].data.geometry.y);
        break;
    case ROTATE_ABSOLUTE:
        rotate_absolute(path.el[i].data.geometry.heading);
        break;
    case ROTATE_RELATIVE:
        rotate_relative(path.el[i].data.geometry.heading);
        break;
    case HEADING_TO:
        heading_to(path.el[i].data.geometry.x, path.el[i].data.geometry.y, path.el[i].data.geometry.flag);
        break;
    case ROTATE_CIRCULAR:
        circular_rotation(path.el[i].data.geometry.x, path.el[i].data.geometry.heading);
        break;
    case LINE_TO_POINT:
        {
            int x1, y1, x2, y2;

            if (i == 0) {
                x1 = robot_pos.x;
                y1 = robot_pos.y;
                x2 = path.el[i].data.geometry.x;
                y2 = path.el[i].data.geometry.y;
            }
            else {
                if (path.el[i-1].control_type == LINE_TO_POINT) {
                    x1 = path.el[i-1].data.geometry.x;
                    y1 = path.el[i-1].data.geometry.y;
                    x2 = path.el[i].data.geometry.x;
                    y2 = path.el[i].data.geometry.y;
                }
                else {
                    x1 = robot_pos.x;
                    y1 = robot_pos.y;
                    x2 = path.el[i].data.geometry.x;
                    y2 = path.el[i].data.geometry.y;
                }
            }

            line_to_point(x1, y1, x2, y2, path.el[i].data.geometry.flag);
        }
        break;

    case HEADING_WITH_OFFSET:
        heading_to_with_offset(path.el[i].data.gwo.x, path.el[i].data.gwo.y,
                               path.el[i].data.gwo.offs_x, path.el[i].data.gwo.offs_y);
        break;

    case GO_WITH_OFFSET:
        go_with_offset(path.el[i].data.gwo.x, path.el[i].data.gwo.y,
                       path.el[i].data.gwo.offs_x, path.el[i].data.gwo.offs_y);
        break;

    case BUMP_SET_X:
        bump_and_set_x(path.el[i].data.geometry.x, path.el[i].data.geometry.heading);
        break;

    case BUMP_SET_Y:
        bump_and_set_y(path.el[i].data.geometry.y, path.el[i].data.geometry.heading);
        break;

    case SIMPLE_BUMP:
        simple_bump();
        break;

    case SET_DISTANCE_CONTROLLER:
        set_distance_speed(path.el[i].data.control.max_speed, path.el[i].data.control.accel,
                           path.el[i].data.control.decel, path.el[i].data.control.kd);
        robot_pos.target_got = 1;
        break;

    case SET_HEADING_CONTROLLER:
        set_heading_speed(path.el[i].data.control.max_speed, path.el[i].data.control.accel,
                          path.el[i].data.control.decel, path.el[i].data.control.kd);
        robot_pos.target_got = 1;
        break;

    case SET_POINT_CONTROLLER:
        /* set_point_control_speed(path.el[i].data.control.max_speed, path.el[i].data.control.accel, */
        /*                         path.el[i].data.control.decel, path.el[i].data.control.kd, */
        /*                         path.el[i].data.control.max_speed_h, path.el[i].data.control.decel_h, */
        /*                         path.el[i].data.control.kd_h); */
        robot_pos.target_got = 1;
        break;

    case SET_LINE_CONTROLLER:
        set_line_control_parameters(path.el[i].data.control.kp_heading, path.el[i].data.control.kp_line_distance,
                                    path.el[i].data.control.change_threshold);
        robot_pos.target_got = 1;
        break;

    case SET_CIRCULAR_ROTATION_CONTROLLER:
        set_circular_rotation_speed(path.el[i].data.control.max_speed, path.el[i].data.control.accel,
                                    path.el[i].data.control.decel, path.el[i].data.control.kd,
                                    path.el[i].data.control.kp_line_distance, path.el[i].data.control.kp_heading);
        robot_pos.target_got = 1;
        break;
    }
}


void path_control(void)
{
    if (path.current >= path.elements) {
        path.current = 0;
        path.elements = 0;
        path.stopped = 1;
        robot_pos.path_done = 1;
        return;
    }

    if (path.stopped == 1) {
        // start the path machine
        path.current = 0;
        path.stopped = 0;
        execute_next_path_element();
        return;
    }

    if (robot_pos.target_got == 1) {
        robot_pos.target_got = 0;
        ++path.current;

        if (path.current >= path.elements) {
            path.current = 0;
            path.elements = 0;
            path.stopped = 1;
            robot_pos.path_done = 1;
            return;
        }

        execute_next_path_element();
    }
}


bool get_next_point_heading(float * hdg, int point_type)
{
    int i = path.current;

    if ( (i + 1) < path.elements ) {
        if (path.el[i+1].control_type == point_type) {
            int x, y;
            x = path.el[i+1].data.geometry.x;
            y = path.el[i+1].data.geometry.y;
            *hdg = atan2(y - robot_pos.y, x - robot_pos.x);
            return true;
        }
    }

    return false;
}


bool get_next_line_to_point_heading(float * hdg)
{
    return get_next_point_heading(hdg, LINE_TO_POINT);
}

