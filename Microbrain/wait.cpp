/*
 * wait.c
 */

#include "defines.h"

#include <xc.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <libpic30++.h>

#include "ecan_lib.h"
#include "bus_interface.h"
#include "routing.h"
#include "simulator_interface.h"
#include "wait.h"

WaitResult waitImpl(const AbstractWaitableEvent *events[], unsigned int events_count)
{
	__delay_ms(50);

	printf("wait(");
	for (unsigned i = 0; i < events_count; i++)
	{
		if (i != 0)
			printf(", ");
		events[i]->printDescription();
	}
	printf(")\n");

	while (true)
	{
		simulator_relax();

		for (unsigned i = 0; i < events_count; i++)
		{
			const WaitResult r = events[i]->testOccurred();
			if (r != NOT_OCCURRED)
				return r;
		}
	}
}

// AbstractWaitableEvent
AbstractWaitableEvent::~AbstractWaitableEvent()
{
}

// MotionEvent
MotionEvent::MotionEvent()
{
}

void MotionEvent::printDescription() const
{
	printf("MotionEvent()");
}

WaitResult MotionEvent::testOccurred() const
{
	double obstacle_direction;
	ecan_update_object(ROBOT_POSITION_OBJECT);

	if (check_obstacle_detected(&obstacle_direction))
	{
		closePointsInDirection(obstacle_direction);
		printf("OBSTACLE\n");
		return OBSTACLE;
	}

	if (robot_position.block_detection)
	{
		printf("MOTOR_LOCKED\n");
		motion_brake();
		return MOTOR_LOCKED;
	}

	if (robot_position.path_done)
	{
		printf("PATH_DONE\n");
		return PATH_DONE;
	}

	return NOT_OCCURRED;
}

// ObstacleEvent
ObstacleEvent::ObstacleEvent()
{
}

void ObstacleEvent::printDescription() const
{
	printf("ObstacleEvent()");
}

WaitResult ObstacleEvent::testOccurred() const
{
	double obstacle_direction;
	ecan_update_object(ROBOT_POSITION_OBJECT);

	if (check_obstacle_detected(&obstacle_direction))
	{
		closePointsInDirection(obstacle_direction);
		printf("OBSTACLE\n");
		return OBSTACLE;
	}

	return NOT_OCCURRED;
}

// TimeoutEvent
TimeoutEvent::TimeoutEvent(int timeout_qsecs, int initial_game_timer)
: initial_game_timer(initial_game_timer), timeout_qsecs(timeout_qsecs)
{
}

void TimeoutEvent::printDescription() const
{
	printf("TimeoutEvent(%d)", timeout_qsecs);
}

WaitResult TimeoutEvent::testOccurred() const
{
	if (game_timer - initial_game_timer > timeout_qsecs)
	{
		printf("TIMEOUT\n");
		motion_brake(); // CHECKME: perché qui?
		return TIMEOUT_EXPIRED;
	}

	return NOT_OCCURRED;
}

// PlateBumperEvent
PlateBumperEvent::PlateBumperEvent(PlateBumperSide side)
:side(side)
{
}

void PlateBumperEvent::printDescription() const
{
	printf("PlateBumperEvent(");
	
	switch (side)
	{
		case BUMPERS_BOTH:
			printf("BOTH");
			break;
#if defined(ROBOT_GRANDE)
		case BUMPERS_LEFT:
			printf("LEFT");
			break;
		case BUMPERS_RIGHT:
			printf("RIGHT");
			break;
#else
		case BUMPERS_BACK:
			printf("BACK");
			break;
		case BUMPERS_FRONT:
			printf("FRONT");
			break;
#endif
	}
	printf(")");
	
}

WaitResult PlateBumperEvent::testOccurred() const
{
	double obstacle_direction;
	ecan_update_object(ROBOT_POSITION_OBJECT);

	if (check_obstacle_detected(&obstacle_direction))
	{
		closePointsInDirection(obstacle_direction);
		printf("OBSTACLE\n");
		return OBSTACLE;
	}

#if defined(ROBOT_GRANDE)
	if ((side == BUMPERS_BOTH && robot_position.left_bumpers && robot_position.right_bumpers)
		|| (side == BUMPERS_LEFT && robot_position.left_bumpers)
		|| (side == BUMPERS_RIGHT && robot_position.right_bumpers))
#else
	if ((side == BUMPERS_BOTH && robot_position.back_bumpers && robot_position.front_bumpers)
		|| (side == BUMPERS_BACK && robot_position.back_bumpers)
		|| (side == BUMPERS_FRONT && robot_position.front_bumpers))
#endif
	{
		printf("BUMPERS\n");
		motion_brake();
		return BUMPERS;
	}

	if (robot_position.block_detection)
	{
		printf("MOTOR_LOCKED\n");
		motion_brake();
		return MOTOR_LOCKED;
	}

	return NOT_OCCURRED;
}

// PositionTresholdEvent
PositionTresholdEvent::PositionTresholdEvent(PositionTresholdType type, double reference_value)
: type(type), reference_value(reference_value)
{
}

void PositionTresholdEvent::printDescription() const
{
	printf("PositionTresholdEvent(");
	switch (type)
	{
		case X_LESS_THAN:
			printf("X_LESS_THAN");
			break;
		case X_GREATER_THAN:
			printf("X_GREATER_THAN");
			break;
		case Y_LESS_THAN:
			printf("Y_LESS_THAN");
			break;
		case Y_GREATER_THAN:
			printf("Y_GREATER_THAN");
			break;
	}
	printf(", %f)", reference_value);
}

WaitResult PositionTresholdEvent::testOccurred() const
{
	Point p;
	bool reached;

	p = get_pos();

	switch (type)
	{
		case X_LESS_THAN:
			reached = (p.x < reference_value);
			break;
		case X_GREATER_THAN:
			reached = (p.x > reference_value);
			break;
		case Y_LESS_THAN:
			reached = (p.y < reference_value);
			break;
		case Y_GREATER_THAN:
			reached = (p.y > reference_value);
			break;
	}

	if (reached)
	{
		printf("POSITION_REACHED\n");
		return POSITION_REACHED;
	}

	return NOT_OCCURRED;
}


// PositionLockedEvent
PositionLockedEvent::PositionLockedEvent()
    : dx(2), dy(2), dtheta(0.5), lock_counter(0), lock_max(100)
{
}

void PositionLockedEvent::printDescription() const
{
	printf("PositionLockedEvent()");
}

WaitResult checkMotion(PositionLockedEvent * t)
{
	Point p;
        double current_theta;

        __delay_ms(5);

	p = get_pos(&current_theta);

        if (fabs(p.x - t->x) < t->dx && fabs(p.y - t->y) < t->dy &&
            fabs(current_theta - t->theta) < t->dtheta) { //TODO: Da normalizzare?
            ++t->lock_counter;
        }
        else
            t->lock_counter = 0;

        t->x = p.x;
        t->y = p.y;
        t->theta = current_theta;

        if (t->lock_counter == t->lock_max) {
		printf("POSITION LOCKED\n");
		return POSITION_LOCKED;
	}

	return NOT_OCCURRED;
}


WaitResult PositionLockedEvent::testOccurred() const
{
    return checkMotion((PositionLockedEvent *)this);
}



// DistanceTresholdEvent
DistanceTresholdEvent::DistanceTresholdEvent(const Point &pos, double r)
: pos(pos), r(r), r_squared(r * r)
{
}

void DistanceTresholdEvent::printDescription() const
{
	printf("DistanceTresholdEvent(%f, %f, %f)", pos.x, pos.y, r);
}

WaitResult DistanceTresholdEvent::testOccurred() const
{
	Point p = get_pos();

	if (distance2(p, pos) <= r_squared)
	{
		printf("NEAR_DISTANCE\n");
		return NEAR_DISTANCE;
	}

	return NOT_OCCURRED;
}

#ifdef ROBOT_PICCOLO
#include "servos.h"
// OmronSensorEvent
OmronSensorEvent::OmronSensorEvent(int side, bool status)
: side(side), status(status)
{
}

void OmronSensorEvent::printDescription() const
{
	printf("OmronSensorEvent(");

	switch (side)
	{
		case FRONT_SIDE:
			printf("FRONT");
			break;
		case REAR_SIDE:
			printf("REAR");
			break;
		case LEFT_SIDE:
			printf("LEFT");
			break;
		case RIGHT_SIDE:
			printf("RIGHT");
			break;
	}

	printf(", ");
	
	if(status)
		printf("TRUE");
	else
		printf("FALSE");

	printf(")");
}

WaitResult OmronSensorEvent::testOccurred() const
{
    switch (side) {
    case FRONT_SIDE:
        if (omron_front_status() == status) {
            printf("OMRON_FRONT_EVENT\n");
            return OMRON_SENSOR;
	}
        break;
    case REAR_SIDE:
        if (omron_rear_status() == status) {
            printf("OMRON_REAR_EVENT\n");
            return OMRON_SENSOR;
	}
        break;
    default:
        break;
    }
    return NOT_OCCURRED;
}



// BaumerSensorEvent
BaumerSensorEvent::BaumerSensorEvent(int side, bool status)
: side(side), status(status)
{
}

void BaumerSensorEvent::printDescription() const
{
	printf("BaumerSensorEvent(");

	switch (side)
	{
		case FRONT_SIDE:
			printf("FRONT");
			break;
		case REAR_SIDE:
			printf("REAR");
			break;
	}

	printf(", ");
	
	if(status)
		printf("TRUE");
	else
		printf("FALSE");

	printf(")");
}

WaitResult BaumerSensorEvent::testOccurred() const
{
    switch (side) {
    case FRONT_SIDE:
        if (baumer_front_status() == status) {
            printf("BAUMER_FRONT_EVENT\n");
            motion_brake();
            return OBSTACLE;
	}
        break;
    case REAR_SIDE:
        if (baumer_rear_status() == status) {
            printf("BAUMER_REAR_EVENT\n");
            motion_brake();
            return OBSTACLE;
	}
        break;
    default:
        break;
    }
    return NOT_OCCURRED;
}

#endif
