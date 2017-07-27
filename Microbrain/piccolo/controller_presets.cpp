#include "controller_presets.h"

#define VERY_FAST_SPEED      500
#define VERY_FAST_ACCEL      500
#define VERY_FAST_DECEL      400

#define FAST_SPEED           700
#define FAST_ACCEL           500
#define FAST_DECEL           400

#define NORMAL_SPEED         400 //-600
#define NORMAL_ACCEL         800
#define NORMAL_DECEL         800

#define SLOW_SPEED           200
#define SLOW_ACCEL           200
#define SLOW_DECEL           200

#define NORMAL_ROT_SPEED      400 
#define NORMAL_ROT_ACCEL      800
#define NORMAL_ROT_DECEL      700

#define SLOW_ROT_SPEED   200
#define SLOW_ROT_ACCEL   200
#define SLOW_ROT_DECEL   200

/* BEGIN DEFAULT SPEED SETUP */
static LineControllerParameters init_line_default()
{
	LineControllerParameters p;
	// p.max_speed = FAST_SPEED;
	// p.accel = FAST_ACCEL;
	// p.decel = FAST_DECEL;
	// p.kdm = 0; p.kde = 0;
        // ^^^^^^ No more used ^^^^^^
	p.kp_h_m = 4; p.kp_h_e = 2;
	p.kp_line_m = 6; p.kp_line_e = -1;
	p.change_threshold = 200; // -50

	return p;
}

static DistanceControllerParameters init_distance_default()
{
	DistanceControllerParameters p;
	p.max_speed = NORMAL_SPEED;
	p.accel = NORMAL_ACCEL;
	p.decel = NORMAL_DECEL;
	p.kdm = 0; p.kde = 0;
	
	return p;
}

static HeadingControllerParameters init_heading_default()
{
	HeadingControllerParameters p;
	p.max_speed = NORMAL_ROT_SPEED;
	p.accel = NORMAL_ROT_ACCEL;
	p.decel = NORMAL_ROT_DECEL;
	p.kdm = 0; p.kde = 0;
	
	return p;
}

static CircularRotationControllerParameters init_circular_rotation_default()
{
	CircularRotationControllerParameters p;
	p.max_speed = 500;
	p.accel = 800;
	p.decel = 800;
	p.kdm = 0; p.kde = 0;
	p.kp_dist_m = 5; p.kp_dist_e = -3;
	p.kp_hdg_m = 1; p.kp_hdg_e = 2;
	
	return p;
}
/* END DEFAULT SPEED SETUP */



/* BEGIN SLOW SPEED SETUP */
static DistanceControllerParameters init_distance_slow()
{
	DistanceControllerParameters p;
	p.max_speed = SLOW_SPEED;
	p.accel = SLOW_ACCEL;
	p.decel = SLOW_DECEL;
	p.kdm = 0; p.kde = 0;
	
	return p;
}

static HeadingControllerParameters init_heading_slow()
{
	HeadingControllerParameters p;
	p.max_speed = SLOW_ROT_SPEED;
	p.accel = SLOW_ROT_ACCEL;
	p.decel = SLOW_ROT_DECEL;
	p.kdm = 0; p.kde = 0;
	
	return p;
}
/* END SLOW SPEED SETUP */




// static CircularRotationControllerParameters init_circular_rotation_slow() //FIXME Sistemare velocità slow
// {
// 	CircularRotationControllerParameters p;
// 	p.max_speed = 200;
// 	p.accel = 800;
// 	p.decel = 700;
// 	p.kdm = 0; p.kde = 0;
// 	p.kp_dist_m = 5; p.kp_dist_e = -3;
// 	p.kp_hdg_m = 1; p.kp_hdg_e = 2;
	
// 	return p;
// }



/* BEGIN FAST SPEED SETUP */
static DistanceControllerParameters init_distance_fast()
{
	DistanceControllerParameters p;
	p.max_speed = FAST_SPEED;
	p.accel = FAST_ACCEL;
	p.decel = FAST_DECEL;
	p.kdm = 0; p.kde = 0;
	
	return p;
}
/* END FAST SPEED SETUP */


/* BEGIN VERY FAST SPEED SETUP */
static DistanceControllerParameters init_distance_very_fast()
{
	DistanceControllerParameters p;
	p.max_speed = VERY_FAST_SPEED;
	p.accel = VERY_FAST_ACCEL;
	p.decel = VERY_FAST_DECEL;
	p.kdm = 0; p.kde = 0;
	
	return p;
}
/* END VERY FAST SPEED SETUP */



const LineControllerParameters line_default(init_line_default());

const DistanceControllerParameters distance_slow(init_distance_slow());
const HeadingControllerParameters heading_slow(init_heading_slow());

const DistanceControllerParameters distance_default(init_distance_default());
const HeadingControllerParameters heading_default(init_heading_default());

const DistanceControllerParameters distance_very_fast(init_distance_very_fast());

const DistanceControllerParameters distance_fast(init_distance_fast());

const CircularRotationControllerParameters circular_rotation_default(init_circular_rotation_default());

