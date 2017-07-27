#ifndef __CONTROLLER_PRESETS_H
#define __CONTROLLER_PRESETS_H

#include "bus_interface.h"

extern const DistanceControllerParameters distance_slow;
extern const DistanceControllerParameters distance_default;
extern const DistanceControllerParameters distance_very_fast;
extern const DistanceControllerParameters distance_fast;

extern const LineControllerParameters line_default;

extern const HeadingControllerParameters heading_slow;
extern const HeadingControllerParameters heading_default;
/* extern const CircularRotationControllerParameters circular_rotation_lento; */
extern const CircularRotationControllerParameters circular_rotation_default;


#endif
