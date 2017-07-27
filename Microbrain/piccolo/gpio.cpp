#include "defines.h"
#include <xc.h>

#include "gpio.h"

void gpio_init(void)
{
    GPIO_1_DIR = 0;
    GPIO_2_DIR = 0;
    GPIO_10_DIR = 0;
    GPIO_12_DIR = 0;

    STARTER_DIR = 1;
    COLOR_SEL_DIR = 1;

    OMRON_LEFT_DIR = 1;
    OMRON_RIGHT_DIR = 1;
    OBSTACLE_FRONT_DIR = 1;
    OBSTACLE_REAR_DIR = 1;
}

void sucker_front(int v)
{
    GPIO_12 = v;
    GPIO_1 = v;
}

void sucker_rear(int v)
{
    GPIO_10 = v;
    GPIO_2 = v;
}

