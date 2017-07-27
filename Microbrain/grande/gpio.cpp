#include "defines.h"
#include <xc.h>

#include "adc.h"
#include "gpio.h"
#include "color.h"

void gpio_init(void)
{
    STARTER_DIR = 1;
    COLOR_SEL_DIR = 1;
    GPIO_1_DIR = 0;
    FUNNY_ACTION_DIR = 0;
    FUNNY_ACTION = 0;
}

inline int get_color(void)
{
    return COLOR_SEL;
}

