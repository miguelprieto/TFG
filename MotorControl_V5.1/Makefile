# Project settings
BUILDDIR := build/$(ROBOT)
OUTPUT_FILE := MotionControl_$(ROBOT)
TARGET_CHIP := 33FJ128MC802

SOURCEFILES += main.c
SOURCEFILES += bus_interface.c
SOURCEFILES += clocking.c
SOURCEFILES += controller.c
SOURCEFILES += odometry.c
SOURCEFILES += path.c
SOURCEFILES += position.c
SOURCEFILES += pwm_control.c
SOURCEFILES += qei.c
SOURCEFILES += timers.c
SOURCEFILES += velocity.c

# Robot selection
ifeq ($(ROBOT),grande)
CAN_RTSPPORT := 6661
CFLAGS = -DROBOT_GRANDE
else ifeq ($(ROBOT),piccolo)
CAN_RTSPPORT := 6662
CFLAGS = -DROBOT_PICCOLO
else
$(error PLEASE USE "make ROBOT=grande" or "make ROBOT=piccolo")
endif

# Librerie utilizzate
CAN_STDIO_AND_RTSP_NODE_ID := 3
include ../libs/can_stdio_and_rtsp/can_stdio_and_rtsp_endpoint.mk
include ../libs/ecan_lib/ecan_lib.mk
include ../libs/simple_var_output/simple_var_output.mk

# Carica regole di compilazione. Deve essere incluso dopo aver riempito l'elenco SOURCEFILES
include ../rules.mk
