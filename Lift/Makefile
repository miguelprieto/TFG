# Project settings
BUILDDIR := build
OUTPUT_FILE := Lift
TARGET_CHIP := 33FJ128MC802

CFLAGS += -DUSE_CAN

SOURCEFILES += main.c
SOURCEFILES += bus_interface.c
SOURCEFILES += clocking.c
SOURCEFILES += controller.c
SOURCEFILES += velocity.c
SOURCEFILES += position.c
SOURCEFILES += pwm_control.c
SOURCEFILES += qei.c
SOURCEFILES += timers.c

# Librerie utilizzate
CAN_RTSPPORT := 6661
CAN_STDIO_AND_RTSP_NODE_ID := 6
include ../../2016/libs/can_stdio_and_rtsp/can_stdio_and_rtsp_endpoint.mk
include ../../2016/libs/ecan_lib/ecan_lib.mk

# Carica regole di compilazione. Deve essere incluso dopo aver riempito l'elenco SOURCEFILES
include ../rules.mk
