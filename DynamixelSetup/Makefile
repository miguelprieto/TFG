# Project settings
BUILDDIR := build/
OUTPUT_FILE := Dynamixel
TARGET_CHIP := 33FJ128MC802

# Robot selection
CAN_STDIO_AND_RTSP_NODE_ID := 10
CAN_RTSPPORT := 6661

SOURCEFILES += clocking.cpp
SOURCEFILES += timers.cpp
SOURCEFILES += main.cpp

CFLAGS += -mlarge-code -mlarge-data
#CFLAGS += -S

# Librerie utilizzate
include ../libs/controllib/controllib.mk
include ../libs/peripheralslib/peripheralslib.mk
include ../libs/c++support/c++support.mk
include ../libs/ecan_lib/ecan_lib.mk
include ../libs/can_stdio_and_rtsp/can_stdio_and_rtsp_endpoint.mk
include ../libs/ax12_lib/ax12_lib.mk

# Carica regole di compilazione. Deve essere incluso dopo aver riempito l'elenco SOURCEFILES
include ../rules.mk

#override LDSCRIPT = ./p33FJ128MC802-cdrone.gld

