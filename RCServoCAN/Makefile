# Project settings
BUILDDIR := build/$(ROBOT)
OUTPUT_FILE := RCServoCAN_$(ROBOT)
TARGET_CHIP := 33FJ128MC802

# Robot selection
ifeq ($(ROBOT),grande)
CAN_STDIO_AND_RTSP_NODE_ID := 4
CAN_RTSPPORT := 6661
SOURCEFILES += grande_automation.cpp
CFLAGS += -DSERVO_BASE=0
# else ifeq ($(ROBOT),grande2)
# CAN_STDIO_AND_RTSP_NODE_ID := 5
# CAN_RTSPPORT := 6661
# SOURCEFILES += grande2_automation.cpp
# CFLAGS += -DSERVO_BASE=10
else ifeq ($(ROBOT),piccolo)
CAN_STDIO_AND_RTSP_NODE_ID := 4
CAN_RTSPPORT := 6662
SOURCEFILES += piccolo_automation.cpp
CFLAGS += -DSERVO_BASE=0
else
$(error PLEASE USE "make ROBOT=grande" or "make ROBOT=piccolo")
endif


SOURCEFILES += main.cpp
SOURCEFILES += clocking.cpp
SOURCEFILES += servo.cpp
SOURCEFILES += sensors.cpp

# Librerie utilizzate
include ../libs/can_stdio_and_rtsp/can_stdio_and_rtsp_endpoint.mk
include ../libs/ecan_lib/ecan_lib.mk
include ../libs/c++support/c++support.mk

# Carica regole di compilazione. Deve essere incluso dopo aver riempito l'elenco SOURCEFILES
include ../rules.mk
