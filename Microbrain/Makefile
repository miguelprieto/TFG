# Project settings
BUILDDIR := build/$(ROBOT)
OUTPUT_FILE := MicroBrain_$(ROBOT)

CFLAGS = -I$(ROBOT) -I$(BUILDDIR)/generated -mlarge-code # -Wconversion
SOURCEFILES += adc.cpp
SOURCEFILES += bus_interface.cpp
SOURCEFILES += clocking.cpp
SOURCEFILES += console.cpp
SOURCEFILES += geometry.cpp
SOURCEFILES += goal_manager.cpp
SOURCEFILES += i2c.cpp
SOURCEFILES += main.cpp
SOURCEFILES += routing.cpp
SOURCEFILES += serial.cpp
SOURCEFILES += srf08.cpp
SOURCEFILES += wait.cpp

# Robot selection
ifeq ($(ROBOT),grande)
TARGET_CHIP := 33EP512MC502
CAN_RTSPPORT := 6661
SOURCEFILES += grande/controller_presets.cpp
SOURCEFILES += grande/goals.cpp
SOURCEFILES += grande/gpio.cpp
SOURCEFILES += grande/menu.cpp
SOURCEFILES += grande/pwm.cpp
SOURCEFILES += grande/servos.cpp
else ifeq ($(ROBOT),piccolo)
TARGET_CHIP := 33FJ128MC802
CAN_RTSPPORT := 6662
SOURCEFILES += piccolo/controller_presets.cpp
SOURCEFILES += piccolo/goals.cpp
SOURCEFILES += piccolo/gpio.cpp
SOURCEFILES += piccolo/menu.cpp
SOURCEFILES += piccolo/servos.cpp
else
$(error PLEASE USE "make ROBOT=grande" or "make ROBOT=piccolo")
endif

# Conversione del file .graph in codice C++
$(BUILDDIR)/generated/graph_definitions.gen $(BUILDDIR)/generated/graph_data.gen: $(ROBOT)/routing.graph
	@echo "=> Generating C++ code from $<..."
	@mkdir -p $(BUILDDIR)/generated
	@python ../../../tools/routing_graph/codeGenerator/graph2cpp.py $< $(BUILDDIR)/generated/

# Librerie utilizzate
CAN_STDIO_AND_RTSP_NODE_ID=2
include ../libs/can_stdio_and_rtsp/can_stdio_and_rtsp_endpoint.mk
include ../libs/ecan_lib/ecan_lib.mk
include ../libs/c++support/c++support.mk
include ../libs/simple_var_output/simple_var_output.mk

# Carica regole di compilazione. Deve essere incluso dopo aver riempito l'elenco SOURCEFILES
REQUIRED_TARGETS += $(BUILDDIR)/generated/graph_definitions.gen
REQUIRED_TARGETS += $(BUILDDIR)/generated/graph_data.gen
include ../rules.mk
