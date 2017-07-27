# Ottiene il percorso di questa cartella
AX12LIBDIR := $(dir $(lastword $(MAKEFILE_LIST)))

CFLAGS += -I$(AX12LIBDIR)
SOURCEFILES += $(AX12LIBDIR)/ax12.cpp
