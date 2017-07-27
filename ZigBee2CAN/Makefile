# Project settings
BUILDDIR := build
OUTPUT_FILE := ZigBee2CAN
TARGET_CHIP := 33FJ128MC802

SOURCEFILES += main.cpp
SOURCEFILES += bus_interface.cpp
SOURCEFILES += canstdio_coord.cpp
SOURCEFILES += circqueue.cpp
SOURCEFILES += clock.cpp
# SOURCEFILES += display_fwd.cpp
# SOURCEFILES += nba_fwd.cpp
SOURCEFILES += rtsp_coord.cpp
SOURCEFILES += simple_var_output.cpp
SOURCEFILES += uart.cpp
SOURCEFILES += xbee.cpp

SOURCEFILES += ax12_interface.cpp

# Versione minimale della libreria LZO (per decompressione dati rtsp)
# minilzo-2.06 (le versioni successive alla 2.06 non supportano più sizeof(int) == 2)
# Source: http://www.oberhumer.com/opensource/lzo/download/minilzo-2.06.tar.gz
# Disattiviamo tutte le funzionalità ad eccezione di "decompress_safe" (ovvero
# la decompressione con validazione dell'input, controllata dalla define
# MINILZO_CFG_SKIP_LZO1X_DECOMPRESS_SAFE)
CFLAGS += -Iminilzo-2.06
CFLAGS += -DMINILZO_CFG_SKIP_LZO_UTIL
CFLAGS += -DMINILZO_CFG_SKIP_LZO_STRING
CFLAGS += -DMINILZO_CFG_SKIP_LZO_INIT
CFLAGS += -DMINILZO_CFG_SKIP_LZO1X_1_COMPRESS
CFLAGS += -DMINILZO_CFG_SKIP_LZO1X_DECOMPRESS
CFLAGS += -DHAVE_AX12
CFLAGS += -DUART1_DISABLE_INTERRUPT
SOURCEFILES += minilzo-2.06/minilzo.c

# Librerie utilizzate
include ../libs/ecan_lib/ecan_lib.mk
include ../libs/c++support/c++support.mk
include ../libs/ax12_lib/ax12_lib.mk
include ../libs/peripheralslib/peripheralslib.mk

# Carica regole di compilazione. Deve essere incluso dopo aver riempito l'elenco SOURCEFILES
include ../rules.mk
