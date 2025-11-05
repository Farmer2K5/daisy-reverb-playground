# Daisy Patch SM Project
TARGET = Reverb_Playground
# APP_TYPE = BOOT_SRAM

# Sources - Delays
# CPP_SOURCES := example_firefly_delay_lite.cpp
# CPP_SOURCES := example_firefly_delay_perf.cpp
# CPP_SOURCES := example_firefly_delay_perf_v3.cpp

# Sources - Reverbs
# CPP_SOURCES := example_fdn_tank.cpp
# CPP_SOURCES := example_fdn_reverb.cpp
# CPP_SOURCES := example_firefly_plate_reverb.cpp
# CPP_SOURCES := example_lush_plate_reverb.cpp
CPP_SOURCES := example_nebula_hall_tank.cpp

# Other Source Files
CPP_SOURCES += $(shell find src -name "*.cpp")

# Includes
INC_ROOT = ./include
SUBDIRS := $(shell find $(INC_ROOT) -maxdepth 5 -type d)
C_INCLUDES := $(patsubst %, -I%, $(SUBDIRS))

# (optional) Includes DaisySP-LGPL (like ReverbSc, etc.) source files within project.
#USE_DAISYSP_LGPL=1

# (optional) Includes FatFS source files within project.
#USE_FATFS = 1

# Path to DaisySP and LibDaisy 
DAISYSP_DIR ?= ../lib/DaisySP
LIBDAISY_DIR ?= ../lib/libDaisy

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
