# VEXcode makefile

VERBOSE = 0

include vex/mkenv.mk

SRC_C  = $(wildcard src/*.cpp)
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard custom/src/*.cpp)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

SRC_H  = $(wildcard include/*.h)
SRC_H += $(wildcard custom/include/*.h)
SRC_A  = makefile

INC_F  = include custom/include

# build targets
all: $(BUILD)/$(PROJECT).bin

include vex/mkrules.mk
