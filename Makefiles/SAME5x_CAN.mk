# CoreN2G SAME5x_CAN Configuration Makefile

SAME5x_CAN_BUILD_DIR := SAME5x_CAN
SAME5x_CAN_TARGET := $(SAME5x_CAN_BUILD_DIR)/libCoreN2G.a
SAME5x_CAN_SRC_DIR := src


# Compiler flags - C
SAME5x_CAN_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Werror=implicit \
	-fsingle-precision-constant \
	-Os

# Compiler flags - C++
SAME5x_CAN_CXXFLAGS := -c -std=gnu++17 \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-fsingle-precision-constant \
	-Os

# Defines - C only
SAME5x_CAN_C_DEFS := -D__SAME54P20A__ -Dnoexcept= -DSUPPORT_CAN=1 -DSUPPORT_SDHC=0 -DSUPPORT_USB=0

# Defines - C++
SAME5x_CAN_CXX_DEFS := -D__SAME54P20A__ -DSUPPORT_CAN=1 -DSUPPORT_SDHC=0 -DSUPPORT_USB=0

# Include paths
SAME5x_CAN_INCLUDES := \
	-I$(SAME5x_CAN_SRC_DIR)/atmel/SAME54_DFP/1.1.134/include \
	-I$(SAME5x_CAN_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAME5x_CAN_SRC_DIR) \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/pukcc \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/hal/include \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/hri \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/Config \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/usb \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/usb/device \
	-I$(SAME5x_CAN_SRC_DIR)/SAME5x_C21/SAME5x/usb/class/cdc \
	-I../RRFLibraries/src \
	-I../CANlib/src

# Source files
SAME5x_CAN_CSRC := $(shell find $(SAME5x_CAN_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAMC21/*' \
	! -path '*/SAME5x_C21/SAME5x/usb/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAME5x_CAN_CPPSRC := $(shell find $(SAME5x_CAN_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAMC21/*' \
	! -path '*/SAME5x_C21/SAME5x/usb/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Object files
SAME5x_CAN_COBJ := $(patsubst $(SAME5x_CAN_SRC_DIR)/%.c,$(SAME5x_CAN_BUILD_DIR)/%.o,$(SAME5x_CAN_CSRC))
SAME5x_CAN_CXXOBJ := $(patsubst $(SAME5x_CAN_SRC_DIR)/%.cpp,$(SAME5x_CAN_BUILD_DIR)/%.o,$(SAME5x_CAN_CPPSRC))
SAME5x_CAN_OBJS := $(SAME5x_CAN_COBJ) $(SAME5x_CAN_CXXOBJ)

# Dependency files
SAME5x_CAN_DEPS := $(SAME5x_CAN_OBJS:.o=.d)

# Build target
$(SAME5x_CAN_TARGET): $(SAME5x_CAN_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

# Compilation rules
$(SAME5x_CAN_BUILD_DIR)/%.o: $(SAME5x_CAN_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(SAME5x_CAN_CFLAGS) $(SAME5x_CAN_C_DEFS) $(SAME5x_CAN_INCLUDES) -MMD -MP -o $@ $<

$(SAME5x_CAN_BUILD_DIR)/%.o: $(SAME5x_CAN_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(SAME5x_CAN_CXXFLAGS) $(SAME5x_CAN_CXX_DEFS) $(SAME5x_CAN_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAME5x_CAN_DEPS)

# Configuration target
SAME5x_CAN: $(SAME5x_CAN_TARGET)

.PHONY: SAME5x_CAN
