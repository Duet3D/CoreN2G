# CoreN2G SAME70_CAN Configuration Makefile

SAME70_CAN_BUILD_DIR := SAME70_CAN
SAME70_CAN_TARGET := $(SAME70_CAN_BUILD_DIR)/libCoreN2G.a
SAME70_CAN_SRC_DIR := src


# Compiler flags - C
SAME70_CAN_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee -mno-unaligned-access \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Werror=implicit \
	-fsingle-precision-constant \
	-O3

# Compiler flags - C++
SAME70_CAN_CXXFLAGS := -c -std=gnu++17 \
	-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee -mno-unaligned-access \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-fsingle-precision-constant \
	-O3

# Defines - C only
SAME70_CAN_C_DEFS := -D__SAME70Q20B__ -Dnoexcept= -DSUPPORT_CAN=1 -DSUPPORT_SDHC=0 -DSUPPORT_USB=0

# Defines - C++
SAME70_CAN_CXX_DEFS := -D__SAME70Q20B__ -DSUPPORT_CAN=1 -DSUPPORT_SDHC=0 -DSUPPORT_USB=0

# Include paths
SAME70_CAN_INCLUDES := \
	-I$(SAME70_CAN_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAME70_CAN_SRC_DIR) \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70 \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/SAME70 \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pio \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pmc \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/xdmac \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/utils \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/ioport \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/udc \
	-I$(SAME70_CAN_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I../RRFLibraries/src \
	-I../CANlib/src

# Source files
SAME70_CAN_CSRC := $(shell find $(SAME70_CAN_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAM4E/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/adc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/cmcc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/crccu/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/pdc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/dmac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/udp/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/uotghs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/twi/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4e/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAME70_CAN_CPPSRC := $(shell find $(SAME70_CAN_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAM4E/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/aes/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/adc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/cmcc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/crccu/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/pdc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/dmac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/udp/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/uotghs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/twi/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4e/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Object files
SAME70_CAN_COBJ := $(patsubst $(SAME70_CAN_SRC_DIR)/%.c,$(SAME70_CAN_BUILD_DIR)/%.o,$(SAME70_CAN_CSRC))
SAME70_CAN_CXXOBJ := $(patsubst $(SAME70_CAN_SRC_DIR)/%.cpp,$(SAME70_CAN_BUILD_DIR)/%.o,$(SAME70_CAN_CPPSRC))
SAME70_CAN_OBJS := $(SAME70_CAN_COBJ) $(SAME70_CAN_CXXOBJ)

# Dependency files
SAME70_CAN_DEPS := $(SAME70_CAN_OBJS:.o=.d)

# Build target
$(SAME70_CAN_TARGET): $(SAME70_CAN_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

# Compilation rules
$(SAME70_CAN_BUILD_DIR)/%.o: $(SAME70_CAN_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(SAME70_CAN_CFLAGS) $(SAME70_CAN_C_DEFS) $(SAME70_CAN_INCLUDES) -MMD -MP -o $@ $<

$(SAME70_CAN_BUILD_DIR)/%.o: $(SAME70_CAN_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(SAME70_CAN_CXXFLAGS) $(SAME70_CAN_CXX_DEFS) $(SAME70_CAN_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAME70_CAN_DEPS)

# Configuration target
SAME70_CAN: $(SAME70_CAN_TARGET)

.PHONY: SAME70_CAN
