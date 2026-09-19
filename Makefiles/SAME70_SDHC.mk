# CoreN2G SAME70_SDHC Configuration Makefile

SAME70_SDHC_BUILD_DIR := SAME70_SDHC
SAME70_SDHC_TARGET := $(SAME70_SDHC_BUILD_DIR)/libCoreN2G.a
SAME70_SDHC_SRC_DIR := src


# Compiler flags - C
SAME70_SDHC_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee -mno-unaligned-access \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Werror=implicit \
	-fsingle-precision-constant \
	-Os \
	$(DEBUG_FLAGS)

# Compiler flags - C++
SAME70_SDHC_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee -mno-unaligned-access \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-Os \
	$(DEBUG_FLAGS)

# Defines - C only
SAME70_SDHC_C_DEFS := -D__SAME70Q20B__ -Dnoexcept= -DSUPPORT_CAN=0 -DSUPPORT_SDHC=1 -DSUPPORT_USB=0

# Defines - C++
SAME70_SDHC_CXX_DEFS := -D__SAME70Q20B__ -DSUPPORT_CAN=0 -DSUPPORT_SDHC=1 -DSUPPORT_USB=0

# Include paths
SAME70_SDHC_INCLUDES := \
	-I$(SAME70_SDHC_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAME70_SDHC_SRC_DIR) \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70 \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/SAME70 \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pio \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pmc \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/xdmac \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/utils \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/ioport \
	-I$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I../RRFLibraries/src

# Source files
SAME70_SDHC_EXCLUDE_DIRS := \
	$(SAME70_SDHC_SRC_DIR)/RP2040 \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/usbhs \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/SAM4S \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/SAM4E \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/adc \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/cmcc \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/crccu \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pdc \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/dmac \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/udp \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/uotghs \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/twi \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock/sam4s \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock/sam4e \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s \
	$(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e \
	$(SAME70_SDHC_SRC_DIR)/SAME5x_C21 \
	$(SAME70_SDHC_SRC_DIR)/STM32 \
	$(SAME70_SDHC_SRC_DIR)/STMCubeMX \
	$(SAME70_SDHC_SRC_DIR)/atmel \
	$(SAME70_SDHC_SRC_DIR)/arm

SAME70_SDHC_CSRC := $(filter-out $(addsuffix /%,$(SAME70_SDHC_EXCLUDE_DIRS)),$(call rwildcard,$(SAME70_SDHC_SRC_DIR),*.c))

SAME70_SDHC_CPPSRC := $(filter-out $(addsuffix /%,$(SAME70_SDHC_EXCLUDE_DIRS) $(SAME70_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/aes),$(call rwildcard,$(SAME70_SDHC_SRC_DIR),*.cpp))

# Object files
SAME70_SDHC_COBJ := $(patsubst $(SAME70_SDHC_SRC_DIR)/%.c,$(SAME70_SDHC_BUILD_DIR)/%.o,$(SAME70_SDHC_CSRC))
SAME70_SDHC_CXXOBJ := $(patsubst $(SAME70_SDHC_SRC_DIR)/%.cpp,$(SAME70_SDHC_BUILD_DIR)/%.o,$(SAME70_SDHC_CPPSRC))
SAME70_SDHC_OBJS := $(SAME70_SDHC_COBJ) $(SAME70_SDHC_CXXOBJ)

# Dependency files
SAME70_SDHC_DEPS := $(SAME70_SDHC_OBJS:.o=.d)

# Build target
$(SAME70_SDHC_TARGET): $(SAME70_SDHC_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

# Compilation rules
$(SAME70_SDHC_BUILD_DIR)/%.o: $(SAME70_SDHC_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(SAME70_SDHC_CFLAGS) $(SAME70_SDHC_C_DEFS) $(SAME70_SDHC_INCLUDES) -MMD -MP -o $@ $<

$(SAME70_SDHC_BUILD_DIR)/%.o: $(SAME70_SDHC_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(SAME70_SDHC_CXXFLAGS) $(SAME70_SDHC_CXX_DEFS) $(SAME70_SDHC_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAME70_SDHC_DEPS)

# Configuration target
SAME70_SDHC: $(SAME70_SDHC_TARGET)

.PHONY: SAME70_SDHC
