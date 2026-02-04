# CoreN2G SAM4E_SDHC Configuration Makefile

SAM4E_SDHC_BUILD_DIR := SAM4E_SDHC
SAM4E_SDHC_TARGET := $(SAM4E_SDHC_BUILD_DIR)/libCoreN2G.a
SAM4E_SDHC_SRC_DIR := src


# Compiler flags - C
SAM4E_SDHC_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Werror=implicit \
	-fsingle-precision-constant \
	-Os

# Compiler flags - C++
SAM4E_SDHC_CXXFLAGS := -c -std=gnu++17 \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-fsingle-precision-constant \
	-Os

# Defines - C only
SAM4E_SDHC_C_DEFS := -D__SAM4E8E__ -Dnoexcept= -DSUPPORT_SDHC=1 -DSUPPORT_USB=0

# Defines - C++
SAM4E_SDHC_CXX_DEFS := -D__SAM4E8E__ -DSUPPORT_SDHC=1 -DSUPPORT_USB=0

# Include paths
SAM4E_SDHC_INCLUDES := \
	-I$(SAM4E_SDHC_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAM4E_SDHC_SRC_DIR) \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70 \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/SAM4E \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/utils \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pmc \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device \
	-I$(SAM4E_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/udc \
	-I../RRFLibraries/src

# Source files
SAM4E_SDHC_CSRC := $(shell find $(SAM4E_SDHC_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAME70/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/same70/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/crccu/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/adc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/aes/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/trng/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/usbhs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/uotghs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/twihs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/mcan/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/xdmac/*' \
	! -path '*/SAM4E/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/SAME70/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAM4E_SDHC_CPPSRC := $(shell find $(SAM4E_SDHC_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAME70/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/SAME70/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Object files
SAM4E_SDHC_C_OBJS := $(SAM4E_SDHC_CSRC:%.c=$(SAM4E_SDHC_BUILD_DIR)/%.o)
SAM4E_SDHC_CPP_OBJS := $(SAM4E_SDHC_CPPSRC:%.cpp=$(SAM4E_SDHC_BUILD_DIR)/%.o)
SAM4E_SDHC_OBJS := $(SAM4E_SDHC_C_OBJS) $(SAM4E_SDHC_CPP_OBJS)

# Dependency files
SAM4E_SDHC_DEPS := $(SAM4E_SDHC_OBJS:.o=.d)

# Target rule
.PHONY: SAM4E_SDHC clean-SAM4E_SDHC

SAM4E_SDHC: $(SAM4E_SDHC_TARGET)

$(SAM4E_SDHC_TARGET): $(SAM4E_SDHC_OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

# Compile C++ files
$(SAM4E_SDHC_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAM4E_SDHC_CXXFLAGS) $(SAM4E_SDHC_CXX_DEFS) $(SAM4E_SDHC_INCLUDES) -MMD -MP -o $@ $<

# Compile C files
$(SAM4E_SDHC_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAM4E_SDHC_CFLAGS) $(SAM4E_SDHC_C_DEFS) $(SAM4E_SDHC_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAM4E_SDHC_DEPS)

# Clean target
clean-SAM4E_SDHC:
	$(Q)echo "  RM      $(SAM4E_SDHC_BUILD_DIR)"
	$(Q)rm -rf $(SAM4E_SDHC_BUILD_DIR)
