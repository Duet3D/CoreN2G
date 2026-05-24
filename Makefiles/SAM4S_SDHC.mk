# CoreN2G SAM4S_SDHC Configuration Makefile

SAM4S_SDHC_BUILD_DIR := SAM4S_SDHC
SAM4S_SDHC_TARGET := $(SAM4S_SDHC_BUILD_DIR)/libCoreN2G.a
SAM4S_SDHC_SRC_DIR := src


# Compiler flags - C  
SAM4S_SDHC_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m4 -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Werror=implicit \
	-fsingle-precision-constant \
	-Os

# Compiler flags - C++
SAM4S_SDHC_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m4 -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-Os

# Defines - C only
SAM4S_SDHC_C_DEFS := -D__SAM4S8C__ -Dnoexcept= -DSUPPORT_SDHC=1 -DSUPPORT_USB=0

# Defines - C++
SAM4S_SDHC_CXX_DEFS := -D__SAM4S8C__ -DSUPPORT_SDHC=1 -DSUPPORT_USB=0

# Include paths
SAM4S_SDHC_INCLUDES := \
	-I$(SAM4S_SDHC_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAM4S_SDHC_SRC_DIR) \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70 \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/SAM4S \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pmc \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/include \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/utils \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device \
	-I$(SAM4S_SDHC_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/udc \
	-I../RRFLibraries/src

# Source files
SAM4S_SDHC_CSRC := $(shell find $(SAM4S_SDHC_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4e/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/same70/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/aes/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/cmcc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/dmac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/afec/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/emac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/gmac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/rswdt/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/trng/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/usbhs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/uotghs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/twihs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/mcan/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/xdmac/*' \
	! -path '*/SAM4S_4E_E70/SAM4E/*' \
	! -path '*/SAM4S_4E_E70/SAME70/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/SAME70/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAM4S_SDHC_CPPSRC := $(shell find $(SAM4S_SDHC_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4e/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/same70/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/aes/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/cmcc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/dmac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/afec/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/emac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/gmac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/rswdt/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/trng/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/usbhs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/uotghs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/twihs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/mcan/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/xdmac/*' \
	! -path '*/SAM4S_4E_E70/SAM4E/*' \
	! -path '*/SAM4S_4E_E70/SAME70/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/SAME70/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Object files
SAM4S_SDHC_COBJ := $(patsubst $(SAM4S_SDHC_SRC_DIR)/%.c,$(SAM4S_SDHC_BUILD_DIR)/%.o,$(SAM4S_SDHC_CSRC))
SAM4S_SDHC_CXXOBJ := $(patsubst $(SAM4S_SDHC_SRC_DIR)/%.cpp,$(SAM4S_SDHC_BUILD_DIR)/%.o,$(SAM4S_SDHC_CPPSRC))
SAM4S_SDHC_OBJS := $(SAM4S_SDHC_COBJ) $(SAM4S_SDHC_CXXOBJ)

# Dependency files
SAM4S_SDHC_DEPS := $(SAM4S_SDHC_OBJS:.o=.d)

# Build target
$(SAM4S_SDHC_TARGET): $(SAM4S_SDHC_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

# Compilation rules
$(SAM4S_SDHC_BUILD_DIR)/%.o: $(SAM4S_SDHC_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(SAM4S_SDHC_CFLAGS) $(SAM4S_SDHC_C_DEFS) $(SAM4S_SDHC_INCLUDES) -MMD -MP -o $@ $<

$(SAM4S_SDHC_BUILD_DIR)/%.o: $(SAM4S_SDHC_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(SAM4S_SDHC_CXXFLAGS) $(SAM4S_SDHC_CXX_DEFS) $(SAM4S_SDHC_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAM4S_SDHC_DEPS)

# Configuration target
SAM4S_SDHC: $(SAM4S_SDHC_TARGET)

.PHONY: SAM4S_SDHC
