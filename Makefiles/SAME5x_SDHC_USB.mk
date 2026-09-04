# CoreN2G SAME5x_SDHC_USB Configuration Makefile
# Based on SAME5x_SDHC with USB CDC support added (no RTOS, for IAP)

SAME5x_SDHC_USB_BUILD_DIR := SAME5x_SDHC_USB
SAME5x_SDHC_USB_TARGET := $(SAME5x_SDHC_USB_BUILD_DIR)/libCoreN2G.a
SAME5x_SDHC_USB_SRC_DIR := src


# Compiler flags - C
SAME5x_SDHC_USB_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Werror=implicit \
	-fsingle-precision-constant \
	-Os \
	$(DEBUG_FLAGS)

# Compiler flags - C++
SAME5x_SDHC_USB_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-Os \
	$(DEBUG_FLAGS)

# Defines - C only (same as SAME5x_SDHC but with SUPPORT_USB=1, IAP descriptor overrides)
SAME5x_SDHC_USB_C_DEFS := -D__SAME54P20A__ -Dnoexcept= -DSUPPORT_CAN=0 -DSUPPORT_SDHC=1 -DSUPPORT_USB=1 \
	-DCONF_USB_CDCD_ACM_IDPRODUCT=0x60ef \
	-DCONF_USB_CDCD_ACM_IPRODUCT_EN=1 \
	'-DCONF_USB_CDCD_ACM_IPRODUCT_STR="IAP"'

# Defines - C++
SAME5x_SDHC_USB_CXX_DEFS := -D__SAME54P20A__ -DSUPPORT_CAN=0 -DSUPPORT_SDHC=1 -DSUPPORT_USB=1 \
	-DCONF_USB_CDCD_ACM_IDPRODUCT=0x60ef \
	-DCONF_USB_CDCD_ACM_IPRODUCT_EN=1 \
	'-DCONF_USB_CDCD_ACM_IPRODUCT_STR="IAP"'

# Include paths (same as SAME5x_SDHC plus USB HAL paths)
SAME5x_SDHC_USB_INCLUDES := \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/atmel/SAME54_DFP/1.1.134/include \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAME5x_SDHC_USB_SRC_DIR) \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/pukcc \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/hal/include \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/hri \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/Config \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/usb \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/usb/device \
	-I$(SAME5x_SDHC_USB_SRC_DIR)/SAME5x_C21/SAME5x/usb/class/cdc \
	-I../RRFLibraries/src

# Source files: same as SAME5x_SDHC but include USB sources (remove the usb exclusion)
SAME5x_SDHC_USB_CSRC := $(shell find $(SAME5x_SDHC_USB_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAMC21/*' \
	! -path '*/STM32/*' \
	! -path '*/STMCubeMX/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAME5x_SDHC_USB_CPPSRC := $(shell find $(SAME5x_SDHC_USB_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAMC21/*' \
	! -path '*/STM32/*' \
	! -path '*/STMCubeMX/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Object files
SAME5x_SDHC_USB_COBJ := $(patsubst $(SAME5x_SDHC_USB_SRC_DIR)/%.c,$(SAME5x_SDHC_USB_BUILD_DIR)/%.o,$(SAME5x_SDHC_USB_CSRC))
SAME5x_SDHC_USB_CXXOBJ := $(patsubst $(SAME5x_SDHC_USB_SRC_DIR)/%.cpp,$(SAME5x_SDHC_USB_BUILD_DIR)/%.o,$(SAME5x_SDHC_USB_CPPSRC))
SAME5x_SDHC_USB_OBJS := $(SAME5x_SDHC_USB_COBJ) $(SAME5x_SDHC_USB_CXXOBJ)

# Dependency files
SAME5x_SDHC_USB_DEPS := $(SAME5x_SDHC_USB_OBJS:.o=.d)

# Build target
$(SAME5x_SDHC_USB_TARGET): $(SAME5x_SDHC_USB_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

# Compilation rules
$(SAME5x_SDHC_USB_BUILD_DIR)/%.o: $(SAME5x_SDHC_USB_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(SAME5x_SDHC_USB_CFLAGS) $(SAME5x_SDHC_USB_C_DEFS) $(SAME5x_SDHC_USB_INCLUDES) -MMD -MP -o $@ $<

$(SAME5x_SDHC_USB_BUILD_DIR)/%.o: $(SAME5x_SDHC_USB_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(SAME5x_SDHC_USB_CXXFLAGS) $(SAME5x_SDHC_USB_CXX_DEFS) $(SAME5x_SDHC_USB_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAME5x_SDHC_USB_DEPS)

# Configuration target
SAME5x_SDHC_USB: $(SAME5x_SDHC_USB_TARGET)

.PHONY: SAME5x_SDHC_USB
