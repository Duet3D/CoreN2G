# CoreN2G SAM4E_SDHC_USB_RTOS Configuration Makefile
# Build configuration for SAM4E with SDHC and USB support (RTOS)

SAM4E_BUILD_DIR := SAM4E_SDHC_USB_RTOS
SAM4E_TARGET := $(SAM4E_BUILD_DIR)/libCoreN2G.a

# Source directories
SAM4E_SRC_DIR := src

# Find all source files (Eclipse excludes: src/RP2040|src/SAM4S_4E_E70/asf/common/services/clock/sam4s|
# src/SAM4S_4E_E70/asf/common/services/clock/same70|src/SAM4S_4E_E70/asf/sam/drivers/crccu|
# src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s|src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70|
# src/SAM4S_4E_E70/asf/sam/drivers/adc|src/SAM4S_4E_E70/asf/sam/drivers/aes|src/SAM4S_4E_E70/asf/sam/drivers/trng|
# src/SAM4S_4E_E70/asf/sam/drivers/usbhs|src/SAM4S_4E_E70/asf/sam/drivers/uotghs|src/SAM4S_4E_E70/asf/sam/drivers/twihs|
# src/SAM4S_4E_E70/asf/sam/drivers/mcan|src/SAM4S_4E_E70/asf/sam/drivers/xdmac|src/SAM4S_4E_E70/SAM4S|SAM4E|
# src/SAM4S_4E_E70/SAME70|src/SAME5x_C21|src/SAME70|src/SAME5x_C21/SAME5x/usb|src/SAME5x_C21/SAMC21|src/atmel|src/arm)
SAM4E_CPP_SRCS := $(shell find $(SAM4E_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAME70/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/SAME70/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAM4E_C_SRCS := $(shell find $(SAM4E_SRC_DIR) -name '*.c' \
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
	! -path '*/SAME5x_C21/*' \
	! -path '*/SAME70/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Include paths
SAM4E_INCLUDES := \
	-I$(SAM4E_SRC_DIR) \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70 \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/SAM4E \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/common/utils \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pmc \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device \
	-I$(SAM4E_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/udc \
	-I$(SAM4E_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I../RRFLibraries/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM4F \
	-I../LibTinyusb/src/tinyusb/src \
	-I../LibTinyusb/src

# Preprocessor defines
SAM4E_DEFINES := \
	-D__SAM4E8E__ \
	-DRTOS \
	-DSUPPORT_USB=1 \
	-DSUPPORT_SDHC=1

SAM4E_C_DEFINES := $(SAM4E_DEFINES) -Dnoexcept=

# Compiler flags - C
SAM4E_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m4 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-fsingle-precision-constant \
	$(SAM4E_INCLUDES) \
	$(SAM4E_C_DEFINES)

# Compiler flags - C++
SAM4E_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m4 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-fno-threadsafe-statics \
	-fno-rtti \
	-fno-exceptions \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-fsingle-precision-constant \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	$(SAM4E_INCLUDES) \
	$(SAM4E_DEFINES)

# Add debug flags if DEBUG=1
ifeq ($(DEBUG),1)
SAM4E_CFLAGS += -O0 -g3
SAM4E_CXXFLAGS += -O0 -g3
else
SAM4E_CFLAGS += -Os
SAM4E_CXXFLAGS += -Os
endif

# Object files
SAM4E_CPP_OBJS := $(SAM4E_CPP_SRCS:%.cpp=$(SAM4E_BUILD_DIR)/%.o)
SAM4E_C_OBJS := $(SAM4E_C_SRCS:%.c=$(SAM4E_BUILD_DIR)/%.o)
SAM4E_OBJS := $(SAM4E_CPP_OBJS) $(SAM4E_C_OBJS)

# Dependency files
SAM4E_DEPS := $(SAM4E_OBJS:.o=.d)

# Target rule
.PHONY: SAM4E_SDHC_USB_RTOS
SAM4E_SDHC_USB_RTOS: $(SAM4E_TARGET)

$(SAM4E_TARGET): $(SAM4E_OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

# Compile C++ files
$(SAM4E_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAM4E_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(SAM4E_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAM4E_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAM4E_DEPS)

# Clean target
.PHONY: clean-SAM4E_SDHC_USB_RTOS
clean-SAM4E_SDHC_USB_RTOS:
	$(Q)echo "  RM      $(SAM4E_BUILD_DIR)"
	$(Q)rm -rf $(SAM4E_BUILD_DIR)

