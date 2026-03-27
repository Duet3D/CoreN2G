# CoreN2G SAME5x_CAN_SDHC_USB_RTOS Configuration Makefile
# Build configuration for SAME5x with CAN, SDHC and USB support (RTOS)

SAME5X_BUILD_DIR := SAME5x_CAN_SDHC_USB_RTOS
SAME5X_TARGET := $(SAME5X_BUILD_DIR)/libCoreN2G.a

# Source directories
SAME5X_SRC_DIR := src

# Find all source files (Eclipse excludes: src/RP2040|src/SAM4S_4E_E70|src/SAME5x_C21/SAMC21|src/atmel|src/arm)
SAME5X_CPP_SRCS := $(shell find $(SAME5X_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAMC21/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAME5X_C_SRCS := $(shell find $(SAME5X_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAMC21/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Include paths (matching Eclipse .cproject order)
SAME5X_INCLUDES := \
	-I$(SAME5X_SRC_DIR) \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/pukcc \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/hal/include \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/hri \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/Config \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/usb \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/usb/device \
	-I$(SAME5X_SRC_DIR)/SAME5x_C21/SAME5x/usb/class/cdc \
	-I$(SAME5X_SRC_DIR)/atmel/SAME54_DFP/1.1.134/include \
	-I$(SAME5X_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I../RRFLibraries/src \
	-I../CANlib/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM4F \
	-I../LibTinyusb/src/tinyusb/src \
	-I../LibTinyusb/src

# Preprocessor defines
SAME5X_DEFINES := \
	-D__SAME54P20A__ \
	-DRTOS \
	-DSUPPORT_CAN=1 \
	-DSUPPORT_SDHC=1 \
	-DSUPPORT_USB=1

SAME5X_C_DEFINES := $(SAME5X_DEFINES) -Dnoexcept=

# Compiler flags - C
SAME5X_CFLAGS := -c -std=gnu99 \
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
	-Werror=return-type \
	-Werror=implicit \
	-fsingle-precision-constant \
	-fstack-usage \
	-fdump-rtl-expand \
	-Wall \
	$(SAME5X_INCLUDES) \
	$(SAME5X_C_DEFINES)

# Compiler flags - C++
SAME5X_CXXFLAGS := -c -std=gnu++17 \
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
	-Werror=return-type \
	-Wsuggest-override \
	-fsingle-precision-constant \
	-fstack-usage \
	-fdump-rtl-expand \
	-Wall \
	$(SAME5X_INCLUDES) \
	$(SAME5X_DEFINES)

# Add debug flags if DEBUG=1
ifeq ($(DEBUG),1)
SAME5X_CFLAGS += -O0 -g3
SAME5X_CXXFLAGS += -O0 -g3
else
SAME5X_CFLAGS += -O3
SAME5X_CXXFLAGS += -O3
endif

# Object files
SAME5X_CPP_OBJS := $(SAME5X_CPP_SRCS:%.cpp=$(SAME5X_BUILD_DIR)/%.o)
SAME5X_C_OBJS := $(SAME5X_C_SRCS:%.c=$(SAME5X_BUILD_DIR)/%.o)
SAME5X_OBJS := $(SAME5X_CPP_OBJS) $(SAME5X_C_OBJS)

# Dependency files
SAME5X_DEPS := $(SAME5X_OBJS:.o=.d)

# Target rule
.PHONY: SAME5x_CAN_SDHC_USB_RTOS
SAME5x_CAN_SDHC_USB_RTOS: $(SAME5X_TARGET)

$(SAME5X_TARGET): $(SAME5X_OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

# Compile C++ files
$(SAME5X_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAME5X_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(SAME5X_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAME5X_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAME5X_DEPS)

# Clean target
.PHONY: clean-SAME5x_CAN_SDHC_USB_RTOS
clean-SAME5x_CAN_SDHC_USB_RTOS:
	$(Q)echo "  RM      $(SAME5X_BUILD_DIR)"
	$(Q)rm -rf $(SAME5X_BUILD_DIR)

