# CoreN2G SAME70_CAN_SDHC_USB_RTOS Configuration Makefile
# Build configuration for SAME70 with CAN, SDHC and USB support (RTOS)

SAME70_BUILD_DIR := SAME70_CAN_SDHC_USB_RTOS
SAME70_TARGET := $(SAME70_BUILD_DIR)/libCoreN2G.a

# Source directories
SAME70_SRC_DIR := src

# Find all source files (Eclipse excludes similar paths as SAM4E but with SAME70 included, SAM4E excluded)
SAME70_EXCLUDE_DIRS := \
	$(SAME70_SRC_DIR)/RP2040 \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/SAM4S \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/SAM4E \
	$(SAME70_SRC_DIR)/SAME5x_C21 \
	$(SAME70_SRC_DIR)/STM32 \
	$(SAME70_SRC_DIR)/STMCubeMX \
	$(SAME70_SRC_DIR)/atmel \
	$(SAME70_SRC_DIR)/arm

SAME70_C_EXCLUDE_DIRS := $(SAME70_EXCLUDE_DIRS) \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock/sam4s \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock/sam4e \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/adc \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/cmcc \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/crccu \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/dmac \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pdc \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/trng \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/twi \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/udp \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/uotghs \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s \
	$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e

SAME70_CPP_SRCS := $(filter-out $(addsuffix /%,$(SAME70_EXCLUDE_DIRS)),$(call rwildcard,$(SAME70_SRC_DIR),*.cpp))

SAME70_C_SRCS := $(filter-out $(addsuffix /%,$(SAME70_C_EXCLUDE_DIRS)),$(call rwildcard,$(SAME70_SRC_DIR),*.c))

# Assembly source files
SAME70_ASM_SRCS := $(filter-out $(addsuffix /%,$(SAME70_EXCLUDE_DIRS)),$(call rwildcard,$(SAME70_SRC_DIR),*.s))

# Include paths (matching Eclipse .cproject for SAME70_CAN_SDHC_USB_RTOS_Debug)
SAME70_INCLUDES := \
	-I$(SAME70_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAME70_SRC_DIR) \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70 \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/SAME70 \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pio \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pmc \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/xdmac \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/utils \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/ioport \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device \
	-I$(SAME70_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/udc \
	-I../RRFLibraries/src \
	-I../CANlib/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM7/r0p1 \
	-I../LibTinyusb/src/tinyusb/src \
	-I../LibTinyusb/src

# Preprocessor defines
SAME70_DEFINES := \
	-D__SAME70Q20B__ \
	-DRTOS \
	-DSUPPORT_CAN=1 \
	-DSUPPORT_USB=1 \
	-DSUPPORT_SDHC=1

# Compiler flags - C
SAME70_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m7 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-mno-unaligned-access \
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
	-O3 \
	-Wall \
	$(SAME70_INCLUDES) \
	$(SAME70_DEFINES) \
	-Dnoexcept= \
	$(DEBUG_FLAGS)

# Compiler flags - C++
SAME70_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m7 \
	-mthumb \
	-fno-math-errno \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-mno-unaligned-access \
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
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-fstack-usage \
	-fdump-rtl-expand \
	-O3 \
	-Wall \
	$(SAME70_INCLUDES) \
	$(SAME70_DEFINES) \
	$(DEBUG_FLAGS)

# Assembler flags
SAME70_ASFLAGS := -c \
	-mcpu=cortex-m7 \
	-mthumb \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	$(SAME70_INCLUDES) \
	$(SAME70_DEFINES)

# Object files
SAME70_CPP_OBJS := $(SAME70_CPP_SRCS:%.cpp=$(SAME70_BUILD_DIR)/%.o)
SAME70_C_OBJS := $(SAME70_C_SRCS:%.c=$(SAME70_BUILD_DIR)/%.o)
SAME70_ASM_OBJS := $(SAME70_ASM_SRCS:%.s=$(SAME70_BUILD_DIR)/%.o)
SAME70_OBJS := $(SAME70_CPP_OBJS) $(SAME70_C_OBJS) $(SAME70_ASM_OBJS)

# Dependency files
SAME70_DEPS := $(SAME70_OBJS:.o=.d)

# Target rule
.PHONY: SAME70_CAN_SDHC_USB_RTOS
SAME70_CAN_SDHC_USB_RTOS: $(SAME70_TARGET)

# Archive library
$(SAME70_TARGET): $(SAME70_OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

# Compile C++ files
$(SAME70_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAME70_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(SAME70_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAME70_CFLAGS) -MMD -MP -o $@ $<

# Compile assembly files
$(SAME70_BUILD_DIR)/%.o: %.s
	$(Q)echo "  AS      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(AS) $(SAME70_ASFLAGS) -o $@ $<

# Include dependencies
-include $(SAME70_DEPS)

# Clean target
.PHONY: clean-SAME70_CAN_SDHC_USB_RTOS
clean-SAME70_CAN_SDHC_USB_RTOS:
	$(Q)echo "  RM      $(SAME70_BUILD_DIR)"
	$(Q)rm -rf $(SAME70_BUILD_DIR)
