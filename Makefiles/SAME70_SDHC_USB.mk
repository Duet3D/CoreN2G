# CoreN2G SAME70_SDHC_USB Configuration Makefile (USB without RTOS, for IAP)
# Build configuration for SAME70 with SDHC and USB support (no RTOS, for IAP)
SAME70SU_BUILD_DIR := SAME70_SDHC_USB
SAME70SU_TARGET := $(SAME70SU_BUILD_DIR)/libCoreN2G.a
# Source directories
SAME70SU_SRC_DIR := src
# Find all source files (Eclipse excludes similar paths as SAM4E but with SAME70 included, SAM4E excluded)
SAME70SU_CPP_SRCS := $(shell find $(SAME70SU_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAM4E/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')
SAME70SU_C_SRCS := $(shell find $(SAME70SU_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAM4E/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/common/services/clock/sam4e/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/adc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/cmcc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/crccu/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/dmac/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/pdc/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/trng/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/twi/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/udp/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/drivers/uotghs/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/*' \
	! -path '*/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')
# Assembly source files
SAME70SU_ASM_SRCS := $(shell find $(SAME70SU_SRC_DIR) -name '*.s' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/SAM4S/*' \
	! -path '*/SAM4S_4E_E70/SAM4E/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')
# Include paths (matching Eclipse .cproject for SAME70_SDHC_USB_Debug)
SAME70SU_INCLUDES := \
	-I$(SAME70SU_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAME70SU_SRC_DIR) \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70 \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/SAME70 \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pio \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/pmc \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/drivers/xdmac \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/utils \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/services/clock \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/services/ioport \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/services/sleepmgr \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device \
	-I$(SAME70SU_SRC_DIR)/SAM4S_4E_E70/asf/common/services/usb/udc \
	-I../RRFLibraries/src \
	 \
	 \
	 \
	 \
	
# Preprocessor defines (with IAP USB descriptor overrides)
SAME70SU_DEFINES := \
	-D__SAME70Q20B__ \
	-DSUPPORT_CAN=0 \
	-DSUPPORT_USB=1 \
	-DSUPPORT_SDHC=1 \
	-DUSB_DEVICE_VENDOR_ID=0x1D50 \
	-DUSB_DEVICE_PRODUCT_ID=0x60EF \
	'-DUSB_DEVICE_PRODUCT_NAME="IAP"' \
	-DUDI_CDC_RX_BUFFERS=2048 \
	-DUSB_DEVICE_HS_SUPPORT
# Compiler flags - C
SAME70SU_CFLAGS := -c -std=gnu99 \
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
	-Os \
	-Wall \
	$(SAME70SU_INCLUDES) \
	$(SAME70SU_DEFINES) \
	-Dnoexcept=
# Compiler flags - C++
SAME70SU_CXXFLAGS := -c -std=c++20 \
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
	-Os \
	-Wall \
	$(SAME70SU_INCLUDES) \
	$(SAME70SU_DEFINES)
# Assembler flags
SAME70SU_ASFLAGS := -c \
	-mcpu=cortex-m7 \
	-mthumb \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	$(SAME70SU_INCLUDES) \
	$(SAME70SU_DEFINES)
# Object files
SAME70SU_CPP_OBJS := $(SAME70SU_CPP_SRCS:%.cpp=$(SAME70SU_BUILD_DIR)/%.o)
SAME70SU_C_OBJS := $(SAME70SU_C_SRCS:%.c=$(SAME70SU_BUILD_DIR)/%.o)
SAME70SU_ASM_OBJS := $(SAME70SU_ASM_SRCS:%.s=$(SAME70SU_BUILD_DIR)/%.o)
SAME70SU_OBJS := $(SAME70SU_CPP_OBJS) $(SAME70SU_C_OBJS) $(SAME70SU_ASM_OBJS)
# Dependency files
SAME70SU_DEPS := $(SAME70SU_OBJS:.o=.d)
# Target rule
.PHONY: SAME70_SDHC_USB
SAME70_SDHC_USB: $(SAME70SU_TARGET)
# Archive library
$(SAME70SU_TARGET): $(SAME70SU_OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^
# Compile C++ files
$(SAME70SU_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAME70SU_CXXFLAGS) -MMD -MP -o $@ $<
# Compile C files
$(SAME70SU_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAME70SU_CFLAGS) -MMD -MP -o $@ $<
# Compile assembly files
$(SAME70SU_BUILD_DIR)/%.o: %.s
	$(Q)echo "  AS      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(AS) $(SAME70SU_ASFLAGS) -o $@ $<
# Include dependencies
-include $(SAME70SU_DEPS)
# Clean target
.PHONY: clean-SAME70_SDHC_USB
clean-SAME70_SDHC_USB:
	$(Q)echo "  RM      $(SAME70SU_BUILD_DIR)"
	$(Q)rm -rf $(SAME70SU_BUILD_DIR)
