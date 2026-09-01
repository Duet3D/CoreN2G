# Makefile for CoreN2G configuration: RP2040_CAN_RTOS

RP2040_CAN_RTOS_BUILD_DIR := RP2040_CAN_RTOS
RP2040_CAN_RTOS_TARGET := $(RP2040_CAN_RTOS_BUILD_DIR)/libCoreN2G.a
RP2040_CAN_RTOS_SRC_DIR := src


# Compiler flags - C
RP2040_CAN_RTOS_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m0plus -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Wno-error=return-type \
	-fsingle-precision-constant \
	-O3

# Compiler flags - C++
RP2040_CAN_RTOS_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m0plus -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-O3

# Defines - C only
RP2040_CAN_RTOS_C_DEFS := -D__RP2040__ -Dnoexcept= -DSUPPORT_CAN=1 -DSUPPORT_USB=1 -DRTOS

# Defines - C++
RP2040_CAN_RTOS_CXX_DEFS := -D__RP2040__ -DSUPPORT_CAN=1 -DSUPPORT_USB=1 -DRTOS

# Include paths
RP2040_CAN_RTOS_INCLUDES := \
	-I$(RP2040_CAN_RTOS_SRC_DIR) \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/common/pico_base/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/common/pico_sync/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/common/pico_time/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/boards/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_adc/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_dma/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_base/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_gpio/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_irq/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_pio/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_pwm/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_sync/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_timer/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/hardware_watchdog/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/pico_multicore/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/pico_platform/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/pico_unique_id/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/cmsis/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Core/Include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2040/hardware_regs/include \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/src/rp2040/hardware_structs/include \
	-I../LibTinyusb/src/tinyusb/src \
	-I$(RP2040_CAN_RTOS_SRC_DIR)/RP2040 \
	-I../RRFLibraries/src \
	-I../CANlib/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM0

# Source files - C
RP2040_CAN_RTOS_CSRC := $(shell find $(RP2040_CAN_RTOS_SRC_DIR) \
	-type f -name "*.c" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/SAME5x_C21/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/SAM4S_4E_E70/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/SAME70/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/STM32/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/STMCubeMX/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/atmel/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/arm/*")

# Source files - C++
RP2040_CAN_RTOS_CPPSRC := $(shell find $(RP2040_CAN_RTOS_SRC_DIR) \
	-type f -name "*.cpp" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/RP2040/pico-sdk/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/SAME5x_C21/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/SAM4S_4E_E70/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/SAME70/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/STM32/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/STMCubeMX/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/atmel/*" \
	! -path "$(RP2040_CAN_RTOS_SRC_DIR)/arm/*")

# Object files
RP2040_CAN_RTOS_COBJ := $(patsubst $(RP2040_CAN_RTOS_SRC_DIR)/%.c,$(RP2040_CAN_RTOS_BUILD_DIR)/%.o,$(RP2040_CAN_RTOS_CSRC))
RP2040_CAN_RTOS_CXXOBJ := $(patsubst $(RP2040_CAN_RTOS_SRC_DIR)/%.cpp,$(RP2040_CAN_RTOS_BUILD_DIR)/%.o,$(RP2040_CAN_RTOS_CPPSRC))
RP2040_CAN_RTOS_OBJS := $(RP2040_CAN_RTOS_COBJ) $(RP2040_CAN_RTOS_CXXOBJ)
RP2040_CAN_RTOS_DEPS := $(RP2040_CAN_RTOS_OBJS:.o=.d)

# Main target
.PHONY: RP2040_CAN_RTOS
RP2040_CAN_RTOS: $(RP2040_CAN_RTOS_TARGET)

$(RP2040_CAN_RTOS_TARGET): $(RP2040_CAN_RTOS_OBJS)
	@mkdir -p $(dir $@)
	@echo "  AR      $@"
	$(Q)$(AR) rcs $@ $^

# C++ compilation
$(RP2040_CAN_RTOS_BUILD_DIR)/%.o: $(RP2040_CAN_RTOS_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX     $<"
	$(Q)$(CXX) $(RP2040_CAN_RTOS_CXXFLAGS) $(RP2040_CAN_RTOS_CXX_DEFS) $(RP2040_CAN_RTOS_INCLUDES) -MMD -MP -o $@ $<

# C compilation
$(RP2040_CAN_RTOS_BUILD_DIR)/%.o: $(RP2040_CAN_RTOS_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo "  CC      $<"
	$(Q)$(CC) $(RP2040_CAN_RTOS_CFLAGS) $(RP2040_CAN_RTOS_C_DEFS) $(RP2040_CAN_RTOS_INCLUDES) -MMD -MP -o $@ $<

# Clean
.PHONY: clean-RP2040_CAN_RTOS
clean-RP2040_CAN_RTOS:
	@echo "  Cleaning RP2040_CAN_RTOS..."
	$(Q)rm -rf $(RP2040_CAN_RTOS_BUILD_DIR)

# Include dependencies
-include $(RP2040_CAN_RTOS_DEPS)
