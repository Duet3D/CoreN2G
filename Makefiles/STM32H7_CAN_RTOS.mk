# CoreN2G STM32H7_CAN_RTOS Configuration Makefile

STM32H7_CAN_RTOS_BUILD_DIR := STM32H7_CAN_RTOS
STM32H7_CAN_RTOS_TARGET := $(STM32H7_CAN_RTOS_BUILD_DIR)/libCoreN2G.a
STM32H7_CAN_RTOS_SRC_DIR := src


# Compiler flags - C
STM32H7_CAN_RTOS_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Werror=implicit \
	-fsingle-precision-constant \
	-O3

# Compiler flags - C++
STM32H7_CAN_RTOS_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-fstack-usage \
	-O3

# Defines - C only
STM32H7_CAN_RTOS_C_DEFS := -DSTM32H743xx -Dnoexcept= -DSUPPORT_CAN=1 -DSUPPORT_SDHC=0 -DSUPPORT_USB=0 -DRTOS

# Defines - C++
STM32H7_CAN_RTOS_CXX_DEFS := -DSTM32H743xx -DSUPPORT_CAN=1 -DSUPPORT_SDHC=0 -DSUPPORT_USB=0 -DRTOS

# Include paths
STM32H7_CAN_RTOS_INCLUDES := \
	-I$(STM32H7_CAN_RTOS_SRC_DIR) \
	-I$(STM32H7_CAN_RTOS_SRC_DIR)/STMCubeMX/Drivers/STM32H7xx_HAL_Driver/Inc \
	-I$(STM32H7_CAN_RTOS_SRC_DIR)/STMCubeMX/Core/Inc \
	-I$(STM32H7_CAN_RTOS_SRC_DIR)/STMCubeMX/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
	-I$(STM32H7_CAN_RTOS_SRC_DIR)/STMCubeMX/Drivers/CMSIS/Include \
	-I$(STM32H7_CAN_RTOS_SRC_DIR)/STM32 \
	-I../RRFLibraries/src \
	-I../CANlib/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM7/r0p1

# Source files
STM32H7_CAN_RTOS_CSRC := $(shell find $(STM32H7_CAN_RTOS_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*STM32H5xx*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

STM32H7_CAN_RTOS_CPPSRC := $(shell find $(STM32H7_CAN_RTOS_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/*' \
	! -path '*STM32H5xx*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Object files
STM32H7_CAN_RTOS_COBJ := $(patsubst $(STM32H7_CAN_RTOS_SRC_DIR)/%.c,$(STM32H7_CAN_RTOS_BUILD_DIR)/%.o,$(STM32H7_CAN_RTOS_CSRC))
STM32H7_CAN_RTOS_CXXOBJ := $(patsubst $(STM32H7_CAN_RTOS_SRC_DIR)/%.cpp,$(STM32H7_CAN_RTOS_BUILD_DIR)/%.o,$(STM32H7_CAN_RTOS_CPPSRC))
STM32H7_CAN_RTOS_OBJS := $(STM32H7_CAN_RTOS_COBJ) $(STM32H7_CAN_RTOS_CXXOBJ)

# Dependency files
STM32H7_CAN_RTOS_DEPS := $(STM32H7_CAN_RTOS_OBJS:.o=.d)

# Build target
$(STM32H7_CAN_RTOS_TARGET): $(STM32H7_CAN_RTOS_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

# Compilation rules
$(STM32H7_CAN_RTOS_BUILD_DIR)/%.o: $(STM32H7_CAN_RTOS_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(STM32H7_CAN_RTOS_CFLAGS) $(STM32H7_CAN_RTOS_C_DEFS) $(STM32H7_CAN_RTOS_INCLUDES) -MMD -MP -o $@ $<

$(STM32H7_CAN_RTOS_BUILD_DIR)/%.o: $(STM32H7_CAN_RTOS_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(STM32H7_CAN_RTOS_CXXFLAGS) $(STM32H7_CAN_RTOS_CXX_DEFS) $(STM32H7_CAN_RTOS_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(STM32H7_CAN_RTOS_DEPS)

# Configuration target
STM32H7_CAN_RTOS: $(STM32H7_CAN_RTOS_TARGET)

.PHONY: STM32H7_CAN_RTOS
