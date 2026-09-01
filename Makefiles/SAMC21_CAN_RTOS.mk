# Makefile for CoreN2G configuration: SAMC21_CAN_RTOS

SAMC21_CAN_RTOS_BUILD_DIR := SAMC21_CAN_RTOS
SAMC21_CAN_RTOS_TARGET := $(SAMC21_CAN_RTOS_BUILD_DIR)/libCoreN2G.a
SAMC21_CAN_RTOS_SRC_DIR := src


# Compiler flags - C
SAMC21_CAN_RTOS_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m0plus -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Wno-error=return-type \
	-fsingle-precision-constant \
	-O3

# Compiler flags - C++
SAMC21_CAN_RTOS_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m0plus -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-O3

# Defines - C only
SAMC21_CAN_RTOS_C_DEFS := -D__SAMC21G18A__ -Dnoexcept= -DSUPPORT_CAN=1 -DRTOS

# Defines - C++
SAMC21_CAN_RTOS_CXX_DEFS := -D__SAMC21G18A__ -DSUPPORT_CAN=1 -DRTOS

# Include paths
SAMC21_CAN_RTOS_INCLUDES := \
	-I$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x_C21/SAMC21 \
	-I$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x_C21/SAMC21/hal/include \
	-I$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x_C21/SAMC21/hal/utils/include \
	-I$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x_C21/SAMC21/hri \
	-I$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x_C21/SAMC21/Config \
	-I$(SAMC21_CAN_RTOS_SRC_DIR)/atmel/SAMC21_DFP/1.2.176/samc21/include \
	-I$(SAMC21_CAN_RTOS_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAMC21_CAN_RTOS_SRC_DIR) \
	-I../RRFLibraries/src \
	-I../CANlib/src \
	-I../FreeRTOS/src/include \
	-I../FreeRTOS/src/portable/GCC/ARM_CM0

# Source files - C
SAMC21_CAN_RTOS_CSRC := $(shell find $(SAMC21_CAN_RTOS_SRC_DIR) \
	-type f -name "*.c" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/RP2040/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAM4S_4E_E70/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x_C21/SAME5x/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAME70/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/STM32/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/STMCubeMX/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/atmel/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/arm/*")

# Source files - C++
SAMC21_CAN_RTOS_CPPSRC := $(shell find $(SAMC21_CAN_RTOS_SRC_DIR) \
	-type f -name "*.cpp" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/RP2040/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAM4S_4E_E70/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x_C21/SAME5x/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAME5x/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/SAME70/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/STM32/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/STMCubeMX/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/atmel/*" \
	! -path "$(SAMC21_CAN_RTOS_SRC_DIR)/arm/*")

# Object files
SAMC21_CAN_RTOS_COBJ := $(patsubst $(SAMC21_CAN_RTOS_SRC_DIR)/%.c,$(SAMC21_CAN_RTOS_BUILD_DIR)/%.o,$(SAMC21_CAN_RTOS_CSRC))
SAMC21_CAN_RTOS_CXXOBJ := $(patsubst $(SAMC21_CAN_RTOS_SRC_DIR)/%.cpp,$(SAMC21_CAN_RTOS_BUILD_DIR)/%.o,$(SAMC21_CAN_RTOS_CPPSRC))
SAMC21_CAN_RTOS_OBJS := $(SAMC21_CAN_RTOS_COBJ) $(SAMC21_CAN_RTOS_CXXOBJ)
SAMC21_CAN_RTOS_DEPS := $(SAMC21_CAN_RTOS_OBJS:.o=.d)

# Main target
.PHONY: SAMC21_CAN_RTOS
SAMC21_CAN_RTOS: $(SAMC21_CAN_RTOS_TARGET)

$(SAMC21_CAN_RTOS_TARGET): $(SAMC21_CAN_RTOS_OBJS)
	@mkdir -p $(dir $@)
	@echo "  AR      $@"
	$(Q)$(AR) rcs $@ $^

# C++ compilation
$(SAMC21_CAN_RTOS_BUILD_DIR)/%.o: $(SAMC21_CAN_RTOS_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo "  CXX     $<"
	$(Q)$(CXX) $(SAMC21_CAN_RTOS_CXXFLAGS) $(SAMC21_CAN_RTOS_CXX_DEFS) $(SAMC21_CAN_RTOS_INCLUDES) -MMD -MP -o $@ $<

# C compilation
$(SAMC21_CAN_RTOS_BUILD_DIR)/%.o: $(SAMC21_CAN_RTOS_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo "  CC      $<"
	$(Q)$(CC) $(SAMC21_CAN_RTOS_CFLAGS) $(SAMC21_CAN_RTOS_C_DEFS) $(SAMC21_CAN_RTOS_INCLUDES) -MMD -MP -o $@ $<

# Clean
.PHONY: clean-SAMC21_CAN_RTOS
clean-SAMC21_CAN_RTOS:
	@echo "  Cleaning SAMC21_CAN_RTOS..."
	$(Q)rm -rf $(SAMC21_CAN_RTOS_BUILD_DIR)

# Include dependencies
-include $(SAMC21_CAN_RTOS_DEPS)
