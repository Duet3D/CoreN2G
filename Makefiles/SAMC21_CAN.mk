# Makefile for CoreN2G configuration: SAMC21_CAN

SAMC21_CAN_BUILD_DIR := SAMC21_CAN
SAMC21_CAN_TARGET := $(SAMC21_CAN_BUILD_DIR)/libCoreN2G.a
SAMC21_CAN_SRC_DIR := src


# Compiler flags - C
SAMC21_CAN_CFLAGS := -c -std=gnu99 \
	-mcpu=cortex-m0plus -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Wno-error=return-type \
	-fsingle-precision-constant \
	-Os \
	$(DEBUG_FLAGS)

# Compiler flags - C++
SAMC21_CAN_CXXFLAGS := -c -std=c++20 \
	-mcpu=cortex-m0plus -mthumb \
	-fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib \
	-Wall -Wundef -Wdouble-promotion -Werror=return-type -Wsuggest-override \
	-Werror -Wnoexcept -Wshadow -Wsign-promo \
	-fsingle-precision-constant \
	-Os \
	$(DEBUG_FLAGS)

# Defines - C only
SAMC21_CAN_C_DEFS := -D__SAMC21G18A__ -Dnoexcept= -DSUPPORT_CAN=1

# Defines - C++
SAMC21_CAN_CXX_DEFS := -D__SAMC21G18A__ -DSUPPORT_CAN=1

# Include paths
SAMC21_CAN_INCLUDES := \
	-I$(SAMC21_CAN_SRC_DIR)/SAME5x_C21/SAMC21 \
	-I$(SAMC21_CAN_SRC_DIR)/SAME5x_C21/SAMC21/hal/include \
	-I$(SAMC21_CAN_SRC_DIR)/SAME5x_C21/SAMC21/hal/utils/include \
	-I$(SAMC21_CAN_SRC_DIR)/SAME5x_C21/SAMC21/hri \
	-I$(SAMC21_CAN_SRC_DIR)/SAME5x_C21/SAMC21/Config \
	-I$(SAMC21_CAN_SRC_DIR)/atmel/SAMC21_DFP/1.2.176/samc21/include \
	-I$(SAMC21_CAN_SRC_DIR)/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAMC21_CAN_SRC_DIR) \
	-I../RRFLibraries/src \
	-I../CANlib/src

# Source files - exclude: RP2040, SAM4S_4E_E70, SAME5x_C21/SAME5x, atmel, arm
SAMC21_CAN_CSRC := $(shell find $(SAMC21_CAN_SRC_DIR) -name '*.c' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAME5x/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

SAMC21_CAN_CPPSRC := $(shell find $(SAMC21_CAN_SRC_DIR) -name '*.cpp' \
	! -path '*/RP2040/*' \
	! -path '*/SAM4S_4E_E70/*' \
	! -path '*/SAME5x_C21/SAME5x/*' \
	! -path '*/atmel/*' \
	! -path '*/arm/*')

# Object files
SAMC21_CAN_C_OBJS := $(SAMC21_CAN_CSRC:%.c=$(SAMC21_CAN_BUILD_DIR)/%.o)
SAMC21_CAN_CPP_OBJS := $(SAMC21_CAN_CPPSRC:%.cpp=$(SAMC21_CAN_BUILD_DIR)/%.o)
SAMC21_CAN_OBJS := $(SAMC21_CAN_C_OBJS) $(SAMC21_CAN_CPP_OBJS)

# Dependency files
SAMC21_CAN_DEPS := $(SAMC21_CAN_OBJS:.o=.d)

# Target rule
.PHONY: SAMC21_CAN clean-SAMC21_CAN

SAMC21_CAN: $(SAMC21_CAN_TARGET)

$(SAMC21_CAN_TARGET): $(SAMC21_CAN_OBJS)
	$(Q)echo "  AR      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(AR) rcs $@ $^

# Compile C++ files
$(SAMC21_CAN_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAMC21_CAN_CXXFLAGS) $(SAMC21_CAN_CXX_DEFS) $(SAMC21_CAN_INCLUDES) -MMD -MP -o $@ $<

# Compile C files
$(SAMC21_CAN_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAMC21_CAN_CFLAGS) $(SAMC21_CAN_C_DEFS) $(SAMC21_CAN_INCLUDES) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAMC21_CAN_DEPS)

# Clean target
clean-SAMC21_CAN:
	$(Q)echo "  RM      $(SAMC21_CAN_BUILD_DIR)"
	$(Q)rm -rf $(SAMC21_CAN_BUILD_DIR)
