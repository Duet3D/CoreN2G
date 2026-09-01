# CoreN2G Master Makefile
# Builds CoreN2G library for various MCU configurations

# Cross-compiler toolchain (relative to project root)
#CROSS_COMPILE ?= ../arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
CROSS_COMPILE ?= ../arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
export CROSS_COMPILE

# Toolchain commands
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AS = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

# Quiet build support (Linux kernel style)
# Use V=1 for verbose output
ifeq ($(V),1)
	Q :=
else
	Q := @
endif
export Q

# Available build configurations
CONFIGS := \
	SAMC21_CAN \
	SAMC21_CAN_RTOS \
	SAM4E_SDHC \
	SAM4E_SDHC_USB_RTOS \
	SAME5x_CAN \
	SAME5x_CAN_RTOS \
	SAME5x_CAN_SDHC_USB_RTOS \
	SAME5x_SDHC \
	SAME5x_SDHC_USB \
	SAME5x_SDHC_USB_RTOS \
	SAME70_CAN \
	SAME70_CAN_SDHC_USB_RTOS \
	SAME70_SDHC \
	SAME70_SDHC_USB \
	RP2040_CAN_RTOS \
	RP2040_RTOS \
	STM32H5_CAN_RTOS \
	STM32H7_CAN_RTOS

# Default target
.DEFAULT_GOAL := SAM4E_SDHC_USB_RTOS

# Print available targets
.PHONY: help
help:
	@echo "CoreN2G Library Build System"
	@echo "Available targets:"
	@for config in $(CONFIGS); do echo "  make $$config"; done
	@echo ""
	@echo "Other targets:"
	@echo "  make all          - Build all configurations"
	@echo "  make clean        - Clean all build outputs"
	@echo ""
	@echo "Options:"
	@echo "  V=1               - Verbose build output"
	@echo "  CROSS_COMPILE=$(CROSS_COMPILE)"
	@echo ""
	@echo "Other targets:"
	@echo "  make all          - Build all configurations"
	@echo "  make clean        - Clean all build outputs"
	@echo "  make clean-<config> - Clean specific configuration"
	@echo ""
	@echo "Environment variables:"
	@echo "  CROSS_COMPILE=$(CROSS_COMPILE)"

# Build all configurations
# RP2040 configs are excluded from 'all' (they require the pico-sdk build); still buildable as explicit targets
# STM32H7_CAN_RTOS is excluded too: src/STMCubeMX/Core holds the H523 CubeMX project, there is no H7 one yet
.PHONY: all
all: $(filter-out RP2040_CAN_RTOS RP2040_RTOS STM32H7_CAN_RTOS,$(CONFIGS))

# Include configuration-specific makefiles
-include Makefiles/SAMC21_CAN.mk
-include Makefiles/SAMC21_CAN_RTOS.mk
-include Makefiles/SAM4E_SDHC.mk
-include Makefiles/SAM4E_SDHC_USB_RTOS.mk
-include Makefiles/SAME5x_CAN.mk
-include Makefiles/SAME5x_CAN_RTOS.mk
-include Makefiles/SAME5x_CAN_SDHC_USB_RTOS.mk
-include Makefiles/SAME5x_SDHC.mk
-include Makefiles/SAME5x_SDHC_USB.mk
-include Makefiles/SAME5x_SDHC_USB_RTOS.mk
-include Makefiles/SAME70_CAN.mk
-include Makefiles/SAME70_CAN_SDHC_USB_RTOS.mk
-include Makefiles/SAME70_SDHC.mk
-include Makefiles/SAME70_SDHC_USB.mk
-include Makefiles/RP2040_CAN_RTOS.mk
-include Makefiles/RP2040_RTOS.mk
-include Makefiles/STM32H5_CAN_RTOS.mk
-include Makefiles/STM32H7_CAN_RTOS.mk

# Generic clean target
.PHONY: clean
clean:
	@echo "Cleaning all CoreN2G build outputs..."
	@for config in $(CONFIGS); do \
		if [ -d "$$config" ]; then \
			echo "  Cleaning $$config..."; \
			rm -rf "$$config"; \
		fi; \
	done

# Configuration-specific clean targets are defined in each config makefile
