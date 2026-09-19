# CoreN2G Master Makefile
# Builds CoreN2G library for various MCU configurations

# Cross-compiler toolchain.
# RepRapFirmware exports CROSS_COMPILE when building this as a submodule.
# When building CoreN2G standalone, fall back to a toolchain on PATH.
ARM_GNU_TOOLCHAIN_VERSION ?= 15.2.rel1
ifeq ($(OS),Windows_NT)
HOST_ARCH_RAW := $(subst AMD64,x86_64,$(subst ARM64,aarch64,$(PROCESSOR_ARCHITECTURE)))
else
HOST_ARCH_RAW := $(shell uname -m)
HOST_OS_RAW := $(shell uname -s)
endif

ifeq ($(HOST_ARCH_RAW),aarch64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),arm64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),x86_64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else ifeq ($(HOST_ARCH_RAW),amd64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else
ARM_GNU_TOOLCHAIN_HOST_ARCH := $(HOST_ARCH_RAW)
endif

ifeq ($(OS),Windows_NT)
ARM_GNU_TOOLCHAIN_HOST := mingw-w64-$(ARM_GNU_TOOLCHAIN_HOST_ARCH)
else ifeq ($(HOST_OS_RAW),Darwin)
ARM_GNU_TOOLCHAIN_HOST := darwin-$(subst aarch64,arm64,$(ARM_GNU_TOOLCHAIN_HOST_ARCH))
else
ARM_GNU_TOOLCHAIN_HOST := $(ARM_GNU_TOOLCHAIN_HOST_ARCH)
endif

CROSS_COMPILE ?= $(abspath ../arm-gnu-toolchain-$(ARM_GNU_TOOLCHAIN_VERSION)-$(ARM_GNU_TOOLCHAIN_HOST)-arm-none-eabi/bin/arm-none-eabi-)
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

# Recursive wildcard: $(call rwildcard,<dir>,<patterns>)
# Source lists must not shell out to find, which resolves to FIND.EXE on Windows
rwildcard = $(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

# An empty scan would archive no objects and then look up to date forever, so stop here instead
ifeq ($(wildcard src/*),)
$(error No sources found under src - is the checkout complete?)
endif

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
