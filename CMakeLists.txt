# CoreN2G (the MCU hardware abstraction layer) for the Duet3D firmware, built for DUET_MCU, which
# the consuming project's toolchain file sets along with the core's code generation flags.
#
#   DUET_MCU              SAME70, SAME5x, SAMC21 or STM32H5
#   DUET_PART_DEFINE      the macro naming the MCU part, e.g. __SAME70Q20B__
#   DUET_RTOS             build against Duet::FreeRTOS, which must already exist
#   COREN2G_CAN           CAN support, using Duet::CANlib
#   COREN2G_USB           USB support; with DUET_RTOS it uses Duet::LibTinyusb
#   COREN2G_SDHC          SD card (HSMCI/SDHC) support
#   COREN2G_OPTIMISATION  Release optimisation for this library (default -O3)
#
# Target: Duet::CoreN2G. Needs Duet::RRFLibraries.
#
# The source tree carries the HALs for every Duet MCU; a build compiles only those for DUET_MCU.
#
# The SAME5x HAL is always built for the SAME54P20A with the SAME54 device pack, even when the
# board is a SAME51: one build serves both parts. Consumers are still compiled for their own part,
# so they get its device pack through the INTERFACE include directories. The HAL's sources are
# written against the SAME54's headers and do not compile for a SAME51 part.

cmake_minimum_required(VERSION 3.25)
project(CoreN2G LANGUAGES C CXX)

option(COREN2G_CAN  "Build CoreN2G with CAN support"  OFF)
option(COREN2G_USB  "Build CoreN2G with USB support"  OFF)
option(COREN2G_SDHC "Build CoreN2G with SD card support" OFF)
if(NOT DEFINED COREN2G_OPTIMISATION)
    set(COREN2G_OPTIMISATION -O3)
endif()

# Per MCU:
#   _excludes          source directories under src/ for other MCUs
#   _c_excludes        C source directories this MCU does not use
#   _usb_dirs          source directories of the USB stack, compiled only with COREN2G_USB
#   _public_includes   include directories for CoreN2G and its consumers
#   _private_includes  include directories for CoreN2G's own sources
#   _usb_includes      private include directories of the USB stack
#   _consumer_includes include directories for consumers only
set(_part ${DUET_PART_DEFINE})
set(_c_excludes)
set(_usb_dirs)
set(_usb_includes)
set(_consumer_includes)
set(_c_options -Werror=return-type -Werror=implicit)
if(DUET_MCU STREQUAL "SAME70")
    set(_excludes
        RP2040 SAM4S_4E_E70/SAM4S SAM4S_4E_E70/SAM4E SAME5x_C21 STM32 STMCubeMX atmel arm)
    set(_c_excludes
        SAM4S_4E_E70/asf/common/services/clock/sam4s
        SAM4S_4E_E70/asf/common/services/clock/sam4e
        SAM4S_4E_E70/asf/sam/drivers/adc
        SAM4S_4E_E70/asf/sam/drivers/cmcc
        SAM4S_4E_E70/asf/sam/drivers/crccu
        SAM4S_4E_E70/asf/sam/drivers/dmac
        SAM4S_4E_E70/asf/sam/drivers/pdc
        SAM4S_4E_E70/asf/sam/drivers/twi
        SAM4S_4E_E70/asf/sam/drivers/udp
        SAM4S_4E_E70/asf/sam/drivers/uotghs
        SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s
        SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e)
    if(COREN2G_USB)
        # The USB stack takes the TRNG's place
        list(APPEND _c_excludes SAM4S_4E_E70/asf/sam/drivers/trng)
    endif()
    set(_usb_dirs
        SAM4S_4E_E70/asf/common/services/usb
        SAM4S_4E_E70/asf/sam/drivers/usbhs)
    set(_public_includes
        src
        src/SAM4S_4E_E70
        src/SAM4S_4E_E70/asf
        src/SAM4S_4E_E70/asf/sam/drivers
        src/SAM4S_4E_E70/asf/sam/utils
        src/SAM4S_4E_E70/asf/common/utils
        src/SAM4S_4E_E70/asf/sam/utils/preprocessor
        src/SAM4S_4E_E70/asf/sam/utils/header_files
        src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include
        src/arm/CMSIS/5.4.0/CMSIS/Core/Include
        src/SAM4S_4E_E70/SAME70)
    set(_private_includes
        src/SAM4S_4E_E70/asf/sam/drivers/pio
        src/SAM4S_4E_E70/asf/sam/drivers/pmc
        src/SAM4S_4E_E70/asf/sam/drivers/xdmac
        src/SAM4S_4E_E70/asf/common/services/clock
        src/SAM4S_4E_E70/asf/common/services/ioport
        src/SAM4S_4E_E70/asf/common/services/sleepmgr)
    set(_usb_includes
        src/SAM4S_4E_E70/asf/common/services/usb
        src/SAM4S_4E_E70/asf/common/services/usb/class/cdc
        src/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device
        src/SAM4S_4E_E70/asf/common/services/usb/udc)
elseif(DUET_MCU STREQUAL "SAME5x")
    set(_part __SAME54P20A__)    # whatever the board's part; see the top of this file
    set(_excludes RP2040 SAM4S_4E_E70 SAME5x_C21/SAMC21 STM32 STMCubeMX atmel arm)
    set(_usb_dirs SAME5x_C21/SAME5x/usb)
    set(_public_includes
        src
        src/SAME5x_C21
        src/SAME5x_C21/SAME5x
        src/SAME5x_C21/SAME5x/hal/include
        src/SAME5x_C21/SAME5x/hal/utils/include
        src/SAME5x_C21/SAME5x/hri
        src/SAME5x_C21/SAME5x/Config
        src/arm/CMSIS/5.4.0/CMSIS/Core/Include)
    set(_private_includes
        src/atmel/SAME54_DFP/1.1.134/include
        src/SAME5x_C21/SAME5x/pukcc
        src/SAME5x_C21/SAME5x/usb
        src/SAME5x_C21/SAME5x/usb/device
        src/SAME5x_C21/SAME5x/usb/class/cdc)
    if(DUET_PART_DEFINE MATCHES "^__SAME51")
        set(_consumer_includes src/atmel/SAME51_DFP/1.1.139/include)
    else()
        set(_consumer_includes src/atmel/SAME54_DFP/1.1.134/include)
    endif()
elseif(DUET_MCU STREQUAL "SAMC21")
    set(_excludes RP2040 SAM4S_4E_E70 SAME5x_C21/SAME5x STM32 STMCubeMX atmel arm)
    set(_public_includes
        src
        src/SAME5x_C21/SAMC21/hal/include
        src/SAME5x_C21/SAMC21/hal/utils/include
        src/SAME5x_C21/SAMC21/hri
        src/atmel/SAMC21_DFP/1.2.176/samc21/include
        src/arm/CMSIS/5.4.0/CMSIS/Core/Include)
    set(_private_includes
        src/SAME5x_C21/SAMC21
        src/SAME5x_C21/SAMC21/Config)
    set(_consumer_includes src/SAME5x_C21)
    # The SAMC21 HAL's C sources have functions that fall off the end
    set(_c_options -Wno-error=return-type)
elseif(DUET_MCU STREQUAL "STM32H5")
    set(_excludes
        RP2040 SAM4S_4E_E70 SAME5x_C21 STMCubeMX/Core/Src STMCubeMX/Drivers/STM32H7xx_HAL_Driver
        STMCubeMX/Drivers/CMSIS/Device/ST/STM32H7xx atmel arm)
    set(_public_includes
        src
        src/STMCubeMX/Drivers/STM32H5xx_HAL_Driver/Inc
        src/STMCubeMX/Core/Inc
        src/STMCubeMX/Drivers/CMSIS/Device/ST/STM32H5xx/Include
        src/STMCubeMX/Drivers/CMSIS/Include
        src/STM32)
    set(_private_includes)
    set(_consumer_includes src/STM32/STM32H5)
else()
    message(FATAL_ERROR "CoreN2G: unsupported DUET_MCU '${DUET_MCU}'")
endif()
if(COREN2G_USB)
    list(APPEND _private_includes ${_usb_includes})
else()
    list(APPEND _excludes ${_usb_dirs})
endif()

file(GLOB_RECURSE _cxx_srcs CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE _c_srcs   CONFIGURE_DEPENDS src/*.c)
foreach(_dir IN LISTS _excludes)
    list(FILTER _cxx_srcs EXCLUDE REGEX "/src/${_dir}/")
    list(FILTER _c_srcs   EXCLUDE REGEX "/src/${_dir}/")
endforeach()
foreach(_dir IN LISTS _c_excludes)
    list(FILTER _c_srcs EXCLUDE REGEX "/src/${_dir}/")
endforeach()

# The order of the archive members decides which the linker loads first, and so the layout of the
# image: USB builds list the C++ sources first and the others the C sources
if(COREN2G_USB)
    add_library(CoreN2G STATIC ${_cxx_srcs} ${_c_srcs})
else()
    add_library(CoreN2G STATIC ${_c_srcs} ${_cxx_srcs})
endif()
add_library(Duet::CoreN2G ALIAS CoreN2G)

set_target_properties(CoreN2G PROPERTIES
    C_STANDARD 99
    C_EXTENSIONS ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

target_include_directories(CoreN2G
    PUBLIC ${_public_includes}
    PRIVATE ${_private_includes}
    INTERFACE ${_consumer_includes})

target_compile_definitions(CoreN2G PRIVATE
    ${_part}
    SUPPORT_CAN=$<BOOL:${COREN2G_CAN}>
    SUPPORT_USB=$<BOOL:${COREN2G_USB}>
    SUPPORT_SDHC=$<BOOL:${COREN2G_SDHC}>
    $<$<COMPILE_LANGUAGE:C>:noexcept=>)

target_link_libraries(CoreN2G PUBLIC Duet::RRFLibraries)
if(DUET_RTOS)
    target_link_libraries(CoreN2G PUBLIC Duet::FreeRTOS)
endif()
if(COREN2G_CAN)
    target_link_libraries(CoreN2G PUBLIC Duet::CANlib)
endif()
if(COREN2G_USB AND DUET_RTOS)
    target_link_libraries(CoreN2G PRIVATE Duet::LibTinyusb)
endif()

set(_cxx_options
    -Werror=return-type
    -fno-threadsafe-statics
    -fno-rtti
    -fno-exceptions
    -Wsuggest-override
    -Werror
    -Wnoexcept
    -Wshadow
    -Wsign-promo)

target_compile_options(CoreN2G PRIVATE
    $<$<CONFIG:Release>:${COREN2G_OPTIMISATION}>
    -fno-math-errno
    -fsingle-precision-constant
    -Wall
    -Wundef
    -Wdouble-promotion
    "$<$<COMPILE_LANGUAGE:C>:${_c_options}>"
    "$<$<COMPILE_LANGUAGE:CXX>:${_cxx_options}>"
    -fstack-usage
    -fdump-rtl-expand
    # __FILE__ relative to the library root, so the image does not depend on the checkout location
    -fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=)
