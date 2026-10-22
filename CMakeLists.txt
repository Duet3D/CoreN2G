# CoreN2G (the MCU hardware abstraction layer) for the Duet3D firmware, built for DUET_MCU, which
# the consuming project's toolchain file sets along with the core's code generation flags.
#
#   DUET_MCU          SAME70 or SAME5x
#   DUET_PART_DEFINE  the macro naming the MCU part, e.g. __SAME70Q20B__
#   DUET_RTOS         build against Duet::FreeRTOS, which must already exist
#   COREN2G_CAN   CAN support, using Duet::CANlib
#   COREN2G_USB   USB support, using Duet::LibTinyusb
#   COREN2G_SDHC  SD card (HSMCI/SDHC) support
#
# Target: Duet::CoreN2G. Needs Duet::RRFLibraries.
#
# The source tree carries the HALs for every Duet MCU; a build compiles only those for DUET_MCU.

cmake_minimum_required(VERSION 3.25)
project(CoreN2G LANGUAGES C CXX)

option(COREN2G_CAN  "Build CoreN2G with CAN support"  OFF)
option(COREN2G_USB  "Build CoreN2G with USB support"  OFF)
option(COREN2G_SDHC "Build CoreN2G with SD card support" OFF)

# _excludes: source directories (relative to src/) for other MCUs; _c_excludes: C sources this MCU
# does not use
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
        SAM4S_4E_E70/asf/sam/drivers/trng
        SAM4S_4E_E70/asf/sam/drivers/twi
        SAM4S_4E_E70/asf/sam/drivers/udp
        SAM4S_4E_E70/asf/sam/drivers/uotghs
        SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s
        SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e)
    # Include directories for consumers, then those only CoreN2G's own sources use
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
    set(_excludes RP2040 SAM4S_4E_E70 SAME5x_C21/SAMC21 STM32 STMCubeMX atmel arm)
    set(_c_excludes)
    set(_public_includes
        src
        src/SAME5x_C21
        src/SAME5x_C21/SAME5x
        src/SAME5x_C21/SAME5x/hal/include
        src/SAME5x_C21/SAME5x/hal/utils/include
        src/SAME5x_C21/SAME5x/hri
        src/SAME5x_C21/SAME5x/Config
        src/atmel/SAME54_DFP/1.1.134/include
        src/arm/CMSIS/5.4.0/CMSIS/Core/Include)
    set(_private_includes
        src/SAME5x_C21/SAME5x/pukcc)
    set(_usb_includes
        src/SAME5x_C21/SAME5x/usb
        src/SAME5x_C21/SAME5x/usb/device
        src/SAME5x_C21/SAME5x/usb/class/cdc)
else()
    message(FATAL_ERROR "CoreN2G: unsupported DUET_MCU '${DUET_MCU}'")
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

add_library(CoreN2G STATIC ${_cxx_srcs} ${_c_srcs})
add_library(Duet::CoreN2G ALIAS CoreN2G)

set_target_properties(CoreN2G PROPERTIES
    C_STANDARD 99
    C_EXTENSIONS ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if(NOT COREN2G_USB)
    set(_usb_includes)
endif()
target_include_directories(CoreN2G
    PUBLIC ${_public_includes}
    PRIVATE ${_private_includes} ${_usb_includes})

target_compile_definitions(CoreN2G PRIVATE
    ${DUET_PART_DEFINE}
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
if(COREN2G_USB)
    target_link_libraries(CoreN2G PRIVATE Duet::LibTinyusb)
endif()

target_compile_options(CoreN2G PRIVATE
    $<$<CONFIG:Release>:-O3>
    -fno-math-errno
    -fsingle-precision-constant
    -Wall
    -Wundef
    -Wdouble-promotion
    -Werror=return-type
    $<$<COMPILE_LANGUAGE:C>:-Werror=implicit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics;-fno-rtti;-fno-exceptions;-Wsuggest-override;-Werror;-Wnoexcept;-Wshadow;-Wsign-promo>
    -fstack-usage
    -fdump-rtl-expand
    # __FILE__ relative to the library root, so the image does not depend on the checkout location
    -fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=)
