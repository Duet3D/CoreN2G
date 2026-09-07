# CoreN2G (MCU hardware abstraction layer) as a reusable CMake component.
# See lib/FreeRTOS/FreeRTOS.cmake for the include-a-module-and-call-a-function pattern.
#
#   coren2g_add_library(
#       TARGET <name>
#       MCU    <SAME70>            # currently the SAME70_CAN_SDHC_USB_RTOS variant
#       ARCH   <interface target>
#       [CAN] [USB] [SDHC] [RTOS]  # feature switches -> -DSUPPORT_*/-DRTOS
#   )
#
# CoreN2G's source tree carries the HALs for every Duet MCU in one place; a build selects one MCU
# by pruning the others, exactly as the old Makefiles' find-with-exclusions did.

set(COREN2G_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(COREN2G_LIBRARY_FLAGS
    "CAN"
    "USB"
    "SDHC"
    "RTOS"
)
set(COREN2G_LIBRARY_ARGS
    "CANLIB_INTERFACE"          # interface target for CANlib, if CAN is enabled
    "FREERTOS_INTERFACE"        # interface target for FreeRTOS, if RTOS is enabled
    "RRFLIBRARIES_INTERFACE"    # interface target for RRFLibraries, if RTOS is enabled
    "LIBTINYUSB_INTERFACE"      # interface target for LibTinyusb, if USB is enabled
)

include("${LIBRARIES_DIR}/LibraryUtils.cmake")

function(coren2g_add_interface OUT_TARGET)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "${COREN2G_LIBRARY_FLAGS}" "${DEFAULT_INTERFACE_ARGS}" "")
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "coren2g_add_interface: unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    get_enabled_features(_enabled_features ${COREN2G_LIBRARY_FLAGS})
    make_library_name(_target "CoreN2G" INTERFACE ${ARG_MCU} ${_enabled_features})
    set(${OUT_TARGET} "${_target}" PARENT_SCOPE)
    if(TARGET ${_target})
        return()  # already built for this MCU and feature set
    endif()

    set(_src "${COREN2G_DIR}/src")

    if(ARG_MCU STREQUAL "SAME70")
        set(_mcu_include_dirs
            "${_src}/SAM4S_4E_E70"
            "${_src}/SAM4S_4E_E70/asf"
            "${_src}/SAM4S_4E_E70/asf/sam/drivers"
            "${_src}/SAM4S_4E_E70/asf/sam/utils"
            "${_src}/SAM4S_4E_E70/asf/common/utils"
            "${_src}/SAM4S_4E_E70/asf/sam/utils/preprocessor"
            "${_src}/SAM4S_4E_E70/asf/sam/utils/header_files"
            "${_src}/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include"
            "${_src}/arm/CMSIS/5.4.0/CMSIS/Core/Include"
            "${_src}/SAM4S_4E_E70/SAME70"
        )
    else()
        message(FATAL_ERROR "coren2g_add_interface: unsupported MCU '${ARG_MCU}'")
    endif()

    add_library(${_target} INTERFACE)
    target_include_directories(${_target} INTERFACE
        "${COREN2G_DIR}"
        "${_src}"
        "${_mcu_include_dirs}"
    )
endfunction()

function(coren2g_add_library OUT_TARGET)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "${COREN2G_LIBRARY_FLAGS}" "${DEFAULT_LIBRARY_ARGS};${COREN2G_LIBRARY_ARGS}" "")
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "coren2g_add_library: unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    get_enabled_features(_enabled_features ${COREN2G_LIBRARY_FLAGS})
    make_library_name(_target "CoreN2G" STATIC ${ARG_MCU} ${_enabled_features})
    set(${OUT_TARGET} "${_target}" PARENT_SCOPE)
    if(TARGET ${_target})
        return()  # already built for this MCU and feature set
    endif()

    set(_src "${COREN2G_DIR}/src")
    set(_lib "${COREN2G_DIR}/..")

    if(ARG_MCU STREQUAL "SAME70")
        # Prune the HALs for other MCUs. C and C++ share this base set; C additionally drops the ASF
        # drivers this configuration does not use.
        set(_common_excludes
            "/RP2040/"
            "/SAM4S_4E_E70/SAM4S/"
            "/SAM4S_4E_E70/SAM4E/"
            "/SAME5x_C21/"
            "/atmel/"
            "/arm/"
        )
        set(_c_only_excludes
            "/asf/common/services/clock/sam4s/"
            "/asf/common/services/clock/sam4e/"
            "/asf/sam/drivers/adc/"
            "/asf/sam/drivers/cmcc/"
            "/asf/sam/drivers/crccu/"
            "/asf/sam/drivers/dmac/"
            "/asf/sam/drivers/pdc/"
            "/asf/sam/drivers/trng/"
            "/asf/sam/drivers/twi/"
            "/asf/sam/drivers/udp/"
            "/asf/sam/drivers/uotghs/"
            "/asf/sam/utils/cmsis/sam4s/"
            "/asf/sam/utils/cmsis/sam4e/"
        )

        set(_mcu_compile_options)
        set(_mcu_include_dirs
            "${_src}/arm/CMSIS/5.4.0/CMSIS/Core/Include"
            "${_src}"
            "${_src}/SAM4S_4E_E70"
            "${_src}/SAM4S_4E_E70/SAME70"
            "${_src}/SAM4S_4E_E70/asf"
            "${_src}/SAM4S_4E_E70/asf/sam/drivers"
            "${_src}/SAM4S_4E_E70/asf/sam/drivers/pio"
            "${_src}/SAM4S_4E_E70/asf/sam/drivers/pmc"
            "${_src}/SAM4S_4E_E70/asf/sam/drivers/xdmac"
            "${_src}/SAM4S_4E_E70/asf/sam/utils"
            "${_src}/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include"
            "${_src}/SAM4S_4E_E70/asf/sam/utils/header_files"
            "${_src}/SAM4S_4E_E70/asf/sam/utils/preprocessor"
            "${_src}/SAM4S_4E_E70/asf/common/utils"
            "${_src}/SAM4S_4E_E70/asf/common/services/clock"
            "${_src}/SAM4S_4E_E70/asf/common/services/ioport"
            "${_src}/SAM4S_4E_E70/asf/common/services/sleepmgr"
        )
        set(_mcu_freertos_port "portable/GCC/ARM_CM7/r0p1")
        set(_mcu_usb_include_dirs
            "${_src}/SAM4S_4E_E70/asf/common/services/usb"
            "${_src}/SAM4S_4E_E70/asf/common/services/usb/class/cdc"
            "${_src}/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device"
            "${_src}/SAM4S_4E_E70/asf/common/services/usb/udc"
        )
    else()
        message(FATAL_ERROR "coren2g_add_library: unsupported MCU '${ARG_MCU}'")
    endif()


    file(GLOB_RECURSE _cpp_srcs CONFIGURE_DEPENDS "${_src}/*.cpp")
    file(GLOB_RECURSE _c_srcs   CONFIGURE_DEPENDS "${_src}/*.c")
    foreach(_ex IN LISTS _common_excludes)
        list(FILTER _cpp_srcs EXCLUDE REGEX "${_ex}")
        list(FILTER _c_srcs   EXCLUDE REGEX "${_ex}")
    endforeach()
    foreach(_ex IN LISTS _c_only_excludes)
        list(FILTER _c_srcs EXCLUDE REGEX "${_ex}")
    endforeach()

    add_library(${_target} STATIC ${_cpp_srcs} ${_c_srcs})

    target_link_libraries(${_target} PUBLIC I_${_target}) # link own interface target
    target_link_libraries(${_target} PRIVATE
        ${ARG_RRFLIBRARIES_INTERFACE}
    )

    target_include_directories(${_target} PRIVATE
        "${_mcu_include_dirs}"
    )

    if(ARG_RTOS)
        target_link_libraries(${_target} PRIVATE ${ARG_FREERTOS_INTERFACE})
    endif()

    if(ARG_USB)
        target_include_directories(${_target} PRIVATE
            "${_mcu_usb_include_dirs}"
        #     "${_lib}/LibTinyusb/src/tinyusb/src"
        #     "${_lib}/LibTinyusb/src"
        )
        target_link_libraries(${_target} PRIVATE ${ARG_LIBTINYUSB_INTERFACE})
    endif()

    if(ARG_CAN)
        target_link_libraries(${_target} PRIVATE ${ARG_CANLIB_INTERFACE})
    endif()

    target_compile_definitions(${_target} PRIVATE
        $<$<BOOL:${ARG_RTOS}>:RTOS>
        SUPPORT_USB=$<BOOL:${ARG_USB}>
        SUPPORT_CAN=$<BOOL:${ARG_CAN}>
        SUPPORT_SDHC=$<BOOL:${ARG_SDHC}>
    )

    target_compile_options(${_target} PRIVATE
        -ffunction-sections
        -fdata-sections
        -nostdlib
        -Wall
        -Wundef
        -Wdouble-promotion
        -Werror=return-type
        -fsingle-precision-constant
        -fstack-usage
        -fdump-rtl-expand
        $<$<COMPILE_LANGUAGE:C>:-Werror=implicit;-Dnoexcept=>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics;-fno-rtti;-fno-exceptions;-Wsuggest-override;-Werror;-Wnoexcept;-Wshadow;-Wsign-promo>
        $<$<NOT:$<CONFIG:Debug>>:-O3>
        $<$<CONFIG:Debug>:-Og;-g3>
        ${_mcu_compile_options}
    )

    target_link_libraries(${_target} PRIVATE ${ARG_ARCH})
endfunction()
