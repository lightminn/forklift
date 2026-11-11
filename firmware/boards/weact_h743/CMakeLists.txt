# WeAct MiniSTM32H7xx (STM32H743VIT6) board build -- B1 skeleton (board design 2026-11-12).
# Included from firmware/CMakeLists.txt when FW_BOARD=weact_h743 with the Arm toolchain file.

set(FW_BUILD normal CACHE STRING "normal | commission | hil (board design 2.2)")
set_property(CACHE FW_BUILD PROPERTY STRINGS normal commission hil)
set(FW_MOTOR_OUTPUT 0 CACHE STRING "0 = outputs locked (default); 1 needs FW_IPROPI_PROTECTION_RECORD")
set(FW_IPROPI_PROTECTION_RECORD "" CACHE STRING
    "committed IPROPI protection record (motor output design 2); written by the user, never by an agent")
set(FW_LINK usb CACHE STRING "usb | uart7")
set(FW_ESTOP_PIN none CACHE STRING "none or a pin name; the schematic has no e-stop net yet")
set(FW_TEST_IWDG_STALL 0 CACHE STRING "0, or N = 1..60: S5 test build that stops the tick N s after boot")

if(NOT FW_LINK MATCHES "^(usb|uart7)$")
    message(FATAL_ERROR "FW_LINK must be usb or uart7")
endif()
if(NOT FW_BUILD MATCHES "^(normal|commission|hil)$")
    message(FATAL_ERROR "FW_BUILD must be normal, commission or hil")
endif()
if(NOT FW_TEST_IWDG_STALL MATCHES "^([0-9]|[1-5][0-9]|60)$")
    message(FATAL_ERROR "FW_TEST_IWDG_STALL must be 0 or a number of seconds 1..60")
endif()
if(FW_TEST_IWDG_STALL AND FW_MOTOR_OUTPUT)
    # Lead condition 2026-11-12: the stall test build always reports outputs_locked, so the real
    # executor refuses teleop and autonomy on it (board design 9).
    message(FATAL_ERROR "FW_TEST_IWDG_STALL needs the outputs locked (FW_MOTOR_OUTPUT=0)")
endif()
if(NOT FW_MOTOR_OUTPUT MATCHES "^[01]$")
    message(FATAL_ERROR "FW_MOTOR_OUTPUT must be 0 or 1")
endif()
set(FW_OPEN_MSG "Opening the motor outputs on the vehicle is the user's decision after the IPROPI "
    "protection is confirmed (lead decision Q6); being able to build this code is not that decision.")
string(CONCAT FW_OPEN_MSG ${FW_OPEN_MSG})
if(FW_MOTOR_OUTPUT AND NOT FW_BUILD STREQUAL "commission")
    # Motor output design 2: only the commissioning build may open the outputs now -- normal waits
    # for the e-stop pin (Q3), hil and the IWDG stall test build stay locked.
    message(FATAL_ERROR "FW_MOTOR_OUTPUT=1 is only for FW_BUILD=commission. ${FW_OPEN_MSG}")
endif()

set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(TP ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party)

# The IPROPI protection record gate (motor output design 2): checked now and before every build.
set(RECORD_H ${CMAKE_CURRENT_BINARY_DIR}/generated/protection_record.h)
if(FW_MOTOR_OUTPUT)
    set(RECORD_CHECK ${CMAKE_COMMAND} -DSRC_DIR=${BOARD_DIR} -DRECORD=${FW_IPROPI_PROTECTION_RECORD}
        -DOUT=${RECORD_H} -P ${BOARD_DIR}/check_protection_record.cmake)
    execute_process(COMMAND ${RECORD_CHECK} RESULT_VARIABLE rc ERROR_VARIABLE err)
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "${err}")
    endif()
    add_custom_target(protection_record COMMAND ${RECORD_CHECK} BYPRODUCTS ${RECORD_H}
                      COMMENT "IPROPI protection record" VERBATIM)
    set(FW_OUTPUTS_WORD OPEN)
    set(FW_OUTPUTS_LOCKED 0)
else()
    set(FW_OUTPUTS_WORD locked)
    set(FW_OUTPUTS_LOCKED 1)
endif()

# Build identity (board design 3.7): what is on the board must be readable from the ELF. It is
# regenerated at every build, not only at configure time, so a new commit or a dirty tree in the
# same build directory reaches the ELF.
set(BUILD_INFO_H ${CMAKE_CURRENT_BINARY_DIR}/generated/build_info.h)
# The IWDG stall test build is not told apart by STATE.flags: its build kind, the USB product
# string and the image name say it (board design 9).
if(FW_TEST_IWDG_STALL)
    set(FW_BUILD_KIND "${FW_BUILD}+iwdg-stall-${FW_TEST_IWDG_STALL}s")
    set(FW_IMAGE fw_iwdg_stall)
else()
    set(FW_BUILD_KIND "${FW_BUILD}")
    set(FW_IMAGE fw)
endif()
add_custom_target(build_info
    COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${BOARD_DIR} -DIN=${BOARD_DIR}/build_info.h.in
            -DOUT=${BUILD_INFO_H} -DFW_BUILD=${FW_BUILD_KIND} -DFW_LINK=${FW_LINK}
            -DFW_OUTPUTS_WORD=${FW_OUTPUTS_WORD} -DFW_OUTPUTS_LOCKED=${FW_OUTPUTS_LOCKED}
            "-DFW_TOOLCHAIN_VERSION=${FW_TOOLCHAIN_VERSION}"
            -P ${BOARD_DIR}/gen_build_info.cmake
    BYPRODUCTS ${BUILD_INFO_H}
    COMMENT "build identity"
    VERBATIM)

# The core, compiled for the MCU from the same sources the host tests use.
file(GLOB FW_CORE_ARM_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../../core/src/*.c)
add_library(fw_core_arm STATIC ${FW_CORE_ARM_SOURCES})
target_include_directories(fw_core_arm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../core/include)
target_compile_options(fw_core_arm PRIVATE -O2 -Wdouble-promotion -fstack-usage -fcallgraph-info=su)

set(BOARD_SOURCES
    ${BOARD_DIR}/startup_stm32h743.S
    ${BOARD_DIR}/src/vectors.c
    ${BOARD_DIR}/src/main.c
    ${BOARD_DIR}/src/clock.c
    ${BOARD_DIR}/src/timebase.c
    ${BOARD_DIR}/src/iwdg.c
    ${BOARD_DIR}/src/reset_cause.c
    ${BOARD_DIR}/src/rng.c
    ${BOARD_DIR}/src/led.c
    ${BOARD_DIR}/src/gpio.c
    ${BOARD_DIR}/src/drv_spi.c
    ${BOARD_DIR}/src/kth_spi.c
    ${BOARD_DIR}/src/encoder.c
    ${BOARD_DIR}/src/adc.c
    ${BOARD_DIR}/src/board_config.c
    ${BOARD_DIR}/src/board_rules.c)
# Motor outputs (board design 3.6, motor output design): the locked file, or behind the gate the
# TIM1 one -- the only board source that may name TIM1.
if(FW_MOTOR_OUTPUT)
    set(MOTOR_OUT ${BOARD_DIR}/src/motor_out_tim1.c)
else()
    set(MOTOR_OUT ${BOARD_DIR}/src/motor_out_locked.c)
endif()

# HIL (board design 9): the vehicle model shared with the host model endpoint. A plain path, not a
# generator expression, so the per-source TIM1 guard below reaches it.
if(FW_BUILD STREQUAL "hil")
    list(APPEND BOARD_SOURCES ${BOARD_DIR}/src/board_hal_hil.c
                              ${CMAKE_CURRENT_SOURCE_DIR}/../../plant/plant.c)
else()
    list(APPEND BOARD_SOURCES ${BOARD_DIR}/src/board_hal.c)
endif()

# The link (board design 5). TinyUSB is vendored (firmware/third_party/MANIFEST.yaml) and built as
# its own library: its sources are not ours to hold to -Wconversion, but they are compiled for
# the stack check and see the TIM1 guard like every board source.
if(FW_LINK STREQUAL "usb")
    list(APPEND BOARD_SOURCES ${BOARD_DIR}/src/link_usb.c ${BOARD_DIR}/src/usb_descriptors.c)
    set(TUSB ${TP}/tinyusb/src)
    add_library(tinyusb_arm STATIC
        ${TUSB}/tusb.c
        ${TUSB}/common/tusb_fifo.c
        ${TUSB}/device/usbd.c
        ${TUSB}/class/cdc/cdc_device.c
        ${TUSB}/portable/synopsys/dwc2/dcd_dwc2.c
        ${TUSB}/portable/synopsys/dwc2/dwc2_common.c)
    target_include_directories(tinyusb_arm PUBLIC ${TUSB} ${BOARD_DIR}/include)
    target_include_directories(tinyusb_arm SYSTEM PUBLIC ${TP}/cmsis_core/CMSIS/Core/Include
                                                         ${TP}/cmsis_device_h7/Include)
    target_compile_definitions(tinyusb_arm PUBLIC STM32H743xx)
    target_compile_options(tinyusb_arm PRIVATE -O2 -w -fstack-usage -fcallgraph-info=su
        -include ${BOARD_DIR}/include/board/lock_guard.h)
else()
    list(APPEND BOARD_SOURCES ${BOARD_DIR}/src/link_uart7.c)
endif()
string(TOUPPER "${FW_LINK}" FW_LINK_UPPER)

add_executable(fw ${BOARD_SOURCES} ${MOTOR_OUT})
# Every board C source sees TIM1 poisoned (board design 3.6), in output builds too; only the
# motor_out file is left out of the guard.
set(GUARD "-include;${BOARD_DIR}/include/board/lock_guard.h")
foreach(src ${BOARD_SOURCES})
    if(src MATCHES "\\.c$")
        set_source_files_properties(${src} PROPERTIES COMPILE_OPTIONS "${GUARD}")
    endif()
endforeach()
set_source_files_properties(${BOARD_DIR}/src/vectors.c PROPERTIES COMPILE_OPTIONS
    "${GUARD};-Wno-override-init;-Wno-pedantic")
if(NOT FW_MOTOR_OUTPUT)
    set_source_files_properties(${MOTOR_OUT} PROPERTIES COMPILE_OPTIONS "${GUARD}")
endif()
add_dependencies(fw build_info)
if(FW_MOTOR_OUTPUT)
    add_dependencies(fw protection_record)
    target_compile_definitions(fw PRIVATE FW_MOTOR_OUTPUT_OPEN=1)
endif()
set_target_properties(fw PROPERTIES SUFFIX ".elf" OUTPUT_NAME ${FW_IMAGE})
target_include_directories(fw PRIVATE ${BOARD_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/generated
                                      ${CMAKE_CURRENT_SOURCE_DIR}/../../plant)
target_include_directories(fw SYSTEM PRIVATE ${TP}/cmsis_core/CMSIS/Core/Include
                                               ${TP}/cmsis_device_h7/Include)
target_compile_definitions(fw PRIVATE STM32H743xx FW_BUILD_${FW_BUILD}=1 FW_LINK_${FW_LINK_UPPER}=1)
if(FW_TEST_IWDG_STALL)
    target_compile_definitions(fw PRIVATE FW_TEST_IWDG_STALL_S=${FW_TEST_IWDG_STALL})
endif()
target_compile_options(fw PRIVATE $<$<COMPILE_LANGUAGE:C>:-O2 -Wdouble-promotion -fstack-usage -fcallgraph-info=su>)
target_link_options(fw PRIVATE -T${BOARD_DIR}/stm32h743vi_dtcm.ld -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FW_IMAGE}.map
                    -Wl,--print-memory-usage)
target_link_libraries(fw PRIVATE fw_core_arm m)
if(FW_LINK STREQUAL "usb")
    target_link_libraries(fw PRIVATE tinyusb_arm)
endif()
set_property(TARGET fw APPEND PROPERTY LINK_DEPENDS ${BOARD_DIR}/stm32h743vi_dtcm.ld)
# Vehicle values from config/real/vehicle.yaml, never written twice (plan P0-6).
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/vehicle_header.cmake)
fw_vehicle_header(fw ${CMAKE_CURRENT_BINARY_DIR}/generated)

add_custom_command(TARGET fw POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:fw> ${CMAKE_CURRENT_BINARY_DIR}/${FW_IMAGE}.bin
    COMMAND ${CMAKE_SIZE} -A $<TARGET_FILE:fw> > ${CMAKE_CURRENT_BINARY_DIR}/${FW_IMAGE}.size.txt
    VERBATIM)
