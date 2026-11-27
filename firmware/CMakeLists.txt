# Host build of the hardware-independent firmware core and its tests, or (FW_BOARD set, with
# that board's toolchain file) the MCU build in boards/<board>/.
cmake_minimum_required(VERSION 3.16)
project(forklift_fw C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Werror -Wconversion -Wshadow)

set(FW_BOARD "" CACHE STRING "MCU board to cross-build (empty: host build and tests)")
if(FW_BOARD)
    enable_language(ASM)
    add_subdirectory(boards/${FW_BOARD})
    return()
endif()

file(GLOB FW_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/core/src/*.c)
add_library(fw_core STATIC ${FW_CORE_SOURCES})
target_include_directories(fw_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include)
target_link_libraries(fw_core PUBLIC m)
# The host core carries single-axis commissioning (fw-single) for its tests and the MCU model; the
# board compiles it in only for FW_SINGLE_AXIS (partial-parts plan 2026-11-25 PR-B).
target_compile_definitions(fw_core PUBLIC FW_CORE_SINGLE_AXIS=1)
# It also carries the bench extension (bench console plan A1, contract 8) for its tests and the MCU model; the
# board compiles it in only for FW_BENCH, so every other image keeps the bench types unknown.
target_compile_definitions(fw_core PUBLIC FW_CORE_BENCH=1)
# The same core without the bench extension, as every non-bench image compiles it: its test shows the bench
# types are unknown there (plan A1: a non-bench image never decodes 0x06/0x08).
add_library(fw_core_nobench STATIC ${FW_CORE_SOURCES})
target_include_directories(fw_core_nobench PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include)
target_link_libraries(fw_core_nobench PUBLIC m)
target_compile_definitions(fw_core_nobench PUBLIC FW_CORE_SINGLE_AXIS=1)

# The vehicle model shared by the host model endpoint and the MCU's HIL build (board design 9).
add_library(fw_plant STATIC ${CMAKE_CURRENT_SOURCE_DIR}/plant/plant.c)
target_include_directories(fw_plant PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/plant)
target_link_libraries(fw_plant PUBLIC fw_core)

enable_testing()
file(GLOB FW_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/core/test/test_*.c)
foreach(test_source ${FW_TESTS})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/test)
    target_link_libraries(${test_name} PRIVATE fw_core)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

add_executable(test_link_nobench ${CMAKE_CURRENT_SOURCE_DIR}/core/test/variants/test_link_nobench.c)
target_include_directories(test_link_nobench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/test)
target_link_libraries(test_link_nobench PRIVATE fw_core_nobench)
add_test(NAME test_link_nobench COMMAND test_link_nobench)

# Board logic that does not touch registers (reset cause, clock constants), host-tested.
add_executable(test_weact_h743_reset_cause
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/test/test_reset_cause.c
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/src/reset_cause.c)
target_include_directories(test_weact_h743_reset_cause PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/test ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/include)
target_link_libraries(test_weact_h743_reset_cause PRIVATE fw_core)
add_test(NAME test_weact_h743_reset_cause COMMAND test_weact_h743_reset_cause)

add_executable(test_weact_h743_board_pure
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/test/test_board_pure.c
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/src/board_rules.c)
target_include_directories(test_weact_h743_board_pure PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/test ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/include)
target_link_libraries(test_weact_h743_board_pure PRIVATE fw_core)
add_test(NAME test_weact_h743_board_pure COMMAND test_weact_h743_board_pure)

# motor_out_tim1.c on the host register model (motor output design 4.2): the fake device header in
# test/regmodel comes first on the include path.
set(W ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743)
add_library(weact_h743_regmodel STATIC ${W}/test/regmodel/regmodel.c ${W}/src/motor_out_tim1.c)
target_include_directories(weact_h743_regmodel PUBLIC ${W}/test/regmodel ${W}/include)
target_compile_definitions(weact_h743_regmodel PUBLIC FW_HOST_REGMODEL=1)
target_link_libraries(weact_h743_regmodel PUBLIC fw_core)
add_executable(test_weact_h743_motor_out ${W}/test/test_motor_out.c)
target_include_directories(test_weact_h743_motor_out PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/test)
target_link_libraries(test_weact_h743_motor_out PRIVATE weact_h743_regmodel)
add_test(NAME test_weact_h743_motor_out COMMAND test_weact_h743_motor_out)

add_executable(test_plant ${CMAKE_CURRENT_SOURCE_DIR}/plant/test_plant.c)
target_include_directories(test_plant PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/test)
target_link_libraries(test_plant PRIVATE fw_plant)
add_test(NAME test_plant COMMAND test_plant)

add_executable(test_weact_h743_link ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/test/test_link.c)
target_include_directories(test_weact_h743_link PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/test ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/include)
add_test(NAME test_weact_h743_link COMMAND test_weact_h743_link)

# The board's real configuration needs the generated vehicle header (FW_PYTHON).
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/vehicle_header.cmake)
if(FW_PYTHON)
    # The core driving motor_out_tim1.c on the register model (motor output design 4.2).
    add_executable(test_weact_h743_motor_out_e2e
        ${W}/test/test_motor_out_e2e.c ${W}/src/board_config.c ${W}/src/board_rules.c
        ${CMAKE_CURRENT_SOURCE_DIR}/plant/plant.c)
    target_include_directories(test_weact_h743_motor_out_e2e PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/test ${CMAKE_CURRENT_SOURCE_DIR}/plant)
    target_compile_definitions(test_weact_h743_motor_out_e2e PRIVATE FW_BUILD_normal=1)
    target_link_libraries(test_weact_h743_motor_out_e2e PRIVATE weact_h743_regmodel)
    fw_vehicle_header(test_weact_h743_motor_out_e2e ${CMAKE_CURRENT_BINARY_DIR}/generated_e2e)
    add_test(NAME test_weact_h743_motor_out_e2e COMMAND test_weact_h743_motor_out_e2e)

    # One configuration test per build kind (board design 2.2, 9).
    foreach(kind normal commission hil iwdg_stall commission_open normal_waived commission_single)
        set(t test_weact_h743_board_config_${kind})
        add_executable(${t}
            ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/test/test_board_config.c
            ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/src/board_config.c
            ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/src/board_rules.c)
        target_include_directories(${t} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/core/test ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/include)
        if(kind STREQUAL "iwdg_stall") # the S5 test build: a normal build that stalls (design 9)
            target_compile_definitions(${t} PRIVATE FW_BUILD_normal=1 FW_TEST_IWDG_STALL_S=5)
        elseif(kind STREQUAL "commission_open") # behind the gate (motor output design 2)
            target_compile_definitions(${t} PRIVATE FW_BUILD_commission=1 FW_MOTOR_OUTPUT_OPEN=1)
        elseif(kind STREQUAL "commission_single") # fw-single (partial-parts plan 2026-11-25)
            target_compile_definitions(${t} PRIVATE FW_BUILD_commission=1 FW_MOTOR_OUTPUT_OPEN=1
                                                    FW_COMMISSION_SINGLE=1)
        elseif(kind STREQUAL "normal_waived") # V1.0 e-stop waiver (plan 2026-11-18)
            target_compile_definitions(${t} PRIVATE FW_BUILD_normal=1 FW_MOTOR_OUTPUT_OPEN=1
                                                    FW_ESTOP_WAIVED=1)
        else()
            target_compile_definitions(${t} PRIVATE FW_BUILD_${kind}=1)
        endif()
        target_link_libraries(${t} PRIVATE fw_core)
        fw_vehicle_header(${t} ${CMAKE_CURRENT_BINARY_DIR}/generated_board_${kind})
        add_test(NAME ${t} COMMAND ${t})
    endforeach()
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tools/link_tool.c)
    add_executable(link_tool ${CMAKE_CURRENT_SOURCE_DIR}/tools/link_tool.c)
    target_link_libraries(link_tool PRIVATE fw_core)
endif()

# MCU model endpoint (sim-to-real plan P0-7): the core on a fake board in real time over a tty.
# Optional: it needs the vehicle header generated from config/real/vehicle.yaml by a Python that can
# import forklift_core (FW_PYTHON); the core tests above never depend on Python.
option(FW_MCU_MODEL "Build the host-only MCU model endpoint (needs FW_PYTHON)" OFF)
if(FW_MCU_MODEL)
    if(NOT FW_PYTHON)
        message(FATAL_ERROR "FW_MCU_MODEL needs -DFW_PYTHON=<python with forklift_core importable>")
    endif()
    get_filename_component(FW_REPO ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
    set(FW_VEHICLE_YAML ${FW_REPO}/config/real/vehicle.yaml CACHE FILEPATH "real vehicle config")
    set(FW_VEHICLE_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/vehicle_config.h)
    # The header also depends on the chassis model the config names; reconfigure when the config changes.
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${FW_REPO}/src
                ${FW_PYTHON} ${FW_REPO}/tools/real_vehicle_header.py --config ${FW_VEHICLE_YAML} --print-chassis
        OUTPUT_VARIABLE FW_CHASSIS_YAML OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _fw_rc)
    if(NOT _fw_rc EQUAL 0)
        message(FATAL_ERROR "could not read the chassis model named by ${FW_VEHICLE_YAML}")
    endif()
    if(NOT EXISTS ${FW_CHASSIS_YAML})
        message(FATAL_ERROR "chassis model ${FW_CHASSIS_YAML} named by ${FW_VEHICLE_YAML} not found")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${FW_VEHICLE_YAML})
    add_custom_command(
        OUTPUT ${FW_VEHICLE_HEADER}
        COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${FW_REPO}/src
                ${FW_PYTHON} ${FW_REPO}/tools/real_vehicle_header.py
                --config ${FW_VEHICLE_YAML} --out ${FW_VEHICLE_HEADER}
        DEPENDS ${FW_VEHICLE_YAML}
                ${FW_CHASSIS_YAML}
                ${FW_REPO}/tools/real_vehicle_header.py
                ${FW_REPO}/src/forklift_core/plant/real_vehicle.py
        COMMENT "Generating vehicle_config.h from ${FW_VEHICLE_YAML}")
    add_executable(mcu_model ${CMAKE_CURRENT_SOURCE_DIR}/tools/mcu_model.c ${FW_VEHICLE_HEADER})
    target_include_directories(mcu_model PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
    target_link_libraries(mcu_model PRIVATE fw_plant fw_core)
endif()
