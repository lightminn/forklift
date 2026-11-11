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

enable_testing()
file(GLOB FW_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/core/test/test_*.c)
foreach(test_source ${FW_TESTS})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/test)
    target_link_libraries(${test_name} PRIVATE fw_core)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

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

add_executable(test_weact_h743_link ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/test/test_link.c)
target_include_directories(test_weact_h743_link PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/test ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/include)
add_test(NAME test_weact_h743_link COMMAND test_weact_h743_link)

# The board's real configuration needs the generated vehicle header (FW_PYTHON).
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/vehicle_header.cmake)
if(FW_PYTHON)
    add_executable(test_weact_h743_board_config
        ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/test/test_board_config.c
        ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/src/board_config.c
        ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/src/board_rules.c)
    target_include_directories(test_weact_h743_board_config PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/test ${CMAKE_CURRENT_SOURCE_DIR}/boards/weact_h743/include)
    target_link_libraries(test_weact_h743_board_config PRIVATE fw_core)
    fw_vehicle_header(test_weact_h743_board_config ${CMAKE_CURRENT_BINARY_DIR}/generated_board)
    add_test(NAME test_weact_h743_board_config COMMAND test_weact_h743_board_config)
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
    target_link_libraries(mcu_model PRIVATE fw_core)
endif()
