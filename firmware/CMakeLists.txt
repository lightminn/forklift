# Host build of the hardware-independent firmware core and its tests.
# The MCU target build (toolchain, startup, board HAL) is added once the board is fixed.
cmake_minimum_required(VERSION 3.16)
project(forklift_fw C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Werror -Wconversion -Wshadow)

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
    add_custom_command(
        OUTPUT ${FW_VEHICLE_HEADER}
        COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${FW_REPO}/src
                ${FW_PYTHON} ${FW_REPO}/tools/real_vehicle_header.py
                --config ${FW_VEHICLE_YAML} --out ${FW_VEHICLE_HEADER}
        DEPENDS ${FW_VEHICLE_YAML}
                ${FW_REPO}/sim/models/dls08_measured/parameters.yaml
                ${FW_REPO}/tools/real_vehicle_header.py
                ${FW_REPO}/src/forklift_core/plant/real_vehicle.py
        COMMENT "Generating vehicle_config.h from ${FW_VEHICLE_YAML}")
    add_executable(mcu_model ${CMAKE_CURRENT_SOURCE_DIR}/tools/mcu_model.c ${FW_VEHICLE_HEADER})
    target_include_directories(mcu_model PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
    target_link_libraries(mcu_model PRIVATE fw_core)
endif()
