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
