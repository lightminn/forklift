# CMake toolchain file: Arm GNU Toolchain 13.3.rel1 for the WeAct H743 board (board design
# 2026-11-12 section 2.1). The toolchain directory is given at configure time and never written
# into the repository:
#   cmake -S firmware -B build/mcu -DFW_BOARD=weact_h743 \
#         -DARM_GNU_TOOLCHAIN_DIR=<dir with bin/arm-none-eabi-gcc> \
#         -DCMAKE_TOOLCHAIN_FILE=firmware/boards/weact_h743/arm-gnu-13.3.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# try_compile runs this file again in a fresh project; pass the directory through.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_GNU_TOOLCHAIN_DIR)
if(NOT ARM_GNU_TOOLCHAIN_DIR AND DEFINED ENV{ARM_GNU_TOOLCHAIN_DIR})
    set(ARM_GNU_TOOLCHAIN_DIR $ENV{ARM_GNU_TOOLCHAIN_DIR})
endif()
if(NOT ARM_GNU_TOOLCHAIN_DIR)
    message(FATAL_ERROR "set -DARM_GNU_TOOLCHAIN_DIR=<Arm GNU Toolchain 13.3.rel1 directory>")
endif()

set(_bin ${ARM_GNU_TOOLCHAIN_DIR}/bin)
set(CMAKE_C_COMPILER ${_bin}/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${_bin}/arm-none-eabi-gcc)
set(CMAKE_OBJCOPY ${_bin}/arm-none-eabi-objcopy CACHE FILEPATH "")
set(CMAKE_OBJDUMP ${_bin}/arm-none-eabi-objdump CACHE FILEPATH "")
set(CMAKE_SIZE ${_bin}/arm-none-eabi-size CACHE FILEPATH "")
set(CMAKE_NM ${_bin}/arm-none-eabi-nm CACHE FILEPATH "")

# The pinned version, checked here so that a different toolchain cannot reproduce "the same"
# size and stack reports by accident.
execute_process(COMMAND ${CMAKE_C_COMPILER} --version OUTPUT_VARIABLE _ver ERROR_QUIET)
string(REGEX MATCH "^[^\n]*" FW_TOOLCHAIN_VERSION "${_ver}")
if(NOT FW_TOOLCHAIN_VERSION MATCHES "Arm GNU Toolchain 13\\.3\\.Rel1")
    message(FATAL_ERROR "need Arm GNU Toolchain 13.3.Rel1, found: '${FW_TOOLCHAIN_VERSION}'")
endif()

set(FW_CPU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
string(REPLACE ";" " " _cpu "${FW_CPU_FLAGS}")
set(CMAKE_C_FLAGS_INIT "${_cpu} -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS_INIT "${_cpu} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_cpu} --specs=nano.specs -nostartfiles -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
