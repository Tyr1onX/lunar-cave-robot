#******************************************************************************
# @file    gcc-arm-none-eabi.cmake
# @brief   CMake toolchain file: arm-none-eabi-gcc (GNU Tools for STM32)
#
#          工具链查找顺序：
#            1. 环境变量 STM32CUBECLT_PATH 指向的 ST 官方 GNU Tools 根目录
#            2. 系统 PATH 中的 arm-none-eabi-gcc
#******************************************************************************

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(TOOLCHAIN_PREFIX                arm-none-eabi-)

#---------------- Toolchain location ----------------
if(DEFINED ENV{STM32CUBECLT_PATH} AND NOT "$ENV{STM32CUBECLT_PATH}" STREQUAL "")
    file(TO_CMAKE_PATH "$ENV{STM32CUBECLT_PATH}" TOOLCHAIN_ROOT)
endif()

if(TOOLCHAIN_ROOT)
    if(WIN32)
        set(TOOLCHAIN_EXECUTABLE_SUFFIX ".exe")
    else()
        set(TOOLCHAIN_EXECUTABLE_SUFFIX "")
    endif()
    set(CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXECUTABLE_SUFFIX}")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_EXECUTABLE_SUFFIX}")
    set(CMAKE_ASM_COMPILER "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXECUTABLE_SUFFIX}")
    set(CMAKE_OBJCOPY "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_EXECUTABLE_SUFFIX}")
    set(CMAKE_SIZE "${TOOLCHAIN_ROOT}/bin/${TOOLCHAIN_PREFIX}size${TOOLCHAIN_EXECUTABLE_SUFFIX}")
else()
    set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}gcc")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
    set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
    set(CMAKE_OBJCOPY "${TOOLCHAIN_PREFIX}objcopy")
    set(CMAKE_SIZE "${TOOLCHAIN_PREFIX}size")
endif()

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)
