cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME lunar_rover)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and CXX languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# C++ business modules (mirror the "others" group of MDK-ARM/lunar_rover.uvprojx)
set(USER_SOURCES
    others/src/app/Remote.c
    others/src/app/arm_safety_state.c
    others/src/app/Host.cpp
    others/src/bsp/bsp_can.cpp
    others/src/callcallback/can_callback.cpp
    others/src/callcallback/usb_callback.cpp
    others/src/obj/motor_dm.cpp
    others/src/obj/roboarm.cpp
    others/src/obj/roboarm_kinematics.cpp
    others/src/thread/task_error.cpp
    others/src/thread/task_host.cpp
    others/src/thread/task_robo_arm.cpp
    others/src/thread/task_start.cpp
    others/src/thread/variables.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SOURCES}
)

# Add include paths
set(USER_INCLUDE_DIRS
    others/inc
    others/inc/algorithm
    others/inc/app
    others/inc/bsp
    others/inc/modules
    others/inc/obj
    others/inc/thread
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Add LibXR
include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)
