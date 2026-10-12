#******************************************************************************
# @file    CMakeLists.txt
# @brief   CLion / GCC 鏋勫缓鑴氭湰锛堟浛浠?Keil MDK-ARM 宸ョ▼锛?#          鐩爣鑺墖锛歋TM32H723VGTx锛圕ortex-M7, 1MB Flash锛?#          宸ュ叿閾? 锛歛rm-none-eabi-gcc锛堣 cmake/gcc-arm-none-eabi.cmake锛?#          鍛戒护琛屾瀯寤虹ず渚嬶細
#            cmake --preset Debug
#            cmake --build --preset Debug
#          鎴栨墜鍔細
#            cmake -B build/Debug -G Ninja ^
#                  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake ^
#                  -DCMAKE_BUILD_TYPE=Debug
#            cmake --build build/Debug
#          婧愭枃浠舵竻鍗曚笌 Keil 宸ョ▼ CtrlBoard-H7_IMU.uvprojx 淇濇寔涓€鑷淬€?#******************************************************************************

cmake_minimum_required(VERSION 3.22)

project(CtrlBoard-H7_IMU C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    
set(CMAKE_BUILD_TYPE Debug)
endif()

set(TARGET_NAME ${PROJECT_NAME})

#---------------- MCU 缂栬瘧/閾炬帴鍙傛暟 ----------------
set(MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS   "${MCU_FLAGS} -fdata-sections -ffunction-sections -Wall -MMD -MP")
set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -x assembler-with-cpp -MMD -MP")

# 娉ㄦ剰锛氬惈璺緞鐨勯摼鎺ュ弬鏁颁笉鑳界敤瀛楃涓叉嫾杩?*_LINKER_FLAGS锛堣矾寰勫惈绌烘牸浼氳鎷嗗垎锛夛紝
# NOTE: see link options below
# target_link_options 閫愭潯浼犻€掞紙瑙佷笅鏂?
# add_executable ... (see below)
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} --specs=nano.specs -Wl,--gc-sections")

set(CMAKE_C_FLAGS_DEBUG   "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O2 -g0")

#---------------- 鍏ㄥ眬瀹忓畾涔夛紙涓?Keil 宸ョ▼涓€鑷达級 ----------------
add_compile_definitions(
    USE_HAL_DRIVER
    STM32H723xx
    USE_PWR_LDO_SUPPLY
)

#---------------- 澶存枃浠惰矾寰勶紙涓?Keil IncludePath 涓€鑷达級 ----------------
include_directories(
    Core/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    User/Algorithm/EKF
    User/Algorithm/kalman
    User/Algorithm/mahony
    User/APP
    User/Bsp
    User/Devices/BMI088
    User/Devices/HM_Motor
    User/Controller
    User/Lib
)

#---------------- 婧愭枃浠?----------------
# Core锛坢ain/freertos/澶栬鍒濆鍖?涓柇/绯荤粺鏂囦欢锛?
file(GLOB CORE_SOURCES CONFIGURE_DEPENDS "Core/Src/*.c")

# HAL 搴擄紙鎺掗櫎妯℃澘鏂囦欢锛屾ā鏉夸笌宸ョ▼鍐呮枃浠堕噸鍚嶅啿绐侊級
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS "Drivers/STM32H7xx_HAL_Driver/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*_template\\.c$")

# FreeRTOS 鍐呮牳 + CMSIS-RTOS v1 + GCC/ARM_CM4F 绉绘灞傦紙涓?Keil 鐨?RVDS/ARM_CM4F 瀵瑰簲锛?
set(FREERTOS_SOURCES
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# 鐢ㄦ埛浠ｇ爜
set(USER_SOURCES
    User/APP/Chaiss_Communication.c
    User/APP/chassis_command.c
    User/APP/chassis_safety.c
    User/APP/INS_task.c
    User/APP/chassis_task.c
    User/APP/kinematics.c
    User/Bsp/bsp_dwt.c
    User/Bsp/bsp_PWM.c
    User/Bsp/rc_control_mode.c
    User/Bsp/rc_sbus.c
    User/Bsp/can_bsp.c
        User/Controller/controller.c
    User/Devices/BMI088/BMI088driver.c
    User/Devices/BMI088/BMI088Middleware.c
    User/Devices/HM_Motor/hm_motor.c
    User/Algorithm/EKF/QuaternionEKF.c
    User/Algorithm/kalman/kalman_filter.c
    User/Algorithm/mahony/mahony_filter.c
    User/Lib/user_lib.c
)

# GCC 鐗堝惎鍔ㄦ枃浠讹紙Keil 鐨?startup_stm32h723xx.s 鏄?ARM 姹囩紪璇硶锛屼笉鑳界敤浜?GCC锛?
set(STARTUP_SOURCE Core/Startup/startup_stm32h723xx.s)

add_executable(${TARGET_NAME}
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${USER_SOURCES}
    ${STARTUP_SOURCE}
)

#---------------- 閾炬帴閫夐」锛坱arget_link_options 浼氬鍚┖鏍艰矾寰勮嚜鍔ㄥ姞寮曞彿锛?----------------
target_link_options(${TARGET_NAME} PRIVATE
    "-T${CMAKE_SOURCE_DIR}/STM32H723VGTX_FLASH.ld"
    "-Wl,-Map=${PROJECT_NAME}.map,--cref"
    "-Wl,--print-memory-usage"
)

#---------------- 閾炬帴搴?----------------
# CMSIS-DSP 棰勭紪璇戝簱锛圙CC 鐗堬紝Cortex-M7 鍙岀簿搴?FPU锛? 鏁板搴?
target_link_libraries(${TARGET_NAME}
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a
    m
    c
    nosys
)

#---------------- 鏋勫缓鍚庯細鐢熸垚 hex/bin 骞舵墦鍗板ぇ灏?----------------
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex   $<TARGET_FILE:${TARGET_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${TARGET_NAME}>
    COMMENT "Generating ${PROJECT_NAME}.hex / ${PROJECT_NAME}.bin"
)

#---------------- 鐑у綍鐩爣锛圫T-LINK锛孲TM32CubeProgrammer锛?----------------
find_program(STM32_PROGRAMMER_CLI NAMES STM32_Programmer_CLI)
if(STM32_PROGRAMMER_CLI)
    add_custom_target(flash
        COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD -d $<TARGET_FILE:${TARGET_NAME}> -rst
        DEPENDS ${TARGET_NAME}
        COMMENT "Flashing ${TARGET_NAME}.elf via ST-LINK (SWD)"
    )
endif()

# CMSIS-DAP / OpenOCD download support. Override OPENOCD_EXECUTABLE at configure
# time when OpenOCD is installed outside PATH. This target is never run by the
# repository's offline build or validation scripts.
find_program(OPENOCD_EXECUTABLE NAMES openocd openocd.exe)
set(CMSIS_DAP_OPENOCD_CONFIG
    "${CMAKE_SOURCE_DIR}/openocd/cmsis-dap-stm32h7.cfg")

if(OPENOCD_EXECUTABLE)
    add_custom_target(flash-cmsis-dap
        COMMAND "${OPENOCD_EXECUTABLE}"
            -f "${CMSIS_DAP_OPENOCD_CONFIG}"
            -c "program $<TARGET_FILE_NAME:${TARGET_NAME}> verify reset exit"
        DEPENDS ${TARGET_NAME}
        USES_TERMINAL
        COMMENT "Flashing ${TARGET_NAME} through CMSIS-DAP/OpenOCD"
    )
else()
    message(WARNING
        "OpenOCD was not found at '${OPENOCD_EXECUTABLE}'. "
        "Set OPENOCD_EXECUTABLE to enable the flash-cmsis-dap target.")
endif()








