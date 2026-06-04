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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME damiao)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
        :libarm_cortexM7lfsp_math.a
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        BSP/algorithm/pid.c
        BSP/algorithm/crc.c
        BSP/algorithm/rls.c
        BSP/algorithm/user_lib.c
        BSP/algorithm/pid_enhanced.c
        BSP/algorithm/vmc.c
        BSP/algorithm/lqr.c
        BSP/usart/bsp_uart.c
        BSP/can/bsp_can.c
        BSP/dwt/bsp_dwt.c
        BSP/pwm/bsp_pwm.c
        Module/motor/dji/m3508/m3508.c
        Module/motor/DJI/GM6020/gm6020.c
        Module/motor/DJI/M2006/m2006.c
        Module/motor/LK/lk.c
        Module/motor/DAMIAO/DM8009P/dm8009p.c
        Module/motor/DAMIAO/DM4310/dm4310.c
        Module/motor/RS/rs02.c
        Module/DBUS/dbus.c
        Module/NUC/nuc.c
        Module/NX/nx.c
        Module/Referee/referee.c
        Module/COMM/comm.c
        Module/COMM/CBoard_chassis.c
        Module/COMM/CBoard_gimbal.c
        Module/IMU/BMI088/BMI088Middleware.c
        Module/IMU/BMI088/BMI088driver.c
        BSP/algorithm/kalman_filter.c
        Module/IMU/EKF/QuaternionEKF.c
        Module/IMU/INS/ins.c
        Module/IMU/HI12/hi12.c
        Module/Super_Cap/super_cap.c
        Module/Power_Ctrl/power_ctrl.c
        Module/Power_Ctrl/power_ctrl_param_get.c
        Module/LED/led.c
        Module/LED/ws2812.c
        Module/VOFA/vofa.c
        APP/task_init.c
        APP/module_init.c
        APP/chassis_task.c
        APP/observe_task.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        BSP
        Module
        APP
        BSP/algorithm
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        ARM_MATH_CM7;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING;
        MADE_BY_CMAKE

        CHASSIS
        #INFANTRY     # 步兵轮毂电机为M3508，关节电机为DM8009P
        HERO        # 英雄轮毂电机为MF9025，关节电机为DM8009P
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a # cubemx不会自己链接armmath库
    # Add user defined libraries
)
