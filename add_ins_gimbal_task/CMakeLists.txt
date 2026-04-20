cmake_minimum_required(VERSION 3.22)

# 启用 printf 的浮点支持（必须加这行）
add_link_options(-u _printf_float)
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
set(CMAKE_PROJECT_NAME ins_task)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        application/INS_task.c
        application/INS_task.h
        application/struct_typedef.h
        application/test_task.c
        application/test_task.h
        application/bsp/imu/bsp_delay.c
        application/bsp/imu/bsp_delay.h
        application/bsp/imu/bsp_imu_pwm.c
        application/bsp/imu/bsp_imu_pwm.h
        application/bsp/imu/bsp_led.h
        application/bsp/imu/bsp_led.c
        application/bsp/imu/bsp_spi.c
        application/bsp/imu/bsp_spi.h
        application/components/algorithm/MahonyAHRS.h
        application/components/algorithm/MahonyAHRS.C
        application/components/algorithm/user_lib.h
        application/components/algorithm/user_lib.c
        application/components/controller/pid.c
        application/components/controller/pid.h
        application/components/devices/BMI088driver.h
        application/components/devices/BMI088driver.c
        application/components/devices/BMI088Middleware.h
        application/components/devices/BMI088Middleware.c
        application/components/devices/BMI088reg.h
        application/components/devices/ist8310driver.c
        application/components/devices/ist8310driver.h
        application/components/devices/ist8310driver_middleware.h
        application/components/devices/ist8310driver_middleware.c

        translate/all_tou_h.h
        translate/remote/remote_control.h
        translate/remote/remote_control.c
        translate/remote/bsp_rc.c
        translate/remote/bsp_rc.h
        translate/can/bsp_can.c
        translate/can/bsp_can.h
        translate/can/CAN_receive.c
        translate/can/CAN_receive.h
        translate/modules/oled/oledfont.h
        translate/modules/oled/OLED.h
        translate/modules/oled/OLED.c
        translate/modules/usart_printf/usart_printf_task.c
        translate/modules/usart_printf/usart_printf_task.h
        translate/Task/gimbal/gimbal_pid_control.c
        translate/Task/gimbal/gimbal_pid_comtrol.h
        translate/Task/gimbal/gimbal_task.c
        translate/Task/gimbal/gimbal_task.h
        translate/Task/gimbal/gimbal_behaviour.c
        translate/Task/gimbal/gimbal_behaviour.h
        "translate/Task/gimbal/can2LaunchTransmission .c"
        "translate/Task/gimbal/can2LaunchTransmission .h"
        translate/Task/shoot/shoot_task.c
        translate/Task/shoot/shoot_task.h

        i6x_remote/ix6.c
        i6x_remote/ix6.h
        imu_date_receive/chassis_imu_date.c
        imu_date_receive/chassis_imu_date.h

        bmi088_date_send/bmi088_date_send.c
        bmi088_date_send/bmi088_date_send.h
        yaw_date_send/yaw_date_send_task.c
        yaw_date_send/yaw_date_send_task.h


)



# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)



# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${CMAKE_SOURCE_DIR}/Core
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Core/Src

        ${CMAKE_SOURCE_DIR}/translate/Task/INS
        ${CMAKE_SOURCE_DIR}/translate
        ${CMAKE_SOURCE_DIR}/translate/can
        ${CMAKE_SOURCE_DIR}/translate/remote
        ${CMAKE_SOURCE_DIR}/translate/Task
        ${CMAKE_SOURCE_DIR}/translate/modules
        ${CMAKE_SOURCE_DIR}/translate/modules/usart_printf

        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/application
        ${CMAKE_SOURCE_DIR}/application/bsp
        ${CMAKE_SOURCE_DIR}/application/bsp/imu
        ${CMAKE_SOURCE_DIR}/application/components
        ${CMAKE_SOURCE_DIR}/application/components/algorithm
        ${CMAKE_SOURCE_DIR}/application/components/controller
        ${CMAKE_SOURCE_DIR}/application/components/devices
        ${CMAKE_SOURCE_DIR}/application/usart_printf

        ${CMAKE_SOURCE_DIR}/i6x_remote
        ${CMAKE_SOURCE_DIR}/imu_date_receive

        ${CMAKE_SOURCE_DIR}/bmi088_date_send

        ${CMAKE_SOURCE_DIR}/yaw_date_send

)


# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 添加包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

)

