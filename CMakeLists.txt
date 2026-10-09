cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
# 新增C++标准配置，适配嵌入式ARM编译场景
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME my_project1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 关键修复：补充CXX语言支持，让CMake能识别C++源文件
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    applications/buzzer_task.cpp
    applications/led_task.cpp
    applications/imu_task.cpp
    applications/plotter_task.cpp
    applications/uart_task.cpp
    applications/control_task.cpp

    
   
    sp_middleware/io/buzzer/buzzer.cpp
    sp_middleware/io/led/led.cpp 
    sp_middleware/io/dbus/dbus.cpp
    sp_middleware/io/plotter/plotter.cpp
    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/io/can/can.cpp
    sp_middleware/io/bmi088/bmi088.cpp
    sp_middleware/tools/mahony/mahony.cpp
    sp_middleware/tools/gimbal/gimbal.cpp
    sp_middleware/tools/pid/pid.cpp 
    sp_middleware/tools/math_tools/math_tools.cpp
    sp_middleware/tools/low_pass_filter/low_pass_filter.cpp
    sp_middleware/tools/mecanum/mecanum.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    sp_middleware/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# 增加版本判断，兼容不同版本ARM工具链，避免高版本编译器下找不到libob的报错
if(CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0)
    list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
endif()

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)
