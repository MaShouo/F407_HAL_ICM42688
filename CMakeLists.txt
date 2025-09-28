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
set(CMAKE_PROJECT_NAME LED)

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
        TMT/TMT.c
        TMT/TMT.h
        User_Core/Inc/Delay.h
        User_Core/Inc/icm42688.h
        User_Core/Inc/Key.h
        User_Core/Inc/OLED.h
        User_Core/Inc/OLED_Data.h
        User_Core/Src/Delay.c
        User_Core/Src/icm42688.c
        User_Core/Src/Key.c
        User_Core/Src/OLED.c
        User_Core/Src/OLED_Data.c
        User_Core/Inc/Task.h
        User_Core/Src/Task.c
        User_Core/Inc/Motor.h
        User_Core/Src/Motor.c
        User_Core/Src/PID.c
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

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
include_directories(${PROJECT_SOURCE_DIR}/TMT)
include_directories(${PROJECT_SOURCE_DIR}/User_Core/Inc)
include_directories(${PROJECT_SOURCE_DIR}/User_Core/Src)

