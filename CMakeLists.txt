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
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    # TIM1 triggers the DMA-backed fast FOC path; SmartKnob feedback runs in
    # ControlTask. An unoptimised build is not suitable for a flashed image.
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ROLLERCAN)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Collect user source files
file(GLOB MYFILE_SOURCES "MyFile/src/*.c")
set(U8G2_SOURCES
    U8g2_lib/u8g2_bitmap.c
    U8g2_lib/u8g2_box.c
    U8g2_lib/u8g2_buffer.c
    U8g2_lib/u8g2_circle.c
    U8g2_lib/u8g2_d_memory.c
    U8g2_lib/u8g2_d_setup.c
    U8g2_lib/u8g2_font.c
    U8g2_lib/u8g2_fonts.c
    U8g2_lib/u8g2_hvline.c
    U8g2_lib/u8g2_intersection.c
    U8g2_lib/u8g2_line.c
    U8g2_lib/u8g2_ll_hvline.c
    U8g2_lib/u8g2_setup.c
    U8g2_lib/u8x8_8x8.c
    U8g2_lib/u8x8_byte.c
    U8g2_lib/u8x8_cad.c
    U8g2_lib/u8x8_d_ssd1306_64x48.c
    U8g2_lib/u8x8_display.c
    U8g2_lib/u8x8_gpio.c
    U8g2_lib/u8x8_setup.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MYFILE_SOURCES}
    ${U8G2_SOURCES}
    # Additional user sources (not managed by CubeMX)
    Core/Src/flash.c
    Core/Src/i2c_ex.c
    App/src/app_rtos.cpp
    App/src/fast_control_link.cpp
    App/src/runtime_metrics.cpp
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # CMSIS-DSP
    Drivers/CMSIS/DSP/Include
    # User libraries
    MyFile/inc
    U8g2_lib
    App/inc
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ROLLERCAN_USE_FREERTOS=1
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

if(CMAKE_OBJCOPY)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        # The application is linked at 0x08002000. For standalone development,
        # mirror its vector table at 0x08000000 so a cold reset reaches the
        # current Reset_Handler without relying on the production bootloader.
        COMMAND ${CMAKE_OBJCOPY} -O binary --only-section=.isr_vector
                $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
                ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_boot_vector.bin
        COMMAND ${CMAKE_OBJCOPY} -I ihex -O ihex
                --add-section .boot_vector=${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_boot_vector.bin
                --set-section-flags .boot_vector=alloc,load,readonly,data
                --change-section-address .boot_vector=0x08000000
                ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
                ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_standalone.hex
        COMMENT "Generating application and standalone HEX/BIN images"
        VERBATIM
    )
endif()
