cmake_minimum_required(VERSION 3.10.0)

set(TARGET_PLATFORM "arm-none-linux-gnueabihf" CACHE STRING
    "Select target platform: arm-none-linux-gnueabihf or local")

if(TARGET_PLATFORM STREQUAL "arm-none-linux-gnueabihf")
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_SOURCE_DIR}/cmake/arm-none-linux-gnueabihf.cmake"
        CACHE FILEPATH "Toolchain file")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
    set(_DEFAULT_STATIC_LINKING ON)
    set(_DEFAULT_STRIP_BINARY ON)
elseif(TARGET_PLATFORM STREQUAL "local")
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type")
    set(_DEFAULT_STATIC_LINKING OFF)
    set(_DEFAULT_STRIP_BINARY OFF)
else()
    message(FATAL_ERROR "Unsupported TARGET_PLATFORM: ${TARGET_PLATFORM}")
endif()

if(NOT DEFINED STRIP_RELEASE_BINARY OR STRIP_RELEASE_BINARY STREQUAL "")
    set(STRIP_RELEASE_BINARY ${_DEFAULT_STRIP_BINARY} CACHE BOOL
        "Strip the final Release executable")
else()
    set(STRIP_RELEASE_BINARY ${STRIP_RELEASE_BINARY} CACHE BOOL
        "Strip the final Release executable")
endif()

if(NOT DEFINED USE_STATIC_LINKING OR USE_STATIC_LINKING STREQUAL "")
    set(USE_STATIC_LINKING ${_DEFAULT_STATIC_LINKING} CACHE BOOL "Use static linking")
else()
    set(USE_STATIC_LINKING ${USE_STATIC_LINKING} CACHE BOOL "Use static linking")
endif()

project(rollercan_i2c_calibrator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "Target platform: ${TARGET_PLATFORM}")
message(STATUS "Static linking: ${USE_STATIC_LINKING}")
message(STATUS "Strip Release binary: ${STRIP_RELEASE_BINARY}")

add_subdirectory(src)
