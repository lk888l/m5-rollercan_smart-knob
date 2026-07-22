set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_VERSION 1)

set(TOOLCHAIN_PATH "" CACHE PATH
    "Path to the cross-toolchain bin directory. Leave empty to search PATH and C:/kk_software/toolchain.")
set(TOOLCHAIN_PREFIX "arm-none-linux-gnueabihf" CACHE STRING
    "Cross-compiler executable prefix")
set(SYSROOT_PATH "" CACHE PATH
    "Path to the target sysroot. Leave empty to use the compiler's built-in sysroot.")

if(SYSROOT_PATH)
    if(NOT EXISTS "${SYSROOT_PATH}")
        message(FATAL_ERROR "SYSROOT_PATH does not exist: ${SYSROOT_PATH}")
    endif()
    set(CMAKE_SYSROOT "${SYSROOT_PATH}")
endif()

if(TOOLCHAIN_PATH)
    set(CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-gcc")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-g++")
    set(CMAKE_AR "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-ar")
    set(CMAKE_RANLIB "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-ranlib")
    set(CMAKE_STRIP "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-strip")
    set(CMAKE_OBJCOPY "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-objcopy")
    set(CMAKE_NM "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-nm")
else()
    file(GLOB _LOCAL_TOOLCHAIN_BIN_CANDIDATES
        "C:/kk_software/toolchain/*${TOOLCHAIN_PREFIX}*/bin")

    find_program(_C_COMPILER NAMES ${TOOLCHAIN_PREFIX}-gcc
        HINTS ${_LOCAL_TOOLCHAIN_BIN_CANDIDATES} PATHS ENV PATH REQUIRED)
    find_program(_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}-g++
        HINTS ${_LOCAL_TOOLCHAIN_BIN_CANDIDATES} PATHS ENV PATH REQUIRED)
    find_program(_AR NAMES ${TOOLCHAIN_PREFIX}-ar
        HINTS ${_LOCAL_TOOLCHAIN_BIN_CANDIDATES} PATHS ENV PATH REQUIRED)
    find_program(_RANLIB NAMES ${TOOLCHAIN_PREFIX}-ranlib
        HINTS ${_LOCAL_TOOLCHAIN_BIN_CANDIDATES} PATHS ENV PATH)
    find_program(_STRIP NAMES ${TOOLCHAIN_PREFIX}-strip
        HINTS ${_LOCAL_TOOLCHAIN_BIN_CANDIDATES} PATHS ENV PATH)
    find_program(_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}-objcopy
        HINTS ${_LOCAL_TOOLCHAIN_BIN_CANDIDATES} PATHS ENV PATH)
    find_program(_NM NAMES ${TOOLCHAIN_PREFIX}-nm
        HINTS ${_LOCAL_TOOLCHAIN_BIN_CANDIDATES} PATHS ENV PATH)

    set(CMAKE_C_COMPILER "${_C_COMPILER}")
    set(CMAKE_CXX_COMPILER "${_CXX_COMPILER}")
    set(CMAKE_AR "${_AR}")
    if(_RANLIB)
        set(CMAKE_RANLIB "${_RANLIB}")
    endif()
    if(_STRIP)
        set(CMAKE_STRIP "${_STRIP}")
    endif()
    if(_OBJCOPY)
        set(CMAKE_OBJCOPY "${_OBJCOPY}")
    endif()
    if(_NM)
        set(CMAKE_NM "${_NM}")
    endif()
endif()

if(CMAKE_SYSROOT)
    set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

message(STATUS "========== Toolchain Configuration ==========")
message(STATUS "System:       ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "AR:           ${CMAKE_AR}")
message(STATUS "Sysroot:      ${CMAKE_SYSROOT}")
message(STATUS "============================================")
