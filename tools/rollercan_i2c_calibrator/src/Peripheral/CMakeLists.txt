set(TARGET_NAME peripheral_lib)

add_library(${TARGET_NAME} STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/i2c/bsp_i2c.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/i2c/linux_i2c_bus.cpp"
)

target_include_directories(${TARGET_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/i2c"
)
