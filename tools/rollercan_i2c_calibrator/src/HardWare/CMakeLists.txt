set(TARGET_NAME hardware_rollercan_lib)

add_library(${TARGET_NAME} STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/rollercan/rollercan.cpp"
)

target_include_directories(${TARGET_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(${TARGET_NAME} PUBLIC peripheral_lib)
