add_subdirectory(Peripheral)
add_subdirectory(HardWare)
add_subdirectory(App)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    app_lib
    hardware_rollercan_lib
    peripheral_lib
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(USE_STATIC_LINKING)
    target_link_options(${PROJECT_NAME} PRIVATE
        -static
        -static-libgcc
        -static-libstdc++
        -Wl,--gc-sections
    )
endif()

if(STRIP_RELEASE_BINARY AND CMAKE_BUILD_TYPE STREQUAL "Release" AND CMAKE_STRIP)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND "${CMAKE_STRIP}" "$<TARGET_FILE:${PROJECT_NAME}>"
        COMMENT "Stripping ${PROJECT_NAME}"
    )
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
