cmake_minimum_required(VERSION 3.20)

project(smart_knob_host_tests C)

enable_testing()

add_executable(smart_knob_regression_tests
    smart_knob_regression_tests.c
    ../MyFile/src/smart_knob.c
    ../MyFile/src/smart_knob_modes.c
)

target_compile_features(smart_knob_regression_tests PRIVATE c_std_11)
target_include_directories(smart_knob_regression_tests PRIVATE
    stubs
    ../MyFile/inc
)

if(MSVC)
    target_compile_options(smart_knob_regression_tests PRIVATE /W4)
else()
    target_compile_options(smart_knob_regression_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_link_libraries(smart_knob_regression_tests PRIVATE m)
endif()

add_test(
    NAME coarse_strong_current_continuity
    COMMAND smart_knob_regression_tests coarse-current-continuity
)
add_test(
    NAME bounded_coarse_center_does_not_drift
    COMMAND smart_knob_regression_tests bounded-center-hold
)
add_test(
    NAME exact_center_commands_zero
    COMMAND smart_knob_regression_tests exact-center-zero
)
