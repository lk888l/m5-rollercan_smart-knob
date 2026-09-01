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

add_test(
    NAME local_profile_applies_from_preset_defaults
    COMMAND smart_knob_regression_tests local-profile
)

add_test(
    NAME menu_navigation_mode_is_private
    COMMAND smart_knob_regression_tests navigation-mode
)

add_executable(button_event_tests
    button_event_tests.c
    ../MyFile/src/button.c
)

target_compile_features(button_event_tests PRIVATE c_std_11)
target_include_directories(button_event_tests PRIVATE
    stubs
    ../MyFile/inc
)

if(MSVC)
    target_compile_options(button_event_tests PRIVATE /W4)
else()
    target_compile_options(button_event_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME button_double_click COMMAND button_event_tests double-click)
add_test(NAME button_long_press COMMAND button_event_tests long-press)

add_executable(local_rgb_state_tests
    local_rgb_state_tests.c
    ../MyFile/src/local_rgb.c
)

target_compile_features(local_rgb_state_tests PRIVATE c_std_11)
target_include_directories(local_rgb_state_tests PRIVATE
    stubs
    ../MyFile/inc
)

if(MSVC)
    target_compile_options(local_rgb_state_tests PRIVATE /W4)
else()
    target_compile_options(local_rgb_state_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME local_rgb_state_machine COMMAND local_rgb_state_tests)

add_executable(host_compatibility_tests
    host_compatibility_tests.c
    ../MyFile/src/host_control.c
    ../MyFile/src/smart_knob.c
    ../MyFile/src/smart_knob_modes.c
    ../MyFile/src/smart_knob_persistence.c
)

target_compile_features(host_compatibility_tests PRIVATE c_std_11)
target_include_directories(host_compatibility_tests PRIVATE
    stubs
    ../App/inc
    ../MyFile/inc
)

if(MSVC)
    target_compile_options(host_compatibility_tests PRIVATE /W4)
else()
    target_compile_options(host_compatibility_tests PRIVATE
        -Wall -Wextra -Wpedantic
    )
    target_link_libraries(host_compatibility_tests PRIVATE m)
endif()

add_test(NAME host_read_only_and_timeout
         COMMAND host_compatibility_tests authority-timeout)
add_test(NAME host_configuration_transaction_and_stop
         COMMAND host_compatibility_tests transaction-stop)
add_test(NAME host_partial_transaction_is_discardable
         COMMAND host_compatibility_tests partial-transaction)
add_test(NAME full_snapshot_roundtrip_and_crc
         COMMAND host_compatibility_tests snapshot-crc)
add_test(NAME legacy_migration_and_selective_edit
         COMMAND host_compatibility_tests legacy-selective)
add_test(NAME firmware_presets_match_host_table
         COMMAND host_compatibility_tests preset-table)
