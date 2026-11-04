cmake_minimum_required(VERSION 3.20)

project(smart_knob_host_tests C)

enable_testing()

add_executable(motion_estimator_tests
    motion_estimator_tests.c
    ../MyFile/src/motion_estimator.c
)
target_compile_features(motion_estimator_tests PRIVATE c_std_11)
target_include_directories(motion_estimator_tests PRIVATE ../MyFile/inc)
if(MSVC)
    target_compile_options(motion_estimator_tests PRIVATE /W4)
else()
    target_compile_options(motion_estimator_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(motion_estimator_tests PRIVATE m)
endif()
add_test(NAME motion_estimator_wrap_timing_and_reset COMMAND motion_estimator_tests)

add_executable(smart_knob_regression_tests
    smart_knob_regression_tests.c
    ../MyFile/src/smart_knob.c
    ../MyFile/src/smart_knob_modes.c
    ../MyFile/src/frictionless_hold.c
)

target_compile_features(smart_knob_regression_tests PRIVATE c_std_11)
target_compile_definitions(smart_knob_regression_tests PRIVATE ROLLERCAN_MOTOR_DIAGNOSTICS=1)
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
add_test(NAME coarse_damping_follows_fast_motion
         COMMAND smart_knob_regression_tests coarse-damping-motion)
add_test(NAME coarse_flick_release_settles
         COMMAND smart_knob_regression_tests coarse-flick-release)
add_test(NAME coarse_high_speed_limits_and_recovery
         COMMAND smart_knob_regression_tests coarse-high-speed)
add_test(NAME coarse_fast_flick_release_settles
         COMMAND smart_knob_regression_tests coarse-fast-flick-release)
add_test(NAME fine_high_speed_tracks_detent_center
         COMMAND smart_knob_regression_tests fine-high-speed)
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
add_test(NAME frictionless_motion_stop_and_reversal
         COMMAND smart_knob_regression_tests frictionless-motion)
add_test(NAME frictionless_current_limits_faults_and_recovery
         COMMAND smart_knob_regression_tests frictionless-limits)

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
    ../MyFile/src/frictionless_hold.c
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
add_test(NAME v1_migration_and_frictionless_parameter_roundtrip
         COMMAND host_compatibility_tests v1-frictionless)
add_test(NAME legacy_migration_and_selective_edit
         COMMAND host_compatibility_tests legacy-selective)
add_test(NAME firmware_presets_match_host_table
         COMMAND host_compatibility_tests preset-table)
add_test(NAME frictionless_canonical_can_and_persistence_parameters
         COMMAND host_compatibility_tests frictionless-canonical)

add_test(NAME frictionless_unlimited_speed_and_recovery
         COMMAND smart_knob_regression_tests frictionless-unlimited-speed)
add_test(NAME frictionless_observer_never_writes_actuator
         COMMAND smart_knob_regression_tests frictionless-observer)
add_test(NAME frictionless_observed_motion_handover_is_continuous
         COMMAND smart_knob_regression_tests frictionless-handover)
add_test(NAME frictionless_spatial_ripple_period_and_direction
         COMMAND smart_knob_regression_tests frictionless-spatial)
add_test(NAME frictionless_ripple_envelope_limits_and_stop
         COMMAND smart_knob_regression_tests frictionless-ripple-limits)
add_test(NAME frictionless_inertia_acceleration_and_constant_speed
         COMMAND smart_knob_regression_tests frictionless-inertia-motion)
add_test(NAME frictionless_inertia_stop_fault_and_recovery
         COMMAND smart_knob_regression_tests frictionless-inertia-reset)
add_test(NAME frictionless_inertia_parameter_and_current_limits
         COMMAND smart_knob_regression_tests frictionless-inertia-limits)
add_test(NAME frictionless_inertia_observer_continuity
         COMMAND smart_knob_regression_tests frictionless-inertia-observer)

add_test(NAME frictionless_hold_rejects_model_error
         COMMAND smart_knob_regression_tests frictionless-hold-plant)
add_test(NAME frictionless_hold_captures_release_after_real_launch
         COMMAND smart_knob_regression_tests frictionless-hold-launch)
add_test(NAME frictionless_hold_rejects_temperature_drift_and_residual_load
         COMMAND smart_knob_regression_tests frictionless-hold-drift)
add_test(NAME frictionless_hold_stop_reversal_fault_and_mode_reset
         COMMAND smart_knob_regression_tests frictionless-hold-reset)
add_test(NAME frictionless_hold_gesture_releases_old_speed
         COMMAND smart_knob_regression_tests frictionless-hold-gesture)
add_test(NAME frictionless_hold_observer_releases_at_current_speed
         COMMAND smart_knob_regression_tests frictionless-hold-handover)
add_test(NAME frictionless_hold_gentle_start_and_slow_gestures
         COMMAND smart_knob_regression_tests frictionless-hold-slow-gesture)
add_test(NAME frictionless_hold_extreme_saved_tuning_and_small_current_budget
         COMMAND smart_knob_regression_tests frictionless-hold-current-budget)

add_executable(frictionless_hold_tests
    frictionless_hold_tests.c
    ../MyFile/src/frictionless_hold.c
)
target_compile_features(frictionless_hold_tests PRIVATE c_std_11)
target_include_directories(frictionless_hold_tests PRIVATE ../MyFile/inc)
if(MSVC)
    target_compile_options(frictionless_hold_tests PRIVATE /W4)
else()
    target_compile_options(frictionless_hold_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(frictionless_hold_tests PRIVATE m)
endif()
add_test(NAME frictionless_hold_controller COMMAND frictionless_hold_tests)
