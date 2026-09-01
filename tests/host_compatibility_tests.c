#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "host_control.h"
#include "mysys.h"
#include "smart_knob.h"
#include "smart_knob_modes.h"
#include "smart_knob_persistence.h"

uint8_t motor_mode = MODE_DIAL;
uint8_t error_code;
uint8_t over_vol_flag;
float mechanical_rad;
float motor_rps;
float ph_crrent_lpf;

static uint32_t fake_time_us;
static float commanded_current_ma;

uint32_t HAL_GetTick(void)
{
    return fake_time_us / 1000U;
}

uint32_t micros(void)
{
    return fake_time_us;
}

void MotorDriverSetCurrentReal(float phase_current_ma)
{
    commanded_current_ma = phase_current_ma;
}

void MotorDriverSetCurrentRealContinuous(float phase_current_ma)
{
    commanded_current_ma = phase_current_ma;
}

uint8_t MotorDriverIsOutputEnabled(void)
{
    return 1U;
}

static void fail(const char *test_name, const char *message)
{
    fprintf(stderr, "FAIL %s: %s\n", test_name, message);
    exit(EXIT_FAILURE);
}

static void require_true(const char *test_name, bool value, const char *message)
{
    if (!value) {
        fail(test_name, message);
    }
}

static void put_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

static void put_i32(uint8_t *bytes, int32_t value)
{
    const uint32_t raw = (uint32_t)value;
    bytes[0] = (uint8_t)raw;
    bytes[1] = (uint8_t)(raw >> 8);
    bytes[2] = (uint8_t)(raw >> 16);
    bytes[3] = (uint8_t)(raw >> 24);
}

static CanProtocolCommand ping_command(void)
{
    CanProtocolCommand command = {0};
    command.command_id = 0U;
    return command;
}

static CanProtocolCommand read_command(uint16_t function)
{
    CanProtocolCommand command = {0};
    command.command_id = 17U;
    put_u16(command.data, function);
    return command;
}

static CanProtocolCommand write_command(uint16_t function, int32_t value)
{
    CanProtocolCommand command = {0};
    command.command_id = 18U;
    put_u16(command.data, function);
    put_i32(&command.data[4], value);
    return command;
}

static uint32_t process_command(HostControlState *state,
                                const CanProtocolCommand *command,
                                uint32_t now_ms,
                                uint8_t output)
{
    uint32_t events = host_control_before_command(
        state, command, now_ms, output);
    events |= host_control_after_command(state, command, now_ms, true);
    return events;
}

static void test_read_only_discovery_and_timeout(void)
{
    static const char name[] = "read_only_discovery_and_timeout";
    HostControlState state;
    host_control_initialize(&state);

    CanProtocolCommand command = ping_command();
    require_true(name, process_command(&state, &command, 100U, 1U) == 0U,
                 "ping must remain read-only");
    command = read_command(SMART_KNOB_FUNC_MODE_COUNT);
    require_true(name, process_command(&state, &command, 200U, 1U) == 0U,
                 "parameter discovery must remain read-only");
    require_true(name, state.connected == 0U,
                 "discovery must not acquire OLED control");
    command = (CanProtocolCommand){0};
    command.command_id = 20U;
    require_true(name, process_command(&state, &command, 250U, 1U) == 0U &&
                           state.connected == 0U,
                 "I2C bridge traffic must not acquire local control");

    command = write_command(SMART_KNOB_FUNC_TELEMETRY_ENABLE, 1);
    const uint32_t events = process_command(&state, &command, 300U, 1U);
    require_true(name, (events & HOST_CONTROL_EVENT_ACQUIRED) != 0U,
                 "first supported write must acquire control");
    require_true(name, state.connected != 0U && state.restore_output == 1U,
                 "takeover must remember the local run state");

    command = ping_command();
    (void)process_command(&state, &command, 1000U, 1U);
    require_true(name, host_control_poll(&state, 3999U) == 0U,
                 "timeout must not fire before three seconds");
    require_true(name,
                 (host_control_poll(&state, 4000U) &
                  HOST_CONTROL_EVENT_TIMED_OUT) != 0U,
                 "timeout must fire at exactly three seconds");

    host_control_initialize(&state);
    command = write_command(SMART_KNOB_FUNC_TELEMETRY_ENABLE, 1);
    (void)process_command(&state, &command, 5000U, 0U);
    require_true(name, state.restore_output == 0U,
                 "takeover must also preserve a paused local state");
}

static void test_configuration_transaction_and_stop(void)
{
    static const char name[] = "configuration_transaction_and_stop";
    HostControlState state;
    host_control_initialize(&state);
    uint32_t now = 10U;

    CanProtocolCommand command = write_command(0x7004U, 0);
    uint32_t events = process_command(&state, &command, now++, 1U);
    require_true(name, (events & HOST_CONTROL_EVENT_ACQUIRED) != 0U,
                 "disable must acquire control");
    command = write_command(0x7006U, 0);
    (void)process_command(&state, &command, now++, 0U);
    command = write_command(SMART_KNOB_FUNC_MODE,
                            SMART_KNOB_MODE_COARSE_STRONG);
    events = host_control_before_command(&state, &command, now, 0U);
    require_true(name, (events & HOST_CONTROL_EVENT_CONFIG_BEGIN) != 0U,
                 "disable/current/mode must begin a configuration transaction");
    events |= host_control_after_command(&state, &command, now++, true);
    require_true(name, (events & HOST_CONTROL_EVENT_SNAPSHOT_CHANGED) != 0U,
                 "mode selection must be staged");
    command = write_command(SMART_KNOB_FUNC_P_GAIN, 28000);
    events = process_command(&state, &command, now++, 0U);
    require_true(name, (events & HOST_CONTROL_EVENT_SNAPSHOT_CHANGED) != 0U,
                 "tuning must remain part of the staged transaction");
    command = write_command(0x7005U, MODE_DIAL);
    (void)process_command(&state, &command, now++, 0U);
    command = write_command(0x7004U, 1);
    events = process_command(&state, &command, now++, 0U);
    require_true(name, (events & HOST_CONTROL_EVENT_CONFIG_COMMITTED) != 0U,
                 "only successful Dial enable may commit the transaction");
    require_true(name, state.has_committed_config != 0U,
                 "committed snapshot must remain available for disconnect");

    command = write_command(0x7006U, 0);
    (void)process_command(&state, &command, now++, 1U);
    command = write_command(0x7004U, 0);
    events = process_command(&state, &command, now++, 1U);
    require_true(name, (events & HOST_CONTROL_EVENT_STOPPED) != 0U,
                 "current-zero followed by disable must be recognized as Stop");
    require_true(name, state.connected != 0U,
                 "Stop must keep OLED ownership until traffic times out");

    command = write_command(0x7006U, 0);
    (void)process_command(&state, &command, now++, 0U);
    command = write_command(0x7004U, 0);
    events = process_command(&state, &command, now++, 0U);
    require_true(name, (events & HOST_CONTROL_EVENT_STOPPED) != 0U,
                 "repeating Stop must request another durable-save attempt");
}

static void test_partial_transaction_is_not_committed(void)
{
    static const char name[] = "partial_transaction_is_not_committed";
    HostControlState state;
    host_control_initialize(&state);
    CanProtocolCommand command = write_command(0x7004U, 0);
    (void)process_command(&state, &command, 0U, 0U);
    command = write_command(0x7006U, 0);
    (void)process_command(&state, &command, 1U, 0U);
    command = write_command(SMART_KNOB_FUNC_MODE, SMART_KNOB_MODE_FINE_DETENTS);
    (void)process_command(&state, &command, 2U, 0U);
    command = write_command(SMART_KNOB_FUNC_CLICK_CURRENT, 250);
    (void)process_command(&state, &command, 3U, 0U);
    require_true(name, state.has_committed_config == 0U,
                 "half-written config must not become durable");

    command = write_command(0x7004U, 0);
    (void)process_command(&state, &command, 4U, 0U);
    command = write_command(0x7006U, 0);
    (void)process_command(&state, &command, 5U, 0U);
    command = write_command(SMART_KNOB_FUNC_MODE,
                            SMART_KNOB_MODE_COARSE_WEAK);
    uint32_t events = host_control_before_command(&state, &command, 6U, 0U);
    require_true(name, (events & HOST_CONTROL_EVENT_CONFIG_BEGIN) != 0U,
                 "a retry must request a clean full-snapshot baseline");
    events |= host_control_after_command(&state, &command, 6U, true);
    require_true(name, state.has_committed_config == 0U,
                 "retry setup must not make either partial transaction durable");
    require_true(name,
                 (host_control_poll(&state, 3006U) &
                  HOST_CONTROL_EVENT_TIMED_OUT) != 0U,
                 "partial transaction must still release on timeout");
}

static void test_snapshot_roundtrip_and_crc(void)
{
    static const char name[] = "snapshot_roundtrip_and_crc";
    smart_knob_modes_reset_defaults();
    require_true(name,
                 smart_knob_select_mode(SMART_KNOB_MODE_COARSE_WEAK),
                 "test mode must be selectable");
    require_true(name,
                 smart_knob_write_parameter(SMART_KNOB_FUNC_P_GAIN, 12345, 0x42),
                 "tuning write must succeed");
    require_true(name,
                 smart_knob_write_parameter(SMART_KNOB_FUNC_CUSTOM_LED_HUE,
                                            231, 0x42),
                 "custom write must succeed");

    SmartKnobPersistentSnapshot source;
    SmartKnobPersistentSnapshot decoded;
    require_true(name, smart_knob_snapshot_export(&source),
                 "snapshot export must succeed");
    uint8_t bytes[SMART_KNOB_FLASH_TOTAL_SIZE];
    memset(bytes, 0xA5, sizeof(bytes));
    require_true(name,
                 smart_knob_persistence_encode(bytes, sizeof(bytes), &source),
                 "512-byte encoding must succeed");
    for (size_t index = 0U; index < SMART_KNOB_FLASH_LEGACY_SIZE; ++index) {
        require_true(name, bytes[index] == 0xA5,
                     "encoder must preserve the legacy 48-byte prefix");
    }
    require_true(name,
                 smart_knob_persistence_decode(bytes, sizeof(bytes), &decoded),
                 "encoded snapshot must decode");
    require_true(name, memcmp(&source, &decoded, sizeof(source)) == 0,
                 "snapshot must round-trip byte-for-byte");
    require_true(name,
                 !smart_knob_persistence_decode(bytes,
                                                sizeof(bytes) - 8U,
                                                &decoded),
                 "a truncated extension must not be accepted");

    bytes[200] ^= 0x01U;
    require_true(name,
                 !smart_knob_persistence_decode(bytes, sizeof(bytes), &decoded),
                 "CRC damage must reject the extension");
    require_true(name,
                 !smart_knob_persistence_decode(bytes,
                                                SMART_KNOB_FLASH_LEGACY_SIZE,
                                                &decoded),
                 "an old 48-byte page must be detected as legacy");
}

static void test_legacy_migration_and_selective_local_edit(void)
{
    static const char name[] = "legacy_migration_and_selective_local_edit";
    smart_knob_modes_reset_defaults();
    require_true(name,
                 smart_knob_mode_apply_local_profile(
                     SMART_KNOB_MODE_COARSE_STRONG, 75U, 300U, 12U),
                 "legacy four-field profile must migrate over defaults");
    require_true(name,
                 smart_knob_select_mode(SMART_KNOB_MODE_COARSE_STRONG),
                 "migrated mode must be selected");
    SmartKnobPersistentSnapshot migrated;
    require_true(name, smart_knob_snapshot_export(&migrated),
                 "migrated defaults must expand to a full snapshot");
    require_true(name,
                 migrated.mode_values[SMART_KNOB_MODE_COARSE_STRONG][0] == 12000 &&
                 migrated.mode_values[SMART_KNOB_MODE_COARSE_STRONG][3] == 150 &&
                 migrated.mode_values[SMART_KNOB_MODE_COARSE_STRONG][4] == 300,
                 "legacy width/force/current must survive migration");

    require_true(name,
                 smart_knob_write_parameter(SMART_KNOB_FUNC_P_GAIN, 31000, 0x42),
                 "advanced tuning write must succeed");
    require_true(name,
                 smart_knob_write_parameter(SMART_KNOB_FUNC_CURRENT_SCALE,
                                            213, 0x42),
                 "non-local force value must succeed");
    LocalProfileEdit edit;
    uint8_t mask = 0U;
    require_true(name,
                 smart_knob_mode_local_projection(
                     SMART_KNOB_MODE_COARSE_STRONG, &edit, &mask),
                 "OLED projection must be available");
    require_true(name,
                 (mask & SMART_KNOB_LOCAL_UNREPRESENTABLE_FORCE) != 0U &&
                 (mask & SMART_KNOB_LOCAL_UNREPRESENTABLE_ADVANCED) != 0U,
                 "non-local force and hidden P must produce a star");

    require_true(name,
                 smart_knob_write_parameter(SMART_KNOB_FUNC_CURRENT_SCALE,
                                            200, 0x42),
                 "exact local force value must succeed");
    require_true(name,
                 smart_knob_mode_local_projection(
                     SMART_KNOB_MODE_COARSE_STRONG, &edit, &mask),
                 "exact OLED projection must be available");
    require_true(name,
                 (mask & SMART_KNOB_LOCAL_UNREPRESENTABLE_FORCE) == 0U,
                 "an exact local force value must not produce a force star");

    require_true(name,
                 smart_knob_select_mode(SMART_KNOB_MODE_UNBOUNDED_SMOOTH) &&
                 smart_knob_write_parameter(SMART_KNOB_FUNC_CURRENT_SCALE,
                                            38, 0x42),
                 "protocol-quantized fractional force must be writable");
    require_true(name,
                 smart_knob_mode_local_projection(
                     SMART_KNOB_MODE_UNBOUNDED_SMOOTH, &edit, &mask),
                 "fractional force projection must be available");
    require_true(name,
                 (mask & SMART_KNOB_LOCAL_UNREPRESENTABLE_FORCE) != 0U,
                 "0.038 A cannot be represented exactly as a 5% step of 0.0375 A");

    require_true(name,
                 smart_knob_select_mode(SMART_KNOB_MODE_COARSE_STRONG) &&
                 smart_knob_write_parameter(SMART_KNOB_FUNC_CURRENT_SCALE,
                                            213, 0x42),
                 "restore the non-local force before selective editing");
    require_true(name,
                 smart_knob_mode_local_projection(
                     SMART_KNOB_MODE_COARSE_STRONG, &edit, &mask),
                 "restore the coarse OLED projection before editing");

    edit.step_width_deg = 14U;
    edit.dirty_mask = LOCAL_PROFILE_DIRTY_WIDTH;
    require_true(name, smart_knob_mode_apply_local_edit(&edit),
                 "single-field local edit must succeed");
    const SmartKnobModeConfig *mode =
        smart_knob_mode_get(SMART_KNOB_MODE_COARSE_STRONG);
    require_true(name,
                 lroundf(mode->tuning.p_gain * 1000.0f) == 31000 &&
                 lroundf(mode->tuning.current_scale_a * 1000.0f) == 213,
                 "editing width must preserve hidden P and host force");
    require_true(name,
                 lroundf(mode->config.position_width_radians * 180000.0f / PI) ==
                     14000,
                 "only the dirty width field may change");
}

typedef struct {
    int32_t position;
    int32_t minimum;
    int32_t maximum;
    int32_t width;
    int32_t detent;
    int32_t endstop;
    int32_t snap;
    int32_t bias;
    int32_t hue;
    int32_t p;
    int32_t d;
    int32_t scale;
    int32_t friction;
    int32_t click;
} FrozenPreset;

static int32_t scaled(float value)
{
    return (int32_t)lroundf(value * 1000.0f);
}

static void test_frozen_host_preset_table(void)
{
    static const char name[] = "frozen_host_preset_table";
    static const FrozenPreset expected[SMART_KNOB_MODE_COUNT] = {
        {0, 0, -1, 10000, 0, 1000, 550, 0, 120, 0, 0, 88, 0, 0},
        {0, 0, -1, 10000, 0, 1000, 750, 0, 200, 0, 0, 38, 20, 0},
        {0, 0, 10, 10000, 0, 1000, 1100, 0, 0, 0, 0, 63, 0, 0},
        {0, 0, 72, 10000, 0, 5000, 750, 0, 73, 0, 0, 38, 0, 0},
        {0, 0, 1, 60000, 10000, 1000, 550, 0, 157, 38000, 550, 100, 0, 0},
        {0, 0, 0, 60000, 10, 600, 1100, 0, 45, 40000, 100, 200, 8, 0},
        {127, 0, 255, 1000, 0, 1000, 1100, 0, 219, 0, 100, 75, 0, 0},
        {127, 0, 255, 1000, 1000, 1000, 900, 0, 25, 0, 100, 63, 0, 100},
        {0, 0, 31, 10000, 8000, 1000, 750, 0, 200, 28000, 160, 200, 0, 0},
        {0, 0, 31, 10000, 200, 1000, 900, 0, 0, 5000, 160, 200, 0, 350},
        {0, 0, 31, 7000, 2500, 1000, 700, 0, 73, 40000, 200, 200, 0, 0},
        {0, -6, 6, 60000, 1000, 1000, 550, 400, 157, 10000, 100, 200, 0, 0},
    };

    smart_knob_modes_reset_defaults();
    for (uint8_t index = 0U; index < SMART_KNOB_MODE_COUNT; ++index) {
        const SmartKnobModeConfig *mode = smart_knob_mode_get_default(index);
        const FrozenPreset *want = &expected[index];
        require_true(name, mode != NULL, "all twelve presets must exist");
        if (mode->config.position != want->position ||
            mode->config.min_position != want->minimum ||
            mode->config.max_position != want->maximum ||
            scaled(mode->config.position_width_radians * 180.0f / PI) !=
                want->width ||
            scaled(mode->config.detent_strength_unit) != want->detent ||
            scaled(mode->config.endstop_strength_unit) != want->endstop ||
            scaled(mode->config.snap_point) != want->snap ||
            scaled(mode->config.snap_point_bias) != want->bias ||
            mode->config.led_hue != want->hue ||
            scaled(mode->tuning.p_gain) != want->p ||
            scaled(mode->tuning.d_gain) != want->d ||
            scaled(mode->tuning.current_scale_a) != want->scale ||
            scaled(mode->tuning.friction_current_a) != want->friction ||
            scaled(mode->tuning.click_current_a) != want->click ||
            scaled(mode->tuning.current_limit_a) != 450 ||
            mode->tuning.max_current_permille != 700U) {
            fprintf(stderr, "FAIL %s: preset %u differs from rollercan.rs\n",
                    name, index);
            exit(EXIT_FAILURE);
        }
    }
    const SmartKnobModeConfig *magnetic =
        smart_knob_mode_get_default(SMART_KNOB_MODE_MAGNETIC);
    require_true(name,
                 magnetic->config.detent_positions_count == 4U &&
                 magnetic->config.detent_positions[0] == 2 &&
                 magnetic->config.detent_positions[1] == 10 &&
                 magnetic->config.detent_positions[2] == 21 &&
                 magnetic->config.detent_positions[3] == 22,
                 "magnetic detent positions must match the host table");
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fputs("usage: host_compatibility_tests <case>\n", stderr);
        return EXIT_FAILURE;
    }
    if (strcmp(argv[1], "authority-timeout") == 0) {
        test_read_only_discovery_and_timeout();
    } else if (strcmp(argv[1], "transaction-stop") == 0) {
        test_configuration_transaction_and_stop();
    } else if (strcmp(argv[1], "partial-transaction") == 0) {
        test_partial_transaction_is_not_committed();
    } else if (strcmp(argv[1], "snapshot-crc") == 0) {
        test_snapshot_roundtrip_and_crc();
    } else if (strcmp(argv[1], "legacy-selective") == 0) {
        test_legacy_migration_and_selective_local_edit();
    } else if (strcmp(argv[1], "preset-table") == 0) {
        test_frozen_host_preset_table();
    } else {
        fprintf(stderr, "unknown test case: %s\n", argv[1]);
        return EXIT_FAILURE;
    }
    printf("PASS %s\n", argv[1]);
    return EXIT_SUCCESS;
}
