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
float motor_rps_fast;
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

void MotorDriverSetTorqueCurrent(float phase_current_ma)
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

static void put_test_u32(uint8_t *bytes, uint32_t value)
{
    for (unsigned i = 0; i < 4U; ++i) bytes[i] = (uint8_t)(value >> (8U * i));
}

static void test_v1_snapshot_migration_and_frictionless_parameters(void)
{
    const char *name = "v1_snapshot_migration_and_frictionless_parameters";
    SmartKnobPersistentSnapshot source, decoded, defaults, live_after;
    smart_knob_modes_reset_defaults();
    (void)smart_knob_select_mode(SMART_KNOB_MODE_COARSE_WEAK);
    (void)smart_knob_write_parameter(SMART_KNOB_FUNC_P_GAIN, 12345, 0x42);
    (void)smart_knob_write_parameter(SMART_KNOB_FUNC_CUSTOM_LED_HUE, 231, 0x42);
    (void)smart_knob_snapshot_export(&source);
    (void)smart_knob_snapshot_defaults(&defaults);

    /* Construct the previous on-flash format independently: twelve 8-int
       records, then 14 custom values. Its padding is not part of the CRC. */
    uint8_t old[512] = {0};
    memcpy(&old[48], "SKN2", 4U);
    old[52] = 1U; old[53] = 12U; old[54] = source.selected_mode;
    old[56] = 0xB8U; old[57] = 1U;
    size_t offset = 64U;
    for (unsigned mode = 0; mode < 12U; ++mode) {
        for (unsigned field = 0; field < 8U; ++field) {
            put_test_u32(&old[offset], (uint32_t)source.mode_values[mode][field]);
            offset += 4U;
        }
    }
    for (unsigned field = 0; field < 14U; ++field) {
        put_test_u32(&old[offset], (uint32_t)source.custom_values[field]);
        offset += 4U;
    }
    uint8_t crc_input[452];
    memcpy(crc_input, &old[48], 12U);
    memcpy(&crc_input[12], &old[64], 440U);
    put_test_u32(&old[60], smart_knob_persistence_crc32(crc_input, sizeof(crc_input)));
    require_true(name, smart_knob_persistence_decode(old, sizeof(old), &decoded),
        "the existing 512-byte v1 page must migrate");
    for (unsigned mode = 0; mode < 12U; ++mode) {
        require_true(name, memcmp(source.mode_values[mode], decoded.mode_values[mode], 32U) == 0,
            "all old tuning values must survive migration exactly");
        require_true(name, decoded.mode_values[mode][8] == 0,
            "legacy modes must receive a zero viscous coefficient");
    }
    require_true(name, memcmp(source.custom_values, decoded.custom_values, sizeof(source.custom_values)) == 0,
        "custom settings must survive migration exactly");
    require_true(name, memcmp(defaults.mode_values[12], decoded.mode_values[12], sizeof(defaults.mode_values[12])) == 0,
        "the new preset must receive its compiled defaults");
    (void)smart_knob_snapshot_export(&live_after);
    require_true(name, memcmp(&source, &live_after, sizeof(source)) == 0,
        "decoding must not change live parameters or mode selection");
    old[100] ^= 1U;
    require_true(name, !smart_knob_persistence_decode(old, sizeof(old), &decoded),
        "damaged old-format CRC must be rejected");

    (void)smart_knob_select_mode(SMART_KNOB_MODE_FRICTIONLESS);
    require_true(name, smart_knob_write_parameter(SMART_KNOB_FUNC_VISCOUS_FRICTION, 275, 0x42),
        "new CAN coefficient must be writable");
    int32_t coefficient;
    require_true(name, smart_knob_read_parameter(SMART_KNOB_FUNC_VISCOUS_FRICTION, &coefficient) && coefficient == 275,
        "viscous coefficient uses microamps per radian per second");
    CanProtocolCommand command = write_command(SMART_KNOB_FUNC_VISCOUS_FRICTION, 275);
    require_true(name, host_control_command_is_mutating(&command) && host_control_command_changes_snapshot(&command),
        "new parameter must participate in host ownership and persistence transactions");
    (void)smart_knob_snapshot_export(&source);
    uint8_t current[SMART_KNOB_FLASH_TOTAL_SIZE];
    memset(current, 0, sizeof(current));
    require_true(name, smart_knob_persistence_encode(current, sizeof(current), &source) &&
        smart_knob_persistence_decode(current, sizeof(current), &decoded) &&
        smart_knob_snapshot_import(&decoded), "new preset and coefficient must round-trip through flash");
    require_true(name, smart_knob_active_mode() == SMART_KNOB_MODE_FRICTIONLESS &&
        smart_knob_read_parameter(SMART_KNOB_FUNC_VISCOUS_FRICTION, &coefficient) && coefficient == 275,
        "restored mode and coefficient must match saved values");
    for (size_t length = 0; length < sizeof(current); ++length) {
        require_true(name, !smart_knob_persistence_decode(current, length, &decoded),
            "all truncated v2 packages must be rejected");
    }
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
    static const FrozenPreset expected[12] = {
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
    for (uint8_t index = 0U; index < 12U; ++index) {
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

static void test_frictionless_canonical_parameters(void)
{
    const char *name = "frictionless_canonical_parameters";
    smart_knob_modes_reset_defaults();
    require_true(name, smart_knob_select_mode(SMART_KNOB_MODE_FRICTIONLESS),
        "FR-L must be selectable for protocol checks");
    const struct { uint16_t parameter; int32_t requested; int32_t canonical; } cases[] = {
        {SMART_KNOB_FUNC_P_GAIN, -1000, 0},
        {SMART_KNOB_FUNC_P_GAIN, 80000, 8000},
        {SMART_KNOB_FUNC_D_GAIN, 1, 5},
        {SMART_KNOB_FUNC_D_GAIN, 1000, 200},
        {SMART_KNOB_FUNC_D_GAIN, -1, 0},
        {SMART_KNOB_FUNC_D_GAIN, 0, 0},
        {SMART_KNOB_FUNC_CURRENT_LIMIT, -1, 0},
        {SMART_KNOB_FUNC_CURRENT_LIMIT, 1200, 800},
    };
    SmartKnobPersistentSnapshot defaults, source, decoded;
    require_true(name, smart_knob_snapshot_defaults(&defaults), "default snapshot must be available");
    uint8_t bytes[SMART_KNOB_FLASH_TOTAL_SIZE] = {0};
    for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        int32_t value = INT32_MIN;
        require_true(name, smart_knob_write_parameter(cases[i].parameter, cases[i].requested, 0x42) &&
            smart_knob_read_parameter(cases[i].parameter, &value) && value == cases[i].canonical,
            "CAN writes and immediate reads must use the effective FR-L range");
        require_true(name, smart_knob_snapshot_export(&source) &&
            smart_knob_persistence_encode(bytes, sizeof(bytes), &source) &&
            smart_knob_persistence_decode(bytes, sizeof(bytes), &decoded) &&
            smart_knob_snapshot_import(&decoded) &&
            smart_knob_read_parameter(cases[i].parameter, &value) && value == cases[i].canonical,
            "canonical values, including D zero's default-filter meaning, must survive flash round trips");
        for (uint8_t mode = 0U; mode < SMART_KNOB_MODE_FRICTIONLESS; ++mode) {
            require_true(name, memcmp(source.mode_values[mode], defaults.mode_values[mode],
                                      sizeof(source.mode_values[mode])) == 0,
                "FR-L canonicalization must not change any other preset");
        }
    }

    /* Export and readback also defend against diagnostics or direct firmware
       edits that have not yet passed through a control iteration. */
    SmartKnobModeConfig *mode = smart_knob_mode_get_mutable(SMART_KNOB_MODE_FRICTIONLESS);
    mode->tuning.p_gain = 80.0f;
    mode->tuning.d_gain = 0.001f;
    mode->tuning.current_limit_a = 1.2f;
    int32_t value;
    require_true(name, smart_knob_read_parameter(SMART_KNOB_FUNC_P_GAIN, &value) && value == 8000 &&
        smart_knob_snapshot_export(&source) &&
        source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][1] == 8000 &&
        source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][2] == 5 &&
        source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][4] == 800,
        "readback and export must never advertise ineffective raw FR-L values");

    source.selected_mode = SMART_KNOB_MODE_COARSE_STRONG;
    source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][1] = 80000;
    source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][2] = 1;
    source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][4] = 1200;
    source.mode_values[SMART_KNOB_MODE_COARSE_STRONG][1] = 80000;
    source.mode_values[SMART_KNOB_MODE_COARSE_STRONG][2] = 1000;
    source.mode_values[SMART_KNOB_MODE_COARSE_STRONG][4] = 1200;
    require_true(name, smart_knob_snapshot_import(&source), "legacy-range snapshot must load");
    mode = smart_knob_mode_get_mutable(SMART_KNOB_MODE_FRICTIONLESS);
    require_true(name, mode->tuning.p_gain == 8.0f &&
        fabsf(mode->tuning.d_gain - 0.005f) < 1e-7f &&
        fabsf(mode->tuning.current_limit_a - 0.8f) < 1e-7f,
        "snapshot import must canonicalize FR-L even when a different mode is selected");
    require_true(name, smart_knob_read_parameter(SMART_KNOB_FUNC_P_GAIN, &value) && value == 80000 &&
        smart_knob_read_parameter(SMART_KNOB_FUNC_D_GAIN, &value) && value == 1000 &&
        smart_knob_read_parameter(SMART_KNOB_FUNC_CURRENT_LIMIT, &value) && value == 1200,
        "legacy modes must keep their original P, D and 1200 mA protocol ranges");
    require_true(name, smart_knob_write_parameter(SMART_KNOB_FUNC_P_GAIN, 12000, 0x42) &&
        smart_knob_read_parameter(SMART_KNOB_FUNC_P_GAIN, &value) && value == 12000 &&
        smart_knob_write_parameter(SMART_KNOB_FUNC_D_GAIN, 1, 0x42) &&
        smart_knob_read_parameter(SMART_KNOB_FUNC_D_GAIN, &value) && value == 1,
        "FR-L limits must not leak into live writes to another mode");

    require_true(name, smart_knob_select_mode(SMART_KNOB_MODE_FRICTIONLESS), "return to FR-L");
    mode->tuning.p_gain = INFINITY;
    mode->tuning.d_gain = NAN;
    mode->tuning.current_limit_a = INFINITY;
    require_true(name, smart_knob_snapshot_export(&source) &&
        source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][1] == 0 &&
        source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][2] == 0 &&
        source.mode_values[SMART_KNOB_MODE_FRICTIONLESS][4] == 0,
        "non-finite direct tuning must export finite inactive/default requests");
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
    } else if (strcmp(argv[1], "v1-frictionless") == 0) {
        test_v1_snapshot_migration_and_frictionless_parameters();
    } else if (strcmp(argv[1], "legacy-selective") == 0) {
        test_legacy_migration_and_selective_local_edit();
    } else if (strcmp(argv[1], "preset-table") == 0) {
        test_frozen_host_preset_table();
    } else if (strcmp(argv[1], "frictionless-canonical") == 0) {
        test_frictionless_canonical_parameters();
    } else {
        fprintf(stderr, "unknown test case: %s\n", argv[1]);
        return EXIT_FAILURE;
    }
    printf("PASS %s\n", argv[1]);
    return EXIT_SUCCESS;
}
