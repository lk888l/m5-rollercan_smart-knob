#include "host_control.h"

#include <stddef.h>

enum {
    CMD_PING = 0,
    CMD_ENABLE = 3,
    CMD_DISABLE = 4,
    CMD_SET_ID = 7,
    CMD_CLEAR_PROTECTION = 9,
    CMD_SAVE = 10,
    CMD_SET_BAUD = 11,
    CMD_STALL_ON = 12,
    CMD_STALL_OFF = 13,
    CMD_OVER_VALUE_ON = 14,
    CMD_OVER_VALUE_OFF = 15,
    CMD_FUNCTION_READ = 17,
    CMD_FUNCTION_WRITE = 18,
};

enum {
    FUNC_SAVE_FLASH = 0x7002,
    FUNC_REMOVE_PROTECTION = 0x7003,
    FUNC_ON_OFF = 0x7004,
    FUNC_RUN_MODE = 0x7005,
    FUNC_CURRENT_SETTING = 0x7006,
    FUNC_SPEED_SETTING = 0x700A,
    FUNC_POSITION_SETTING = 0x7016,
    FUNC_POSITION_MAX_CURRENT = 0x7017,
    FUNC_SPEED_MAX_CURRENT = 0x7018,
    FUNC_SPEED_KP = 0x7020,
    FUNC_SPEED_KI = 0x7021,
    FUNC_SPEED_KD = 0x7022,
    FUNC_POSITION_KP = 0x7023,
    FUNC_POSITION_KI = 0x7024,
    FUNC_POSITION_KD = 0x7025,
    FUNC_DIAL_COUNTER = 0x7033,
    FUNC_OVERVOLTAGE_RELEASE_MODE = 0x7040,
    FUNC_MAX_STALL_ATTEMPTS = 0x7041,
    FUNC_SPEED_STALL_THRESHOLD = 0x7042,
    FUNC_SPEED_STALL_TIMEOUT = 0x7043,
    FUNC_POSITION_STALL_THRESHOLD = 0x7044,
    FUNC_POSITION_STALL_TIMEOUT = 0x7045,
    FUNC_RGB_MODE = 0x7050,
    FUNC_RGB_COLOR = 0x7051,
    FUNC_RGB_BRIGHTNESS = 0x7052,
    FUNC_SMART_KNOB_MODE = 0x8001,
    FUNC_TELEMETRY_ENABLE = 0x8002,
    FUNC_TELEMETRY_HOST_ID = 0x8004,
    FUNC_TUNING_FIRST = 0x8101,
    FUNC_TUNING_LAST = 0x8107,
    FUNC_CUSTOM_FIRST = 0x8201,
    FUNC_CUSTOM_LAST = 0x820E,
    MODE_DIAL = 4,
};

static uint16_t command_function(const CanProtocolCommand *command)
{
    return (uint16_t)command->data[0] |
           ((uint16_t)command->data[1] << 8);
}

static int32_t command_value(const CanProtocolCommand *command)
{
    const uint32_t value = (uint32_t)command->data[4] |
                           ((uint32_t)command->data[5] << 8) |
                           ((uint32_t)command->data[6] << 16) |
                           ((uint32_t)command->data[7] << 24);
    return (int32_t)value;
}

static bool is_known_function_write(uint16_t function)
{
    if ((function >= FUNC_TUNING_FIRST && function <= FUNC_TUNING_LAST) ||
        (function >= FUNC_CUSTOM_FIRST && function <= FUNC_CUSTOM_LAST) ||
        (function >= FUNC_SAVE_FLASH && function <= FUNC_CURRENT_SETTING) ||
        function == FUNC_SPEED_SETTING ||
        (function >= FUNC_POSITION_SETTING && function <= FUNC_POSITION_KD) ||
        function == FUNC_DIAL_COUNTER ||
        (function >= FUNC_OVERVOLTAGE_RELEASE_MODE &&
         function <= FUNC_POSITION_STALL_TIMEOUT) ||
        (function >= FUNC_RGB_MODE && function <= FUNC_RGB_BRIGHTNESS) ||
        (function >= FUNC_SMART_KNOB_MODE &&
         function <= FUNC_TELEMETRY_HOST_ID)) {
        return true;
    }
    return false;
}

static bool is_supported_command(const CanProtocolCommand *command)
{
    if (command == NULL) {
        return false;
    }
    switch (command->command_id) {
    case CMD_PING:
    case CMD_ENABLE:
    case CMD_DISABLE:
    case CMD_SET_ID:
    case CMD_CLEAR_PROTECTION:
    case CMD_SAVE:
    case CMD_SET_BAUD:
    case CMD_STALL_ON:
    case CMD_STALL_OFF:
    case CMD_OVER_VALUE_ON:
    case CMD_OVER_VALUE_OFF:
    case CMD_FUNCTION_READ:
        return true;
    case CMD_FUNCTION_WRITE:
        return is_known_function_write(command_function(command));
    default:
        return false;
    }
}

bool host_control_command_is_mutating(const CanProtocolCommand *command)
{
    return is_supported_command(command) && command->command_id != CMD_PING &&
           command->command_id != CMD_FUNCTION_READ;
}

bool host_control_command_changes_snapshot(const CanProtocolCommand *command)
{
    if (command == NULL || command->command_id != CMD_FUNCTION_WRITE) {
        return false;
    }
    const uint16_t function = command_function(command);
    return function == FUNC_SMART_KNOB_MODE ||
           (function >= FUNC_TUNING_FIRST && function <= FUNC_TUNING_LAST) ||
           (function >= FUNC_CUSTOM_FIRST && function <= FUNC_CUSTOM_LAST);
}

void host_control_initialize(HostControlState *state)
{
    if (state == NULL) {
        return;
    }
    state->connected = 0U;
    state->restore_output = 0U;
    state->has_committed_config = 0U;
    state->pending_config_begin = 0U;
    state->stage = HOST_CONTROL_STAGE_IDLE;
    state->last_valid_ms = 0U;
}

uint32_t host_control_before_command(HostControlState *state,
                                     const CanProtocolCommand *command,
                                     uint32_t now_ms,
                                     uint8_t current_output)
{
    if (state == NULL || !is_supported_command(command) ||
        !host_control_command_is_mutating(command)) {
        return HOST_CONTROL_EVENT_NONE;
    }

    uint32_t events = HOST_CONTROL_EVENT_NONE;
    if (state->connected == 0U) {
        state->connected = 1U;
        state->restore_output = current_output != 0U;
        state->last_valid_ms = now_ms;
        state->stage = HOST_CONTROL_STAGE_IDLE;
        state->has_committed_config = 0U;
        events |= HOST_CONTROL_EVENT_ACQUIRED;
    }

    if (command->command_id == CMD_FUNCTION_WRITE &&
        command_function(command) == FUNC_SMART_KNOB_MODE &&
        state->stage == HOST_CONTROL_STAGE_CURRENT_ZERO) {
        state->pending_config_begin = 1U;
        state->stage = HOST_CONTROL_STAGE_CONFIGURING;
        events |= HOST_CONTROL_EVENT_CONFIG_BEGIN;
    }
    return events;
}

uint32_t host_control_after_command(HostControlState *state,
                                    const CanProtocolCommand *command,
                                    uint32_t now_ms,
                                    bool command_valid)
{
    if (state == NULL || !command_valid || !is_supported_command(command)) {
        return HOST_CONTROL_EVENT_NONE;
    }
    if (state->connected == 0U) {
        return HOST_CONTROL_EVENT_NONE;
    }

    state->last_valid_ms = now_ms;
    uint32_t events = HOST_CONTROL_EVENT_NONE;
    if (command->command_id != CMD_FUNCTION_WRITE) {
        if (command->command_id == CMD_DISABLE) {
            state->stage = HOST_CONTROL_STAGE_DISABLED;
        }
        return events;
    }

    const uint16_t function = command_function(command);
    const int32_t value = command_value(command);
    if (function == FUNC_ON_OFF) {
        if (value == 0) {
            if (state->stage == HOST_CONTROL_STAGE_STOP_ARMED &&
                state->has_committed_config != 0U) {
                state->stage = HOST_CONTROL_STAGE_STOPPED;
                events |= HOST_CONTROL_EVENT_STOPPED;
            } else {
                state->stage = HOST_CONTROL_STAGE_DISABLED;
            }
        } else if (state->stage == HOST_CONTROL_STAGE_DIAL_SELECTED) {
            state->stage = HOST_CONTROL_STAGE_ENABLED;
            state->has_committed_config = 1U;
            events |= HOST_CONTROL_EVENT_CONFIG_COMMITTED;
        }
    } else if (function == FUNC_CURRENT_SETTING && value == 0) {
        if (state->stage == HOST_CONTROL_STAGE_DISABLED) {
            state->stage = HOST_CONTROL_STAGE_CURRENT_ZERO;
        } else if (state->stage == HOST_CONTROL_STAGE_ENABLED ||
                   state->stage == HOST_CONTROL_STAGE_STOPPED) {
            state->stage = HOST_CONTROL_STAGE_STOP_ARMED;
        }
    } else if (function == FUNC_SMART_KNOB_MODE) {
        state->pending_config_begin = 0U;
        if (state->stage == HOST_CONTROL_STAGE_CONFIGURING ||
            state->stage == HOST_CONTROL_STAGE_ENABLED) {
            events |= HOST_CONTROL_EVENT_SNAPSHOT_CHANGED;
        }
    } else if (function == FUNC_RUN_MODE && value == MODE_DIAL &&
               state->stage == HOST_CONTROL_STAGE_CONFIGURING) {
        state->stage = HOST_CONTROL_STAGE_DIAL_SELECTED;
    } else if (host_control_command_changes_snapshot(command) &&
               (state->stage == HOST_CONTROL_STAGE_CONFIGURING ||
                state->stage == HOST_CONTROL_STAGE_DIAL_SELECTED ||
                state->stage == HOST_CONTROL_STAGE_ENABLED)) {
        events |= HOST_CONTROL_EVENT_SNAPSHOT_CHANGED;
    } else if (function == FUNC_SAVE_FLASH && value != 0) {
        events |= HOST_CONTROL_EVENT_SAVE_REQUESTED;
    }
    return events;
}

uint32_t host_control_poll(HostControlState *state, uint32_t now_ms)
{
    if (state == NULL || state->connected == 0U ||
        (uint32_t)(now_ms - state->last_valid_ms) < HOST_CONTROL_TIMEOUT_MS) {
        return HOST_CONTROL_EVENT_NONE;
    }
    state->connected = 0U;
    return HOST_CONTROL_EVENT_TIMED_OUT;
}
