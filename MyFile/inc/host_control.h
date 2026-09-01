#ifndef HOST_CONTROL_H
#define HOST_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HOST_CONTROL_TIMEOUT_MS 3000U

typedef enum {
    HOST_CONTROL_STAGE_IDLE = 0,
    HOST_CONTROL_STAGE_DISABLED,
    HOST_CONTROL_STAGE_CURRENT_ZERO,
    HOST_CONTROL_STAGE_CONFIGURING,
    HOST_CONTROL_STAGE_DIAL_SELECTED,
    HOST_CONTROL_STAGE_ENABLED,
    HOST_CONTROL_STAGE_STOP_ARMED,
    HOST_CONTROL_STAGE_STOPPED,
} HostControlStage;

enum {
    HOST_CONTROL_EVENT_NONE = 0U,
    HOST_CONTROL_EVENT_ACQUIRED = 1U << 0,
    HOST_CONTROL_EVENT_CONFIG_BEGIN = 1U << 1,
    HOST_CONTROL_EVENT_SNAPSHOT_CHANGED = 1U << 2,
    HOST_CONTROL_EVENT_CONFIG_COMMITTED = 1U << 3,
    HOST_CONTROL_EVENT_STOPPED = 1U << 4,
    HOST_CONTROL_EVENT_SAVE_REQUESTED = 1U << 5,
    HOST_CONTROL_EVENT_TIMED_OUT = 1U << 6,
};

typedef struct {
    uint8_t connected;
    uint8_t restore_output;
    uint8_t has_committed_config;
    uint8_t pending_config_begin;
    HostControlStage stage;
    uint32_t last_valid_ms;
} HostControlState;

void host_control_initialize(HostControlState *state);
uint32_t host_control_before_command(HostControlState *state,
                                     const CanProtocolCommand *command,
                                     uint32_t now_ms,
                                     uint8_t current_output);
uint32_t host_control_after_command(HostControlState *state,
                                    const CanProtocolCommand *command,
                                    uint32_t now_ms,
                                    bool command_valid);
uint32_t host_control_poll(HostControlState *state, uint32_t now_ms);
bool host_control_command_is_mutating(const CanProtocolCommand *command);
bool host_control_command_changes_snapshot(const CanProtocolCommand *command);

#ifdef __cplusplus
}
#endif

#endif
