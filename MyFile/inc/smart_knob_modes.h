#ifndef __SMART_KNOB_MODES_H__
#define __SMART_KNOB_MODES_H__

#include "local_profile.h"
#include "smart_knob.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SMART_KNOB_MODE_CUSTOM = 0,
    SMART_KNOB_MODE_UNBOUNDED_SMOOTH,
    SMART_KNOB_MODE_BOUNDED_SMOOTH,
    SMART_KNOB_MODE_MULTI_REV_SMOOTH,
    SMART_KNOB_MODE_ON_OFF_STRONG,
    SMART_KNOB_MODE_RETURN_TO_CENTER,
    SMART_KNOB_MODE_FINE_SMOOTH,
    SMART_KNOB_MODE_FINE_DETENTS,
    SMART_KNOB_MODE_COARSE_STRONG,
    SMART_KNOB_MODE_COARSE_WEAK,
    SMART_KNOB_MODE_MAGNETIC,
    SMART_KNOB_MODE_RETURN_TO_CENTER_DETENTS,
    SMART_KNOB_MODE_COUNT
} SmartKnobMode;

enum {
    SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT = 8,
    SMART_KNOB_SNAPSHOT_CUSTOM_VALUE_COUNT = 14,
};

/* Values use the existing RollerCAN protocol representation: signed
   little-endian int32 values, normally scaled by 1000. The max-current
   permille, positions and LED hue retain their protocol integer units. */
typedef struct {
    uint8_t selected_mode;
    int32_t mode_values[SMART_KNOB_MODE_COUNT]
                       [SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT];
    int32_t custom_values[SMART_KNOB_SNAPSHOT_CUSTOM_VALUE_COUNT];
} SmartKnobPersistentSnapshot;

enum {
    SMART_KNOB_LOCAL_UNREPRESENTABLE_FORCE = 1U << 0,
    SMART_KNOB_LOCAL_UNREPRESENTABLE_LIMIT = 1U << 1,
    SMART_KNOB_LOCAL_UNREPRESENTABLE_WIDTH = 1U << 2,
    SMART_KNOB_LOCAL_UNREPRESENTABLE_ADVANCED = 1U << 3,
};

/* Change this one definition to select the power-on SmartKnob preset. */
#ifndef SMART_KNOB_DEFAULT_MODE
#define SMART_KNOB_DEFAULT_MODE SMART_KNOB_MODE_COARSE_STRONG
#endif

void smart_knob_modes_initialize(void);
void smart_knob_modes_reset_defaults(void);
uint8_t smart_knob_modes_count(void);
const SmartKnobModeConfig *smart_knob_mode_get(uint8_t mode_index);
const SmartKnobModeConfig *smart_knob_mode_get_default(uint8_t mode_index);
SmartKnobModeConfig *smart_knob_mode_get_mutable(uint8_t mode_index);
bool smart_knob_mode_apply_local_profile(uint8_t mode_index,
                                         uint8_t force_percent,
                                         uint16_t current_limit_ma,
                                         uint8_t step_width_deg);
bool smart_knob_mode_apply_local_edit(const LocalProfileEdit *edit);
bool smart_knob_snapshot_export(SmartKnobPersistentSnapshot *snapshot);
bool smart_knob_snapshot_import(const SmartKnobPersistentSnapshot *snapshot);
bool smart_knob_mode_local_projection(uint8_t mode_index,
                                      LocalProfileEdit *profile,
                                      uint8_t *unrepresentable_mask);
uint8_t smart_knob_mode_default_width_deg(uint8_t mode_index);

#ifdef __cplusplus
}
#endif

#endif
