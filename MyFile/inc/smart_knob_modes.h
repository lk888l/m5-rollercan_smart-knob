#ifndef __SMART_KNOB_MODES_H__
#define __SMART_KNOB_MODES_H__

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

/* Change this one definition to select the power-on SmartKnob preset. */
#ifndef SMART_KNOB_DEFAULT_MODE
#define SMART_KNOB_DEFAULT_MODE SMART_KNOB_MODE_COARSE_STRONG
#endif

void smart_knob_modes_initialize(void);
uint8_t smart_knob_modes_count(void);
const SmartKnobModeConfig *smart_knob_mode_get(uint8_t mode_index);
SmartKnobModeConfig *smart_knob_mode_get_mutable(uint8_t mode_index);
bool smart_knob_mode_apply_local_profile(uint8_t mode_index,
                                         uint8_t force_percent,
                                         uint16_t current_limit_ma,
                                         uint8_t step_width_deg);
uint8_t smart_knob_mode_default_width_deg(uint8_t mode_index);

#ifdef __cplusplus
}
#endif

#endif
