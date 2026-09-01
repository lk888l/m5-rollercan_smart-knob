#ifndef LOCAL_PROFILE_H
#define LOCAL_PROFILE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    LOCAL_PROFILE_DIRTY_MODE = 1U << 0,
    LOCAL_PROFILE_DIRTY_FORCE = 1U << 1,
    LOCAL_PROFILE_DIRTY_WIDTH = 1U << 2,
    LOCAL_PROFILE_DIRTY_LIMIT = 1U << 3,
};

typedef struct {
    uint8_t mode;
    uint8_t force_percent;
    uint8_t current_limit_10ma;
    uint8_t step_width_deg;
    uint8_t dirty_mask;
} LocalProfileEdit;

#ifdef __cplusplus
}
#endif

#endif
