#ifndef SMART_KNOB_PERSISTENCE_H
#define SMART_KNOB_PERSISTENCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "smart_knob_modes.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SMART_KNOB_FLASH_LEGACY_SIZE = 48,
    SMART_KNOB_FLASH_TOTAL_SIZE = 512,
    SMART_KNOB_FLASH_EXTENSION_OFFSET = 48,
};

bool smart_knob_persistence_encode(
    uint8_t *flash_bytes,
    size_t flash_size,
    const SmartKnobPersistentSnapshot *snapshot);

bool smart_knob_persistence_decode(
    const uint8_t *flash_bytes,
    size_t stored_length,
    SmartKnobPersistentSnapshot *snapshot);

uint32_t smart_knob_persistence_crc32(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
