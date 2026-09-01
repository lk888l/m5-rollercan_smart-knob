#include "smart_knob_persistence.h"

#include <string.h>

#define SNAPSHOT_MAGIC 0x324E4B53UL /* "SKN2" in little endian. */
#define SNAPSHOT_VERSION 1U
#define SNAPSHOT_HEADER_SIZE 16U
#define SNAPSHOT_PAYLOAD_OFFSET \
    (SMART_KNOB_FLASH_EXTENSION_OFFSET + SNAPSHOT_HEADER_SIZE)
#define SNAPSHOT_MODE_RECORD_SIZE \
    (SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT * sizeof(int32_t))
#define SNAPSHOT_MODE_PAYLOAD_SIZE \
    (SMART_KNOB_MODE_COUNT * SNAPSHOT_MODE_RECORD_SIZE)
#define SNAPSHOT_CUSTOM_PAYLOAD_SIZE \
    (SMART_KNOB_SNAPSHOT_CUSTOM_VALUE_COUNT * sizeof(int32_t))
#define SNAPSHOT_PAYLOAD_SIZE \
    (SNAPSHOT_MODE_PAYLOAD_SIZE + SNAPSHOT_CUSTOM_PAYLOAD_SIZE)
#define SNAPSHOT_CRC_OFFSET (SMART_KNOB_FLASH_EXTENSION_OFFSET + 12U)

_Static_assert(SNAPSHOT_PAYLOAD_OFFSET + SNAPSHOT_PAYLOAD_SIZE <=
                   SMART_KNOB_FLASH_TOTAL_SIZE,
               "SmartKnob snapshot exceeds the reserved 512-byte area");
_Static_assert((SMART_KNOB_FLASH_TOTAL_SIZE % 8U) == 0U,
               "Flash payload must remain double-word aligned");
_Static_assert(SMART_KNOB_FLASH_TOTAL_SIZE + 8U <= 2048U,
               "Flash package must fit in page 59");

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16_le(const uint8_t *source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static uint32_t get_u32_le(const uint8_t *source)
{
    return (uint32_t)source[0] |
           ((uint32_t)source[1] << 8) |
           ((uint32_t)source[2] << 16) |
           ((uint32_t)source[3] << 24);
}

static void put_i32_le(uint8_t *destination, int32_t value)
{
    put_u32_le(destination, (uint32_t)value);
}

static int32_t get_i32_le(const uint8_t *source)
{
    return (int32_t)get_u32_le(source);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    while (length-- != 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320UL &
                                (uint32_t)-(int32_t)(crc & 1U));
        }
    }
    return crc;
}

uint32_t smart_knob_persistence_crc32(const uint8_t *data, size_t length)
{
    if (data == NULL) {
        return 0U;
    }
    return crc32_update(UINT32_MAX, data, length) ^ UINT32_MAX;
}

static uint32_t snapshot_crc(const uint8_t *flash_bytes)
{
    uint32_t crc = UINT32_MAX;
    crc = crc32_update(crc,
                       &flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET],
                       12U);
    crc = crc32_update(crc,
                       &flash_bytes[SNAPSHOT_PAYLOAD_OFFSET],
                       SNAPSHOT_PAYLOAD_SIZE);
    return crc ^ UINT32_MAX;
}

bool smart_knob_persistence_encode(
    uint8_t *flash_bytes,
    size_t flash_size,
    const SmartKnobPersistentSnapshot *snapshot)
{
    if (flash_bytes == NULL || snapshot == NULL ||
        flash_size < SMART_KNOB_FLASH_TOTAL_SIZE ||
        snapshot->selected_mode >= SMART_KNOB_MODE_COUNT) {
        return false;
    }

    memset(&flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET], 0,
           SMART_KNOB_FLASH_TOTAL_SIZE - SMART_KNOB_FLASH_EXTENSION_OFFSET);
    put_u32_le(&flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET], SNAPSHOT_MAGIC);
    flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 4U] = SNAPSHOT_VERSION;
    flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 5U] =
        SMART_KNOB_MODE_COUNT;
    flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 6U] =
        snapshot->selected_mode;
    flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 7U] = 0U;
    put_u16_le(&flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 8U],
               (uint16_t)SNAPSHOT_PAYLOAD_SIZE);
    put_u16_le(&flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 10U], 0U);

    size_t offset = SNAPSHOT_PAYLOAD_OFFSET;
    for (uint8_t mode = 0U; mode < SMART_KNOB_MODE_COUNT; ++mode) {
        for (uint8_t value = 0U;
             value < SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT;
             ++value) {
            put_i32_le(&flash_bytes[offset], snapshot->mode_values[mode][value]);
            offset += sizeof(int32_t);
        }
    }
    for (uint8_t value = 0U;
         value < SMART_KNOB_SNAPSHOT_CUSTOM_VALUE_COUNT;
         ++value) {
        put_i32_le(&flash_bytes[offset], snapshot->custom_values[value]);
        offset += sizeof(int32_t);
    }

    put_u32_le(&flash_bytes[SNAPSHOT_CRC_OFFSET], snapshot_crc(flash_bytes));
    return true;
}

bool smart_knob_persistence_decode(
    const uint8_t *flash_bytes,
    size_t stored_length,
    SmartKnobPersistentSnapshot *snapshot)
{
    if (flash_bytes == NULL || snapshot == NULL ||
        stored_length < SMART_KNOB_FLASH_TOTAL_SIZE ||
        get_u32_le(&flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET]) !=
            SNAPSHOT_MAGIC ||
        flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 4U] !=
            SNAPSHOT_VERSION ||
        flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 5U] !=
            SMART_KNOB_MODE_COUNT ||
        flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 6U] >=
            SMART_KNOB_MODE_COUNT ||
        get_u16_le(&flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 8U]) !=
            SNAPSHOT_PAYLOAD_SIZE ||
        get_u32_le(&flash_bytes[SNAPSHOT_CRC_OFFSET]) !=
            snapshot_crc(flash_bytes)) {
        return false;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->selected_mode =
        flash_bytes[SMART_KNOB_FLASH_EXTENSION_OFFSET + 6U];
    size_t offset = SNAPSHOT_PAYLOAD_OFFSET;
    for (uint8_t mode = 0U; mode < SMART_KNOB_MODE_COUNT; ++mode) {
        for (uint8_t value = 0U;
             value < SMART_KNOB_SNAPSHOT_MODE_VALUE_COUNT;
             ++value) {
            snapshot->mode_values[mode][value] = get_i32_le(&flash_bytes[offset]);
            offset += sizeof(int32_t);
        }
    }
    for (uint8_t value = 0U;
         value < SMART_KNOB_SNAPSHOT_CUSTOM_VALUE_COUNT;
         ++value) {
        snapshot->custom_values[value] = get_i32_le(&flash_bytes[offset]);
        offset += sizeof(int32_t);
    }
    return true;
}
