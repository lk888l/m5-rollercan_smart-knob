#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t command_id;
  uint8_t command_parameter;
  uint16_t option;
  uint8_t data[8];
} CanProtocolCommand;

typedef struct {
  uint8_t command_id;
  uint8_t destination_id;
  uint16_t option;
  uint8_t data[8];
  bool valid;
} CanProtocolResponse;

#ifdef __cplusplus
}
#endif

#endif
