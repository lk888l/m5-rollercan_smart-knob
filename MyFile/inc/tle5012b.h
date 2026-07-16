/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __TLE5012B_H__
#define __TLE5012B_H__

#include "stm32g4xx.h"
#include "arm_const_structs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ENCODER_SPI_CS_GROUP GPIOA
#define ENCODER_SPI_CS_PIN GPIO_PIN_4
#define ENCODER_SPI_HW (hspi1)

typedef struct {
  uint16_t sample_data;
  uint16_t angle;
  bool no_mag_flag;
  bool pc_flag;
} ENCODER_SPI_Signal_Typedef;

typedef struct {
  uint16_t angle_data;
  uint16_t rectify_angle;
  bool rectify_valid;
} ENCODER_Typedef;

typedef enum {
  ENCODER_DMA_IRQ_NONE = 0,
  ENCODER_DMA_IRQ_COMPLETE,
  ENCODER_DMA_IRQ_ERROR,
} EncoderDmaIrqResult;

extern ENCODER_Typedef encoder;
extern volatile uint32_t encoder_spi_timeout_count;
extern volatile uint32_t encoder_dma_start_count;
extern volatile uint32_t encoder_dma_complete_count;
extern volatile uint32_t encoder_dma_error_count;
extern volatile uint32_t encoder_dma_overlap_count;
extern volatile uint32_t encoder_dma_stale_sample_count;
extern volatile uint32_t encoder_dma_unexpected_irq_count;

void EncoderInit(void);
bool EncoderPrimeDmaRead(void);
bool EncoderStartDmaReadFromISR(void);
EncoderDmaIrqResult EncoderHandleDmaRxIRQ(void);
EncoderDmaIrqResult EncoderHandleDmaTxIRQ(void);
uint16_t EncoderGetLatestAngle(void);
void EncoderRecordStaleSampleFromISR(void);
void EncoderRecordUnexpectedFastCompletionFromISR(void);

#endif
