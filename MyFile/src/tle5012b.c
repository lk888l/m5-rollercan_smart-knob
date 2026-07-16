/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "tle5012b.h"

#include "spi.h"

#define ENCODER_SPI_CS_H (ENCODER_SPI_CS_GROUP->BSRR = ENCODER_SPI_CS_PIN)
#define ENCODER_SPI_CS_L (ENCODER_SPI_CS_GROUP->BRR = ENCODER_SPI_CS_PIN)

#define ENCODER_SPI_BUSY_TIMEOUT_CYCLES 128U
#define ENCODER_DMA_PRIME_TIMEOUT_LOOPS 4096U
#define ENCODER_DMA_WORD_COUNT 2U

#define ENCODER_DMA_RX_CHANNEL DMA2_Channel1
#define ENCODER_DMA_TX_CHANNEL DMA2_Channel2
#define ENCODER_DMA_RX_FLAGS (DMA_ISR_TCIF1 | DMA_ISR_TEIF1)
#define ENCODER_DMA_TX_ERROR_FLAG DMA_ISR_TEIF2
#define ENCODER_DMA_CLEAR_RX_FLAGS DMA_IFCR_CGIF1
#define ENCODER_DMA_CLEAR_TX_FLAGS DMA_IFCR_CGIF2

ENCODER_SPI_Signal_Typedef encoder_spi;
ENCODER_Typedef encoder;

volatile uint32_t encoder_spi_timeout_count;
volatile uint32_t encoder_dma_start_count;
volatile uint32_t encoder_dma_complete_count;
volatile uint32_t encoder_dma_error_count;
volatile uint32_t encoder_dma_overlap_count;
volatile uint32_t encoder_dma_stale_sample_count;
volatile uint32_t encoder_dma_unexpected_irq_count;

static uint16_t encoder_dma_tx_words[ENCODER_DMA_WORD_COUNT] = {0x8021U, 0xFFFFU};
static volatile uint16_t encoder_dma_rx_words[ENCODER_DMA_WORD_COUNT];
static volatile uint8_t encoder_dma_in_flight;

static void EncoderCsHoldDelay(void)
{
  /* TLE5012B requires at least 105 ns CS hold time. At 168 MHz, 24
     core cycles provide about 143 ns before CS is released. */
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

static void EncoderResetSpiDisabled(void)
{
  const uint32_t cr1 = hspi1.Instance->CR1 & ~SPI_CR1_SPE;
  const uint32_t cr2 =
      hspi1.Instance->CR2 & ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  __HAL_SPI_DISABLE(&hspi1);
  __HAL_RCC_SPI1_FORCE_RESET();
  __DSB();
  __HAL_RCC_SPI1_RELEASE_RESET();

  /* A peripheral reset flushes both TX/RX FIFOs and clears OVR/FRE. Restore
     the CubeMX frame configuration but leave SPE low until CS is released. */
  hspi1.Instance->CR1 = cr1;
  hspi1.Instance->CR2 = cr2;
}

static void EncoderAbortDma(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* Publish the abort before touching interrupt-producing hardware. This
     prevents a just-completed priority-0 RX IRQ from accepting the old frame
     while the channels and SPI are being stopped. */
  encoder_dma_in_flight = 0U;
  __DMB();
  CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  CLEAR_BIT(ENCODER_DMA_RX_CHANNEL->CCR, DMA_CCR_EN);
  CLEAR_BIT(ENCODER_DMA_TX_CHANNEL->CCR, DMA_CCR_EN);
  DMA2->IFCR = ENCODER_DMA_CLEAR_RX_FLAGS | ENCODER_DMA_CLEAR_TX_FLAGS;
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);

  EncoderResetSpiDisabled();
  EncoderCsHoldDelay();
  ENCODER_SPI_CS_H;
  __HAL_SPI_ENABLE(&hspi1);
  __DMB();

  if (primask == 0U) {
    __enable_irq();
  }
}

static EncoderDmaIrqResult EncoderFinishDmaTransfer(void)
{
  uint32_t timeout = ENCODER_SPI_BUSY_TIMEOUT_CYCLES;

  CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  CLEAR_BIT(ENCODER_DMA_RX_CHANNEL->CCR, DMA_CCR_EN);
  CLEAR_BIT(ENCODER_DMA_TX_CHANNEL->CCR, DMA_CCR_EN);

  while ((hspi1.Instance->SR & SPI_SR_BSY) != 0U) {
    if (--timeout == 0U) {
      encoder_spi_timeout_count++;
      encoder_dma_error_count++;
      EncoderAbortDma();
      return ENCODER_DMA_IRQ_ERROR;
    }
  }

  EncoderCsHoldDelay();
  ENCODER_SPI_CS_H;
  DMA2->IFCR = ENCODER_DMA_CLEAR_RX_FLAGS | ENCODER_DMA_CLEAR_TX_FLAGS;
  __DMB();

  encoder_spi.sample_data = encoder_dma_rx_words[1];
  encoder_spi.angle = (uint16_t)((encoder_spi.sample_data & 0x7FFFU) >> 1U);
  encoder.angle_data = encoder_spi.angle;
  encoder_dma_in_flight = 0U;
  encoder_dma_complete_count++;
  return ENCODER_DMA_IRQ_COMPLETE;
}

void EncoderInit(void)
{
  encoder_spi = (ENCODER_SPI_Signal_Typedef){0};
  encoder = (ENCODER_Typedef){0};
  encoder_dma_rx_words[0] = 0U;
  encoder_dma_rx_words[1] = 0U;
  encoder_dma_in_flight = 0U;

  CLEAR_BIT(ENCODER_DMA_RX_CHANNEL->CCR, DMA_CCR_EN);
  CLEAR_BIT(ENCODER_DMA_TX_CHANNEL->CCR, DMA_CCR_EN);
  DMA2->IFCR = ENCODER_DMA_CLEAR_RX_FLAGS | ENCODER_DMA_CLEAR_TX_FLAGS;
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);

  ENCODER_DMA_RX_CHANNEL->CPAR = (uint32_t)&hspi1.Instance->DR;
  ENCODER_DMA_RX_CHANNEL->CMAR = (uint32_t)encoder_dma_rx_words;
  ENCODER_DMA_TX_CHANNEL->CPAR = (uint32_t)&hspi1.Instance->DR;
  ENCODER_DMA_TX_CHANNEL->CMAR = (uint32_t)encoder_dma_tx_words;

  CLEAR_BIT(ENCODER_DMA_RX_CHANNEL->CCR, DMA_CCR_HTIE);
  SET_BIT(ENCODER_DMA_RX_CHANNEL->CCR, DMA_CCR_TCIE | DMA_CCR_TEIE);
  CLEAR_BIT(ENCODER_DMA_TX_CHANNEL->CCR, DMA_CCR_TCIE | DMA_CCR_HTIE);
  SET_BIT(ENCODER_DMA_TX_CHANNEL->CCR, DMA_CCR_TEIE);

  ENCODER_SPI_CS_H;
  __HAL_SPI_ENABLE(&hspi1);
}

bool EncoderPrimeDmaRead(void)
{
  uint32_t timeout = ENCODER_DMA_PRIME_TIMEOUT_LOOPS;
  EncoderDmaIrqResult result;

  /* No fast-loop cycle is pending yet. Keep the DMA IRQs masked while the
     first transaction primes SPI, DMA and the cached encoder sample. */
  HAL_NVIC_DisableIRQ(DMA2_Channel1_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Channel2_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);

  if (!EncoderStartDmaReadFromISR()) {
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
    return false;
  }

  while ((DMA2->ISR &
          (DMA_ISR_TCIF1 | DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) == 0U) {
    if (--timeout == 0U) {
      encoder_spi_timeout_count++;
      encoder_dma_error_count++;
      EncoderAbortDma();
      HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
      HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
      return false;
    }
  }

  if ((DMA2->ISR & DMA_ISR_TEIF2) != 0U) {
    result = EncoderHandleDmaTxIRQ();
  } else {
    result = EncoderHandleDmaRxIRQ();
  }

  HAL_NVIC_ClearPendingIRQ(DMA2_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  return result == ENCODER_DMA_IRQ_COMPLETE;
}

bool EncoderStartDmaReadFromISR(void)
{
  if ((encoder_dma_in_flight != 0U) ||
      ((ENCODER_DMA_RX_CHANNEL->CCR & DMA_CCR_EN) != 0U) ||
      ((ENCODER_DMA_TX_CHANNEL->CCR & DMA_CCR_EN) != 0U) ||
      ((hspi1.Instance->SR & SPI_SR_BSY) != 0U)) {
    encoder_dma_overlap_count++;
    EncoderAbortDma();
    return false;
  }

  DMA2->IFCR = ENCODER_DMA_CLEAR_RX_FLAGS | ENCODER_DMA_CLEAR_TX_FLAGS;
  ENCODER_DMA_RX_CHANNEL->CNDTR = ENCODER_DMA_WORD_COUNT;
  ENCODER_DMA_TX_CHANNEL->CNDTR = ENCODER_DMA_WORD_COUNT;

  ENCODER_SPI_CS_L;
  encoder_dma_in_flight = 1U;
  encoder_dma_start_count++;
  __DMB();

  SET_BIT(ENCODER_DMA_RX_CHANNEL->CCR, DMA_CCR_EN);
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);
  SET_BIT(ENCODER_DMA_TX_CHANNEL->CCR, DMA_CCR_EN);
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
  return true;
}

EncoderDmaIrqResult EncoderHandleDmaRxIRQ(void)
{
  const uint32_t status = DMA2->ISR;

  if ((status & ENCODER_DMA_RX_FLAGS) == 0U) {
    encoder_dma_unexpected_irq_count++;
    DMA2->IFCR = ENCODER_DMA_CLEAR_RX_FLAGS;
    return ENCODER_DMA_IRQ_NONE;
  }

  if (encoder_dma_in_flight == 0U) {
    encoder_dma_unexpected_irq_count++;
    DMA2->IFCR = ENCODER_DMA_CLEAR_RX_FLAGS;
    return ENCODER_DMA_IRQ_NONE;
  }

  if ((status & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) != 0U) {
    encoder_dma_error_count++;
    EncoderAbortDma();
    return ENCODER_DMA_IRQ_ERROR;
  }

  return EncoderFinishDmaTransfer();
}

EncoderDmaIrqResult EncoderHandleDmaTxIRQ(void)
{
  const uint32_t status = DMA2->ISR;

  if ((status & ENCODER_DMA_TX_ERROR_FLAG) == 0U) {
    encoder_dma_unexpected_irq_count++;
    DMA2->IFCR = ENCODER_DMA_CLEAR_TX_FLAGS;
    return ENCODER_DMA_IRQ_NONE;
  }

  if (encoder_dma_in_flight == 0U) {
    encoder_dma_unexpected_irq_count++;
    DMA2->IFCR = ENCODER_DMA_CLEAR_TX_FLAGS;
    return ENCODER_DMA_IRQ_NONE;
  }

  encoder_dma_error_count++;
  EncoderAbortDma();
  return ENCODER_DMA_IRQ_ERROR;
}

uint16_t EncoderGetLatestAngle(void)
{
  return encoder.angle_data;
}

void EncoderRecordStaleSampleFromISR(void)
{
  encoder_dma_stale_sample_count++;
}

void EncoderRecordUnexpectedFastCompletionFromISR(void)
{
  encoder_dma_unexpected_irq_count++;
}
