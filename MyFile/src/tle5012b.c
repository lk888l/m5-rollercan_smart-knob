/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "arm_math.h"
#include "arm_const_structs.h"

#include "encoder.h"
#include "spi.h"

#define ENCODER_SPI_CS_H ENCODER_SPI_CS_GROUP->BSRR	=	ENCODER_SPI_CS_PIN
#define ENCODER_SPI_CS_L ENCODER_SPI_CS_GROUP->BRR	=	ENCODER_SPI_CS_PIN
#define ENCODER_SPI_TIMEOUT_CYCLES 4096U


ENCODER_SPI_Signal_Typedef	encoder_spi;
volatile uint32_t encoder_spi_timeout_count;

	uint16_t data_t[2];
	uint16_t data_r[2];

uint8_t SPI_TransmitReceive(SPI_HandleTypeDef * hspi, uint16_t TxData, uint16_t *RxData)            
{
  uint32_t timeout = ENCODER_SPI_TIMEOUT_CYCLES;

  while ((hspi->Instance->SR & SPI_SR_TXE) == 0U) {
    if (--timeout == 0U) {
      encoder_spi_timeout_count++;
      return 1U;
    }
  }

  hspi->Instance->DR = TxData;

  timeout = ENCODER_SPI_TIMEOUT_CYCLES;
  while ((hspi->Instance->SR & SPI_SR_RXNE) == 0U) {
    if (--timeout == 0U) {
      /* Never let an absent or not-yet-enabled encoder lock the priority-0
       * FOC ISR forever.  Re-arm SPI so a later sample can recover. */
      encoder_spi_timeout_count++;
      __HAL_SPI_DISABLE(hspi);
      __HAL_SPI_ENABLE(hspi);
      return 1U;
    }
  }

  *RxData = (uint16_t)hspi->Instance->DR;
  return 0U;
}

void EncoderInit(void)
{
	encoder_spi.sample_data = 0;
	encoder_spi.angle = 0;
	ENCODER_SPI_CS_H;
	__HAL_SPI_ENABLE(&hspi1);
}

void EncoderGetData(void)
{

	data_t[0] = 0x8021;
	data_t[1] = 0xffff;

		//读取SPI数据
		ENCODER_SPI_CS_L;
		ENCODER_SPI_CS_L;

		if (SPI_TransmitReceive(&hspi1, data_t[0], &data_r[0]) != 0U ||
		    SPI_TransmitReceive(&hspi1, data_t[1], &data_r[1]) != 0U) {
			ENCODER_SPI_CS_H;
			return;
		}

		ENCODER_SPI_CS_H;
		ENCODER_SPI_CS_H;
		encoder_spi.sample_data = data_r[1];

		encoder_spi.angle = (((encoder_spi.sample_data & 0x7fff) << 1) >> 2);//&0X7FFF;
}

ENCODER_Typedef	encoder;

float32_t EncoderGetAngle(void)
{
	EncoderGetData();
	encoder.angle_data = encoder_spi.angle;   
	return encoder.angle_data;
}
