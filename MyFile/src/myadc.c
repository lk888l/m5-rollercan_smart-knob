/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "myadc.h"
#include "adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "tim.h"

#include "arm_math.h"
#include "arm_const_structs.h"

__IO uint16_t adc1_convbuf[10];
__IO uint16_t adc2_convbuf[10];

uint16_t ia_adc,ib_adc,ic_adc,vin_adc;
float32_t ia_offset,ib_offset,ic_offset;
float32_t ia,ib,ic;


float32_t theta,bx1,by1,bx2,by2;

float32_t ha_f,hb_f,angle_atan;

int32_t internal_temp = 0;
int32_t internal_temp_raw = 0;

uint8_t adc_cal_ok=0;

void MyADCInit(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc1_convbuf, 6);
    /* The control ISR polls the circular DMA buffer.  Half/full transfer
       callbacks are unused and would otherwise create high-rate IRQ traffic. */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);

    ia=0;ib=0;ic=0;
   
}

uint8_t MyADCZeroCal(void)
{
    enum { ZERO_CAL_SAMPLES = 256U, ZERO_CAL_TIMEOUT_MS = 100U };
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint32_t sum_c = 0U;
    const uint32_t started_ms = HAL_GetTick();
    const uint32_t current_samples_flag = __HAL_DMA_GET_HT_FLAG_INDEX(hadc1.DMA_Handle);

    adc_cal_ok = 0U;
    /* DMA is circular and its half-transfer interrupt is disabled. Its flag
     * still marks each fresh set of three phase-current samples. The final
     * three conversions leave these slots untouched while we copy them.
     * Polling the buffer alone
     * would repeatedly average the same sample and retain its offset noise.
     * At startup the bridge is off and TIM1's FOC interrupt is not enabled. */
    __HAL_DMA_CLEAR_FLAG(hadc1.DMA_Handle, current_samples_flag);
    for (uint32_t sample = 0U; sample < ZERO_CAL_SAMPLES; ++sample) {
        while (__HAL_DMA_GET_FLAG(hadc1.DMA_Handle, current_samples_flag) == RESET) {
            if ((HAL_GetTick() - started_ms) >= ZERO_CAL_TIMEOUT_MS) {
                return 0U;
            }
        }
        if ((HAL_GetTick() - started_ms) >= ZERO_CAL_TIMEOUT_MS) {
            return 0U;
        }
        __HAL_DMA_CLEAR_FLAG(hadc1.DMA_Handle, current_samples_flag);
        sum_a += adc1_convbuf[0];
        sum_b += adc1_convbuf[1];
        sum_c += adc1_convbuf[2];
    }

    /* Publish only a complete calibration. Retain the fractional ADC count:
     * rounding each phase zero creates an electrical-angle-dependent current
     * bias after the Clarke/Park transform, most noticeable at low torque. */
    ia_offset = (float32_t)sum_a / (float32_t)ZERO_CAL_SAMPLES;
    ib_offset = (float32_t)sum_b / (float32_t)ZERO_CAL_SAMPLES;
    ic_offset = (float32_t)sum_c / (float32_t)ZERO_CAL_SAMPLES;
    adc_cal_ok = 1U;
    return 1U;
}

void MyAdcProcess(void)
{


        ia_adc=adc1_convbuf[0];
        ib_adc=adc1_convbuf[1];
        ic_adc=adc1_convbuf[2];

        vin_adc=adc1_convbuf[3];

        internal_temp_raw = __HAL_ADC_CALC_TEMPERATURE(__HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc1_convbuf[5], ADC_RESOLUTION_12B), adc1_convbuf[4], ADC_RESOLUTION_12B);
    

    ia=ia_adc - ia_offset;
    ib=ib_adc - ib_offset;
    ic=ic_adc - ic_offset;
}

uint16_t MyAdcGetVal(uint8_t adc_hw,uint8_t channel)
{
    uint16_t adc_val;
    switch(adc_hw)
    {
        case 1:
        {
            switch(channel)
            {
                case 1:{ adc_val = adc1_convbuf[0]; break;}
                case 2:{ adc_val = adc1_convbuf[1]; break;}
                case 3:{ adc_val = adc1_convbuf[2]; break;}
                case 4:{ adc_val = adc1_convbuf[3]; break;}
            }
            break;
        }

        case 2:
        {
            switch(channel)
            {
                case 1:{ adc_val = adc2_convbuf[0]; break;}
                case 2:{ adc_val = adc2_convbuf[1]; break;}
                case 3:{ adc_val = adc2_convbuf[2]; break;}
                case 4:{ adc_val = adc2_convbuf[3]; break;}
            }
        }
    }
    return adc_val;
}

float32_t MyAdcGetCurrent(uint8_t channel)
{
    float32_t current_val = 0.0f;
    switch(channel)
    {
    case 1:{ current_val = ia; break;}
    case 2:{ current_val = ib; break;}
    case 3:{ current_val = ic; break;}

    default: break;

    }
    return current_val;
}
