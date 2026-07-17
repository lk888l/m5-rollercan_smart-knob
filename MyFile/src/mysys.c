/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "mysys.h"
#include "motordriver.h"
#include "myadc.h"
#include "tim.h"
#include "tle5012b.h"

#include "bmp.h"

#include "arm_const_structs.h"

#include "oled_u8g2.h"
#include "u8g2_disp_fun.h"
#include "encoder.h"
#include "ws2812.h"
#include "i2c.h"
#include "i2c_ex.h"
// #include "usart.h"
#include <stdio.h>
#include <string.h>
#include "pid_controller.h"
#include "smart_knob.h"
#include "arm_math.h"
#include "fdcan.h"
#include "app_rtos.h"
#include "fast_control_link.h"
#include "runtime_metrics.h"

#define MAX_RECORD_SIZE 256
#define MAX_STALLED_CURRENT 500

uint16_t counter_loop_foc,counter_loop_control;

uint16_t counter_rpm = 0, pid_compute_counter = 0;

uint16_t encoder_counter = 0;

float32_t vol_input,vol_lpf, temp_lpf;

float32_t ph_current_rt,ph_crrent_lpf;
uint16_t disp_ph_current;

int32_t adc_current_pha,adc_current_phb,adc_current_phc;
uint16_t abs_pha,abs_phb,abs_phc,ph_current_max;

float32_t encoder_absolute_angle_new,encoder_absolute_angle_old;
float32_t mechanical_angle,mechanical_turns,last_mechanical_turns; 
float32_t mechanical_rad; 
int32_t diff_encoder_value = 0;
float32_t diff_encoder_value_lpf = 0.0f;
float32_t motor_rpm; 
float32_t motor_rps; 
float32_t angle_target,angle_error; 

float32_t angle_kp,angle_ki,angle_kd;
float32_t uq_limit;
uint16_t dbg1,dbg2,dbg3;

float32_t debug=4.0f;

uint32_t act_delay = 0;
uint32_t status_flag = 0;

uint8_t speed_pid_index = 0;
uint32_t speed_pid_int[3] = {1500000, 100000, 40000000};
float speed_pid_float[3] = {15.0f, 0.01f, 400.0f};

uint32_t speed_pid_low_int[3] = {1500000, 100000, 40000000};
float speed_pid_low_float[3] = {15.0f, 0.01f, 400.0f};
uint32_t speed_pid_mid_int[3] = {2500000, 30, 20000000};
float speed_pid_mid_float[3] = {25.0f, 0.000003f, 200.0f};
uint32_t speed_pid_high_int[3] = {2500000, 30, 40000000};
float speed_pid_high_float[3] = {25.0f, 0.000003f, 400.0f};

uint8_t pos_pid_index = 0;
uint32_t pos_pid_int[3] = {1500000, 30, 40000000};
float pos_pid_float[3] = {15.0f, 0.000003f, 400.0f};

uint32_t pos_pid_low_int[3] = {1500000, 30, 40000000};
float pos_pid_low_float[3] = {15.0f, 0.000003f, 400.0f};
uint32_t pos_pid_mid_int[3] = {1500000, 1, 400000000};
float pos_pid_mid_float[3] = {15.0f, 0.0000001f, 4000.0f};
uint32_t pos_pid_high_int[3] = {1500000, 1, 800000000};
float pos_pid_high_float[3] = {15.0f, 0.0000001f, 8000.0f};

PIDControl pid_ctrl_speed_t;
PIDControl pid_ctrl_pos_t;
int32_t speed_point = 0;
int32_t max_speed_current = 100000;
int32_t max_pos_current = 100000;
int32_t pos_point = 0;
int32_t current_point = 0;
float current_point_float = 0.0f;

uint8_t error_code = ERR_NONE;
uint8_t over_vol_protect_mode = 0;
uint8_t over_vol_protect_auto_flag = 0;
uint32_t over_vol_protect_auto_counter = 0;
uint8_t over_vol_flag = 0;

uint8_t over_value_flag = 0;

uint8_t err_recover_try_max = 5;
uint8_t err_stalled_flag = 0;
uint16_t speed_err_value = 0;
uint8_t speed_err_timeout = 0;
uint8_t speed_err_count_flag = 0;
uint8_t speed_err_auto_flag = 0;
uint32_t speed_err_counter = 0;
uint32_t speed_err_auto_counter = 0;
uint32_t speed_err_recover_counter = 0;
uint32_t speed_err_recover_try_counter = 0;

float speed_err_rate = 0.8f;
float pos_err_rate = 0.4f;

uint16_t pos_err_value = 0;
uint8_t pos_err_timeout = 0;
uint8_t pos_err_count_flag = 0;
uint8_t pos_err_auto_flag = 0;
uint32_t pos_err_counter = 0;
uint32_t pos_err_auto_counter = 0;
uint32_t pos_err_recover_counter = 0;
uint32_t pos_err_recover_try_counter = 0;

uint8_t sys_status = SYS_STANDBY;
uint8_t running_index = 0;

volatile uint32_t i2c_stop_timeout_delay = 0;

volatile uint32_t usart_tx_delay = 0;
uint8_t usart_tx_flag = 0;

uint8_t dis_show_flag = DIS_INFO;
uint8_t last_dis_show_flag = DIS_INFO;

/* SmartKnob is the product's default application mode.  The active haptic
   preset itself is selected in smart_knob_modes.h. */
uint8_t motor_mode = MODE_DIAL;
uint8_t last_motor_mode = MODE_DIAL;

uint8_t motor_id = 0;

uint16_t angle_cal_offset = 0;

uint8_t motor_output = 0;

float speed_record[MAX_RECORD_SIZE] = {0};
uint8_t record_index = 0;
volatile uint8_t avg_filter_level = 20;
float32_t uq_output = 0.0f;
float diff_pos_debug = 0.0f;

uint8_t mode_switch_flag = 0;
uint8_t motor_stall_protection_flag = 1;
uint8_t motor_overvalue_protection_flag = 0;

float rpm_rps_count_temp = 0;

uint32_t bps_list[3] = {1, 2, 8};
uint8_t bps_index = 0;
float brightness_list[4] = {1.0f, 0.5f, 0.2f, 0.0f};
uint8_t brightness_index = 100;

uint8_t rgb_show_mode = 0;
uint32_t rgb_color_buffer[RGB_BUFFER_SIZE] = {0};
uint32_t rgb_color_buffer_index = 0;
uint32_t lastest_rgb_color = 0;

#define LEGACY_CONTROL_RATE_HZ 5600.0f
#define LEGACY_MODE_RATE_HZ (56000.0f / 11.0f)
#define CONTROL_TASK_RATE_HZ 1000.0f
#define OVER_VOLTAGE_TRIP_CENTIVOLTS 1800.0f
#define OVER_VOLTAGE_RELEASE_CENTIVOLTS 1750.0f
#define OVER_VOLTAGE_RESTART_DELAY_MS 300U
#define FAST_LOOP_MIN_START_MARGIN_TIM1_TICKS 640U
#define FLASH_DEFAULTS_VERSION_INDEX (FLASH_DATA_SIZE - 1U)
#define FLASH_DEFAULTS_CAN_V1 0xC1U

static volatile uint8_t control_task_active = 0;
static float control_filter_alpha =
    (1.0f / (1.0f + 1.0f/(2.0f * PI * 0.0002f * 2.0f)));
static float speed_filter_alpha =
    (1.0f / (1.0f + 1.0f/(2.0f * PI * (1.0f / LEGACY_CONTROL_RATE_HZ) * 2.0f)));
static float encoder_counts_to_rpm = 60.0f * LEGACY_CONTROL_RATE_HZ;
static float encoder_counts_to_rps = 2.0f * PI * LEGACY_CONTROL_RATE_HZ;
static FastSensorSnapshot control_sensor_snapshot;
static volatile uint8_t fast_cycle_pending = 0U;
static uint32_t fast_cycle_start_cycles = 0U;
static uint32_t fast_cycle_start_active_cycles = 0U;
volatile uint32_t fast_loop_late_start_count;
volatile uint32_t fast_loop_sync_timeout_count;

void Rpm_Count_100us(void);

static void schedule_over_voltage_recovery(uint32_t now_ms)
{
  if (over_vol_protect_mode && motor_output && error_code == ERR_NONE &&
      !err_stalled_flag && !over_value_flag && motor_mode < MODE_MAX) {
    over_vol_protect_auto_counter = now_ms;
    over_vol_protect_auto_flag = 1U;
  }
  else {
    over_vol_protect_auto_flag = 0U;
  }
}

static void service_over_voltage_recovery(uint32_t now_ms)
{
  if (!over_vol_protect_auto_flag) {
    return;
  }
  if (!over_vol_protect_mode || !motor_output || over_vol_flag ||
      error_code != ERR_NONE || err_stalled_flag || over_value_flag ||
      motor_mode >= MODE_MAX) {
    over_vol_protect_auto_flag = 0U;
    return;
  }
  if ((now_ms - over_vol_protect_auto_counter) < OVER_VOLTAGE_RESTART_DELAY_MS) {
    return;
  }

  /* Publish zero before RUN so the ISR cannot reuse a pre-fault current target.
     Dial mode also gets a fresh position baseline and its normal settle delay. */
  MotorDriverSetCurrentReal(0.0f);
  if (motor_mode == MODE_DIAL) {
    init_smart_knob();
  }
  MotorDriverSetMode(MDRV_MODE_RUN);
  over_vol_protect_auto_flag = 0U;
}

/*
 * A new electrical offset changes the encoder's coordinate system.  Treat it
 * like a clean boot for all angle-derived state before SmartKnob is armed;
 * otherwise the old speed/multiturn history is interpreted as real motion.
 */
static void rebase_encoder_after_calibration(void)
{
  float32_t single_turn_angle;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  single_turn_angle = MotorDriverGetMechanicalAngle() / 10.0f;
  encoder_reset_tracking();
  speed_encoder_update();
  speed_encoder_value_t.last_encoder_value = speed_encoder_value_t.encoder_value;

  encoder_absolute_angle_old = single_turn_angle;
  encoder_absolute_angle_new = single_turn_angle;
  mechanical_turns = 0.0f;
  last_mechanical_turns = 0.0f;
  mechanical_angle = single_turn_angle;
  mechanical_rad = single_turn_angle * PI / 180.0f;
  diff_encoder_value = 0;
  diff_encoder_value_lpf = 0.0f;
  rpm_rps_count_temp = 0.0f;
  motor_rpm = 0.0f;
  motor_rps = 0.0f;

  if (primask == 0U) {
    __enable_irq();
  }
}

void _sys_exit(int x){x = x;}

void init_pid(void)
{
  switch (speed_pid_index)
  {
  case 0:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_float[0], 
        speed_pid_float[1], 
        speed_pid_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  case 1:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_low_float[0], 
        speed_pid_low_float[1], 
        speed_pid_low_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  case 2:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_mid_float[0], 
        speed_pid_mid_float[1], 
        speed_pid_mid_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  case 3:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_high_float[0], 
        speed_pid_high_float[1], 
        speed_pid_high_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  
  default:
    break;
  }

  switch (pos_pid_index)
  {
  case 0:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_float[0], 
        pos_pid_float[1], 
        pos_pid_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_pos_t.setpoint = 0;  
    break;
  case 1:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_low_float[0], 
        pos_pid_low_float[1], 
        pos_pid_low_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_pos_t.setpoint = 0;  
    break;
  case 2:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_mid_float[0], 
        pos_pid_mid_float[1], 
        pos_pid_mid_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_pos_t.setpoint = 0;  
    break;
  case 3:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_high_float[0], 
        pos_pid_high_float[1], 
        pos_pid_high_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_pos_t.setpoint = 0;  
    break;
  
  default:
    break;
  }
}

void speed_pid(void)
{
  pid_ctrl_speed_t.input = motor_rpm;
  if (sys_status == SYS_RUNNING) {
    if (fabsf(pid_ctrl_speed_t.point_error/pid_ctrl_speed_t.setpoint > 0.5f)) {
      rgb_flash_slow = 0;
    }
    else {
      rgb_flash_slow = 1;   
    }  
  }
  if (0) {
    pid_ctrl_speed_t.iTerm = 0;
    MotorDriverSetCurrentReal(0.0f);
  }
  else {
    PIDCompute(&pid_ctrl_speed_t);
    if (speed_err_value) {
      // 第一次堵转判断
      if (!speed_err_auto_flag) {
        if (fabsf(pid_ctrl_speed_t.point_error) >= speed_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (!speed_err_count_flag) {
            speed_err_counter = HAL_GetTick();
            speed_err_count_flag = 1;
          }
        }
        else {
          if (speed_err_count_flag == 1)
            speed_err_count_flag = 0;
        }

        // 跳转到堵转保护模式，2S后自动恢复
        if (speed_err_count_flag == 1 && HAL_GetTick() - speed_err_counter > speed_err_timeout * 1000) {
          motor_mode = MODE_SPEED_ERR_PROTECT;
          error_code |= ERR_STALLED;
          MotorDriverSetMode(MDRV_MODE_OFF);
          pid_ctrl_speed_t.iTerm = 0;
          speed_err_auto_counter = HAL_GetTick();
          speed_err_auto_flag = 1;
          return;
        }        
      }
      // 自动恢复后判断是否有堵转
      else {
        if (speed_err_count_flag == 2) {
          speed_err_recover_counter = HAL_GetTick();
          speed_err_count_flag = 1;
        }
        if (fabsf(pid_ctrl_speed_t.point_error) >= speed_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (HAL_GetTick() - speed_err_recover_counter > 500) {
            motor_mode = MODE_SPEED_ERR_PROTECT;
            error_code |= ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
            pid_ctrl_speed_t.iTerm = 0;
            speed_err_auto_counter = HAL_GetTick();          
          }
        }
        else {
          if (HAL_GetTick() - speed_err_recover_counter > 500) {
            speed_err_recover_try_counter = 0;
            error_code &= ~ERR_STALLED;
            speed_err_count_flag = 0;
            speed_err_auto_flag = 0; 
            err_stalled_flag = 0;       
          }          
        }
        if (motor_disable_flag) {
          if (motor_mode == MODE_SPEED_ERR_PROTECT) {
            MotorDriverSetMode(MDRV_MODE_OFF);
            err_stalled_flag = 1;
          }
          else {
            error_code &= ~ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
          }
          motor_disable_flag = 0;
        }         
      }

      MotorDriverSetCurrentReal(pid_ctrl_speed_t.output);
    }
    else {
      if (!err_stalled_flag) {
        speed_err_count_flag = 0;
        error_code &= ~ERR_STALLED;
        speed_err_auto_flag = 0;
      }
      else {
        pid_ctrl_speed_t.iTerm = 0;
      }
      MotorDriverSetCurrentReal(pid_ctrl_speed_t.output);
    }
  }
}

void pos_pid(void)
{
  static uint8_t i_overflow_count_flag = 0;
  static uint8_t i_overflow_counter = 0;

  pid_ctrl_pos_t.input = mechanical_angle;
  if ((pid_ctrl_pos_t.point_error <= 360.0f || pid_ctrl_pos_t.point_error >= -360.0f) && 
  (pid_ctrl_pos_t.iTerm > 30.0f || pid_ctrl_pos_t.iTerm < -30.0f) &&
  (pid_ctrl_pos_t.output < 30.0f || pid_ctrl_pos_t.output > -30.0f) && !i_overflow_count_flag) {
    i_overflow_count_flag = 1;
    i_overflow_counter = HAL_GetTick();
  }

  if (i_overflow_count_flag && HAL_GetTick() - i_overflow_counter > 100) {
    pid_ctrl_pos_t.iTerm = 0;
    i_overflow_count_flag = 0;
  } 
  if (sys_status == SYS_RUNNING) {
    if (fabsf(pid_ctrl_pos_t.point_error) > 10.0f) {
      rgb_flash_slow = 0;
    }  
    else {
      rgb_flash_slow = 1;
    }    
  }
  PIDCompute(&pid_ctrl_pos_t);
    if (pos_err_value) {
      // 第一次堵转判断
      if (!pos_err_auto_flag) {
        if (fabsf(pid_ctrl_pos_t.point_error) >= pos_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (!pos_err_count_flag) {
            pos_err_counter = HAL_GetTick();
            pos_err_count_flag = 1;
          }
        }
        else {
          if (pos_err_count_flag == 1)
            pos_err_count_flag = 0;
        }

        // 跳转到堵转保护模式，2S后自动恢复
        if (pos_err_count_flag == 1 && HAL_GetTick() - pos_err_counter > pos_err_timeout * 1000) {
          motor_mode = MODE_POS_ERR_PROTECT;
          error_code |= ERR_STALLED;
          MotorDriverSetMode(MDRV_MODE_OFF);
          pid_ctrl_pos_t.iTerm = 0;
          pos_err_auto_counter = HAL_GetTick();
          pos_err_auto_flag = 1;
          return;
        }        
      }
      // 自动恢复后判断是否有堵转
      else {
        if (pos_err_count_flag == 2) {
          pos_err_recover_counter = HAL_GetTick();
          pos_err_count_flag = 1;
        }
        if (fabsf(pid_ctrl_pos_t.point_error) >= pos_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (HAL_GetTick() - pos_err_recover_counter > 500) {
            motor_mode = MODE_POS_ERR_PROTECT;
            error_code |= ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
            pid_ctrl_pos_t.iTerm = 0;
            pos_err_auto_counter = HAL_GetTick();          
          }
        }
        else {
          if (HAL_GetTick() - pos_err_recover_counter > 500) {
            pos_err_recover_try_counter = 0;
            error_code &= ~ERR_STALLED;
            pos_err_count_flag = 0;
            pos_err_auto_flag = 0; 
            err_stalled_flag = 0;       
          }          
        }
        if (motor_disable_flag) {
          if (motor_mode == MODE_POS_ERR_PROTECT) {
            MotorDriverSetMode(MDRV_MODE_OFF);
            err_stalled_flag = 1;
          }
          else {
            error_code &= ~ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
          }
          motor_disable_flag = 0;
        }         
      }

      MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
    }
    else {
      if (!err_stalled_flag) {
        pos_err_count_flag = 0;
        error_code &= ~ERR_STALLED;
        pos_err_auto_flag = 0;
      }
      else {
        pid_ctrl_pos_t.iTerm = 0;
      }
      MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
    }  
  MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
   
}

uint8_t crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while(len--)
    {
        crc ^= *data++;
        for(i = 0;i < 8;i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
                else crc >>= 1;
        }
    }
    return crc;
}

void InitMysys(void)
{

  encoder_absolute_angle_new=0;
  encoder_absolute_angle_old=0;
  mechanical_angle=0;
  mechanical_turns=0;
  angle_error=0;

  angle_target = 240.0f;
  angle_kp = 0.5f;

  uq_limit = 500.0f;

  counter_loop_foc=0;
  counter_loop_control=0;

  GPIOB->BSRR=1<<1;//enable DRV8311 to enable intrlnal current sensor

	MyADCInit();
	TIM1->CCR4=995;//Enable TIM1 CH4 for ADC trigger

	/* Initialise every dependency used by the fast loop before TIM1 can enter it.
	 * EncoderInit() prepares SPI1 plus both DMA2 channels; starting TIM1 earlier
	 * would leave the first FOC cycle without a DMA completion event. */
	MotorDriverInit();
	EncoderInit();
	(void)EncoderPrimeDmaRead();

	//Enable TIM1 channels for PWM generate
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  

	HAL_Delay(20);
	MyADCZeroCal();

  /* TIM1 is already running steadily for PWM/ADC. Keep it running and align
     UIE to a natural repetition update. Stopping a center-aligned timer and
     forcing CNT=0 can preserve an internal down-count direction and create a
     first IRQ only a few hundred cycles before the next underflow. */
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
  uint32_t tim1_sync_timeout = 100000U;
  while ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) == RESET) &&
         (--tim1_sync_timeout != 0U)) {
  }
  if (tim1_sync_timeout == 0U) {
    fast_loop_sync_timeout_count++;
  }
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  (void)htim1.Instance->SR;
  __DSB();
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  HAL_Delay(300); 
  /* Seed erased/invalid storage with the CAN default. An old page is migrated
     in RAM; its marker is persisted by the next explicit configuration save,
     so boot does not trigger an unsolicited Flash erase. */
  comm_type = COMM_TYPE_CAN;
  flash_data[FLASH_DEFAULTS_VERSION_INDEX] = FLASH_DEFAULTS_CAN_V1;
  init_flash_data();
  if (flash_data[FLASH_DEFAULTS_VERSION_INDEX] != FLASH_DEFAULTS_CAN_V1) {
    comm_type = COMM_TYPE_CAN;
    flash_data[FLASH_DEFAULTS_VERSION_INDEX] = FLASH_DEFAULTS_CAN_V1;
  }
  u8g2Init(&u8g2);  
  if (!HAL_GPIO_ReadPin(SYS_SW_GPIO_Port, SYS_SW_Pin)) {
    /*
     * Button-held boot is the explicit local setup path. A replacement
     * absolute encoder has a different electrical zero, so align it before
     * the SmartKnob menu enables FOC.  Do not write Flash during the early
     * startup path: an interrupted power-up must never affect the next boot.
     */
    MotorDriverSetMode(MDRV_MODE_ENC_CAL);
    while (IsMotorDriverEncCalBusy()) {
      /* Calibration advances in the TIM1-triggered DMA2 RX fast-loop ISR. */
    }
    angle_cal_offset = GetMotorDriverEncCalOffset();
    MotorDriverSetAngleOffset(angle_cal_offset);
    /* Let the FOC loop sample the encoder with the new offset first. */
    HAL_Delay(5);
    rebase_encoder_after_calibration();
    u8g2_disp_menu_init();
    u8g2_disp_menu_update();
  }  
  init_pid();
  if (comm_type == COMM_TYPE_I2C) {
    user_i2c_init();
    i2c1_it_enable();
  }
  else if (comm_type == COMM_TYPE_CAN) {
    // hard_uart_begin();
    user_fdcan_init();
  }
  else if (comm_type == COMM_TYPE_CAN_I2C) {
    user_i2c_init();
    I2C1_Start();
    user_fdcan_init();
  }

  u8g2_disp_init();
}


void LoopMysys(void)
{
    while (1) {
      LoopMysysOnce();
    }
}

void LoopMysysOnce(void)
{
        i2c_timeout_counter = 0;
        if (i2c_stop_timeout_flag) {
          if (i2c_stop_timeout_delay < HAL_GetTick()) {
            i2c_stop_timeout_counter++;
            i2c_stop_timeout_delay = HAL_GetTick() + 10;
          }
        }
        if (i2c_stop_timeout_counter > 50) {
          LL_I2C_DeInit(I2C1);
          LL_I2C_DisableAutoEndMode(I2C1);
          LL_I2C_Disable(I2C1);
          LL_I2C_DisableIT_ADDR(I2C1);     
          user_i2c_init();    
          i2c1_it_enable();
          HAL_Delay(500);
        }       
        button_update();
        u8g2_disp_update_mode();
        u8g2_disp_update_page();
        u8g2_disp_update_comm();
        
        if (my_button.was_click) {
          dis_show_flag++;
          if (dis_show_flag >= DIS_MAX)
            dis_show_flag = DIS_INFO;
          last_dis_show_flag = dis_show_flag;
          my_button.was_click = 0;
        }
        if (my_button.is_longlongpressed) {
          if (mode_switch_flag) {
            App_PostControlCommand(APP_CONTROL_COMMAND_CYCLE_MODE, 0);
            my_button.is_longlongpressed = 0;
          }
        }
        //get input voltage
        if(ph_crrent_lpf<0)
        {
          disp_ph_current = (uint16_t)(-ph_crrent_lpf);
        }
        else
        {
          disp_ph_current = (uint16_t)(ph_crrent_lpf);
        }

        if (over_vol_flag) {
          dis_show_flag = DIS_OVP;
        }
        else if (err_stalled_flag) {
          dis_show_flag = DIS_STALL;
        }        
        else if (over_value_flag) {
          dis_show_flag = DIS_OVER_VALUE;
        }        
        else {
          dis_show_flag = last_dis_show_flag;
        }
        if (rgb_color_buffer_index && rgb_show_mode) {
          uint32_t rgb_show_index = rgb_color_buffer_index;
          for (uint32_t i = 0; i < rgb_show_index; i++) {
            neopixel_set_color(0, rgb_color_buffer[i]);
            neopixel_set_color(1, rgb_color_buffer[i]);
            ws2812_show();
          }
          rgb_color_buffer_index = 0;
        }         
        switch (dis_show_flag)
        {
        case DIS_CHAR:
          u8g2_disp_char();
          break;
        case DIS_GRAPHY:
          u8g2_disp_all();
          break;
        case DIS_INFO:
          u8g2_disp_info();
          break;
        case DIS_PID:
          u8g2_disp_pid();
          break;
        case DIS_OVP:
          u8g2_disp_ovp();
          break;
        case DIS_STALL:
          u8g2_disp_stall();
          break;          
        case DIS_OVER_VALUE:
          u8g2_disp_over_value();
          break;            
        
        default:
          break;
        }
        ws2812_flash();
        
        if (act_delay < HAL_GetTick()) {
          running_index++;
          if (running_index > 1)
            running_index = 0;
          if (status_flag) {
            status_flag = 0;
          }
          else {
            status_flag = 1;
          }
          act_delay = HAL_GetTick() + 1000;
        }
}

void MysysStorageOnce(void)
{
  /* Flash operations may stall instruction fetch.  Defer them until the
     current-control output is no longer running. */
  if (flash_data_write_back_flag && sys_status != SYS_RUNNING) {
    flash_data_write_back();
    flash_data_write_back_flag = 0;
  }

}

void Loop_FOC(void)
{
  GPIOB->BSRR=GPIO_PIN_9;
  MotorDriverProcess();
  MyAdcProcess();
  FastSensorPublishFromISR(MotorDriverGetEncoderRaw(),
                           MotorDriverGetMechanicalAngle(),
                           MyAdcGetVal(1, 4),
                           MotorDriverGetPhaseCurrentReal(),
                           internal_temp_raw);
  GPIOB->BRR=GPIO_PIN_9;

}

void Loop_Control(void)
{
  FastSensorSnapshot latest_sensor_snapshot;
  if (FastSensorRead(&latest_sensor_snapshot)) {
    control_sensor_snapshot = latest_sensor_snapshot;
  }

  encoder_absolute_angle_old = encoder_absolute_angle_new;
  encoder_absolute_angle_new = control_sensor_snapshot.mechanical_angle_tenths/10.0f;

  if(encoder_absolute_angle_new<90.0f && encoder_absolute_angle_old>180.0f )
  {
    mechanical_turns += 1.0f;
  }

  if(encoder_absolute_angle_new>180.0f && encoder_absolute_angle_old<90.0f )
  {
    mechanical_turns -= 1.0f;
  }

    mechanical_angle =  (360.0f * mechanical_turns) + encoder_absolute_angle_new;

    mechanical_rad =  mechanical_angle * PI / 180.0f;

    //lpfdata += (1.0 / (1.0 + 1.0/(2.0f * 3.14f *T*fc)))*(rawdata - lpfdata );
    //lpfdata ： 滤波后的数据。
    //rawdata ： 滤波前的原始数据。
    //T： 数据的采样频率的倒数，即采样周期，单位是秒。
    //fc : 截止频率。截止频率就是超过该频率的数据（噪声）都被过滤掉，只保留低于该截止频率的数据。

    ph_current_rt = control_sensor_snapshot.phase_current_ma;
    ph_crrent_lpf += control_filter_alpha * (ph_current_rt - ph_crrent_lpf);

    speed_encoder_update_from_angle(control_sensor_snapshot.encoder_raw);

    diff_encoder_value = speed_encoder_value_t.encoder_value - speed_encoder_value_t.last_encoder_value;
    diff_encoder_value_lpf += speed_filter_alpha * (diff_encoder_value - diff_encoder_value_lpf);

    rpm_rps_count_temp = diff_encoder_value_lpf / 16383.0f;
    motor_rpm = rpm_rps_count_temp * encoder_counts_to_rpm;
    motor_rps = rpm_rps_count_temp * encoder_counts_to_rps;
    speed_encoder_value_t.last_encoder_value = speed_encoder_value_t.encoder_value;  

    //get input voltage
    vol_input = (control_sensor_snapshot.bus_voltage_adc*330*6.4545454545f)/4095;//adc1_in4, e.g. 1036 = 10.36v
    vol_lpf += control_filter_alpha * (vol_input - vol_lpf);
    temp_lpf += control_filter_alpha * ((float)control_sensor_snapshot.internal_temp_raw - temp_lpf);
    internal_temp = (int32_t)temp_lpf;

    const uint32_t protection_now_ms = HAL_GetTick();
    if (!over_vol_flag) {
      if (vol_lpf > OVER_VOLTAGE_TRIP_CENTIVOLTS) {
        over_vol_flag = 1;
        error_code |= ERR_OVER_VOLTAGE;
        over_vol_protect_auto_flag = 0U;
        MotorDriverSetCurrentReal(0.0f);
        MotorDriverSetMode(MDRV_MODE_OFF);
      }
      else {
        service_over_voltage_recovery(protection_now_ms);
      }
    }
    else {
      if (vol_lpf <= OVER_VOLTAGE_RELEASE_CENTIVOLTS) {
        over_vol_flag = 0;
        error_code &= ~ERR_OVER_VOLTAGE;
        sys_status = SYS_STANDBY;
        schedule_over_voltage_recovery(protection_now_ms);
      }
      else {
        over_vol_flag = 1;
        error_code |= ERR_OVER_VOLTAGE;
        over_vol_protect_auto_flag = 0U;
      }      
    }
    if (motor_overvalue_protection_flag) {
      int32_t mechanical_angle_int = mechanical_angle * 100;
      if (abs(mechanical_angle_int) > MY_INT32_MAX) {
        over_value_flag = 1;
        error_code |= ERR_OVER_VALUE;
        MotorDriverSetMode(MDRV_MODE_OFF);      
      }
      else {
        if (over_value_flag) {
          over_value_flag = 0;
          error_code &= ~ERR_OVER_VALUE;
          sys_status = SYS_STANDBY;
        }
      }
    }
    else {
      if (over_value_flag) {
        over_value_flag = 0;
        error_code &= ~ERR_OVER_VALUE;
        sys_status = SYS_STANDBY;
      }
    }    
}

float avg_filter(float *data, int len)
{
	float sum = 0;
	float min = data[0];
	float max = data[0];
	for (int i = 0; i < len; i++) {
		if (data[i] < min) {
			min = data[i];
		}
		if (data[i] > max) {
			max = data[i];
		}
    sum += data[i];
	}

	sum -= min;
	sum -= max;

	return sum / (len - 2);
}

void Rpm_Count_100us(void)
{
  speed_encoder_update();

  if (speed_encoder_value_t.encoder_value != speed_encoder_value_t.last_encoder_value) {
    diff_encoder_value = speed_encoder_value_t.encoder_value - speed_encoder_value_t.last_encoder_value;
    diff_encoder_value_lpf += (1.0f / (1.0f + 1.0f/(2.0f * 3.14f *0.00017857142857f*2.0f)))*(diff_encoder_value - diff_encoder_value_lpf );    
    motor_rpm = diff_encoder_value_lpf / 16383.0f * 336000;
    speed_encoder_value_t.last_encoder_value = speed_encoder_value_t.encoder_value;
  }
  else {
    motor_rpm = 0;
  }
}

void MysysCycleMode(void)
{
  if (!mode_switch_flag) {
    return;
  }

  motor_mode++;
  if (motor_mode >= MODE_MAX) {
    motor_mode = MODE_SPEED;
  }
  if (motor_mode == MODE_DIAL) {
    init_smart_knob();
  }
}

static void MysysRunModeController(void)
{
  switch (motor_mode)
  {
  case MODE_SPEED:
    if (sys_status == SYS_RUNNING) {
      if (motor_stall_protection_flag) {
        speed_err_value = abs((int32_t)(speed_err_rate * pid_ctrl_speed_t.setpoint));
        speed_err_timeout = 3;
      }
      else {
        speed_err_value = 0;
        speed_err_timeout = 0;
      }
      speed_pid();
    }
    break;
  case MODE_POS:
    if (sys_status == SYS_RUNNING) {
      if (motor_stall_protection_flag) {
        if (abs((int32_t)pid_ctrl_pos_t.setpoint) > 10)
          pos_err_value = 10;
        else
          pos_err_value = abs((int32_t)(pos_err_rate * pid_ctrl_pos_t.setpoint));
        pos_err_timeout = 3;
      }
      else {
        pos_err_value = 0;
        pos_err_timeout = 0;
      }
      pos_pid();
    }
    break;
  case MODE_SPEED_ERR_PROTECT:
    if (HAL_GetTick() - speed_err_auto_counter > 2000 && speed_err_count_flag) {
      if (!err_stalled_flag && err_recover_try_max && speed_err_recover_try_counter <= err_recover_try_max - 1) {
        speed_err_recover_try_counter++;
        MotorDriverSetMode(MDRV_MODE_RUN);
        motor_mode = MODE_SPEED;
        speed_err_count_flag = 2;
      }
      else {
        err_stalled_flag = 1;
      }
    }
    break;
  case MODE_POS_ERR_PROTECT:
    if (HAL_GetTick() - pos_err_auto_counter > 2000 && pos_err_count_flag) {
      if (!err_stalled_flag && err_recover_try_max && pos_err_recover_try_counter <= err_recover_try_max - 1) {
        pos_err_recover_try_counter++;
        MotorDriverSetMode(MDRV_MODE_RUN);
        motor_mode = MODE_POS;
        pos_err_count_flag = 2;
      }
      else {
        err_stalled_flag = 1;
      }
    }
    break;
  case MODE_CURRENT:
    if (sys_status == SYS_RUNNING) {
      current_point_float = (float)current_point / 100.0f;
      if (current_point_float != 0.0f && fabsf(ph_crrent_lpf / current_point_float) < 0.90f) {
        rgb_flash_slow = 1;
      }
      else {
        rgb_flash_slow = 0;
      }
    }
    break;
  case MODE_DIAL:
    if (sys_status == SYS_RUNNING || sys_status == SYS_STANDBY)
      handle_smart_knob();
    break;
  default:
    break;
  }
}

void MysysControlTaskBegin(void)
{
  uint32_t primask = __get_PRIMASK();

  /* Stop the ISR-side outer scheduler before changing its discrete timing. */
  __disable_irq();
  control_task_active = 1U;
  if (primask == 0U) {
    __enable_irq();
  }

  control_filter_alpha =
      1.0f / (1.0f + 1.0f/(2.0f * PI * (1.0f / CONTROL_TASK_RATE_HZ) * 2.0f));
  speed_filter_alpha = control_filter_alpha;
  encoder_counts_to_rpm = 60.0f * CONTROL_TASK_RATE_HZ;
  encoder_counts_to_rps = 2.0f * PI * CONTROL_TASK_RATE_HZ;

  /* The stored gains are legacy discrete-step coefficients. */
  PIDDiscreteTimeScaleSet(&pid_ctrl_speed_t, LEGACY_MODE_RATE_HZ / CONTROL_TASK_RATE_HZ);
  PIDDiscreteTimeScaleSet(&pid_ctrl_pos_t, LEGACY_MODE_RATE_HZ / CONTROL_TASK_RATE_HZ);
  smart_knob_set_update_rate(CONTROL_TASK_RATE_HZ);
  if (motor_mode == MODE_DIAL) {
    init_smart_knob();
  }
}

void MysysControlStep(void)
{
  Loop_Control();
  MysysRunModeController();
}

static void MysysRunLegacyOuterSchedulerFromISR(void)
{
  /* During early hardware initialisation there is no ControlTask yet. Keep
     the legacy outer scheduler alive only until the task takes ownership. */
  if (!control_task_active) {
    counter_loop_control += 3U;
    if (counter_loop_control >= 10U) {
      counter_loop_control -= 10U;
      Loop_Control();
    }

    pid_compute_counter += 3U;
    if (pid_compute_counter >= 11U) {
      pid_compute_counter -= 11U;
      MysysRunModeController();
    }
  }
}

static void MysysFinishPendingFastLoopFromISR(uint8_t fresh_sample,
                                               uint32_t active_start_cycles,
                                               uint8_t record_dma_isr)
{
  if (fast_cycle_pending == 0U) {
    EncoderRecordUnexpectedFastCompletionFromISR();
    if (record_dma_isr != 0U) {
      RuntimeMetricsRecordEncoderDmaIsr(active_start_cycles);
    }
    return;
  }

  fast_cycle_pending = 0U;
  if (fresh_sample != 0U) {
    RuntimeMetricsRecordEncoderDma(fast_cycle_start_cycles);
  } else {
    EncoderRecordStaleSampleFromISR();
  }

  Loop_FOC();
  RuntimeMetricsRecordFocCpu(
      fast_cycle_start_active_cycles +
      RuntimeMetricsElapsedSince(active_start_cycles));
  RuntimeMetricsRecordFoc(fast_cycle_start_cycles);
  MysysRunLegacyOuterSchedulerFromISR();
  if (record_dma_isr != 0U) {
    RuntimeMetricsRecordEncoderDmaIsr(active_start_cycles);
  }
}


void MysysFastLoopISR(void)
{
  const uint32_t active_start_cycles = RuntimeMetricsCycleNow();
  const uint32_t timer_count = htim1.Instance->CNT;
  const uint32_t timer_period = htim1.Instance->ARR;
  const uint32_t ticks_to_next_boundary =
      ((htim1.Instance->CR1 & TIM_CR1_DIR) != 0U)
          ? timer_count
          : (timer_period - timer_count);
  const uint32_t next_update_already_pending =
      NVIC_GetPendingIRQ(TIM1_UP_TIM16_IRQn);

  /* A priority-0 startup compatibility step can delay a pending TIM1 update
     until the next center-aligned boundary is already close. Starting a
     32-bit SPI frame there would overlap that boundary. Drop only this late
     event; a normal update enters while moving away from a boundary and has
     roughly the full ARR count available. */
  if ((next_update_already_pending != 0U) ||
      (ticks_to_next_boundary < FAST_LOOP_MIN_START_MARGIN_TIM1_TICKS)) {
    fast_loop_late_start_count++;
    RuntimeMetricsRecordTim1Isr(active_start_cycles);
    return;
  }

  fast_cycle_start_cycles = active_start_cycles;
  fast_cycle_pending = 1U;
  MotorDriverPrepareCycleFromISR();

  if (EncoderStartDmaReadFromISR()) {
    fast_cycle_start_active_cycles =
        RuntimeMetricsElapsedSince(active_start_cycles);
  } else {
    /* DMA startup or overlap recovery uses the last valid encoder sample and
       still advances the current loop exactly once for this TIM1 period. */
    fast_cycle_start_active_cycles = 0U;
    MysysFinishPendingFastLoopFromISR(0U, active_start_cycles, 0U);
  }
  RuntimeMetricsRecordTim1Isr(active_start_cycles);
}

void MysysFastLoopOnEncoderSampleFromISR(uint8_t fresh_sample,
                                         uint32_t irq_start_cycles)
{
  MysysFinishPendingFastLoopFromISR(fresh_sample, irq_start_cycles, 1U);
}
