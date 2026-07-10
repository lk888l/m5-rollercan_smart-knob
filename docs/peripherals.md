# 外设与引脚

## MCU 与时钟

目标芯片由工程宏定义为 `STM32G431xx`。`SystemClock_Config()` 使用 HSI 作为 PLL 输入：

- `PLLM = 1`
- `PLLN = 21`
- `PLLP/Q/R = 2`
- AHB/APB1/APB2 均不分频
- 电压档位 `PWR_REGULATOR_VOLTAGE_SCALE1_BOOST`

## GPIO

主要引脚定义来自 `Core/Inc/main.h` 和 `Core/Src/gpio.c`。

| 引脚 | 方向/复用 | 用途 |
| --- | --- | --- |
| PA0/PA1/PA2/PA3 | ADC analog | 三相电流、输入电压等 ADC 通道 |
| PA4 | GPIO output | 编码器 SPI CS，`tle5012b.c` 直接操作 |
| PA5/PA6/PA7 | SPI1 AF5 | TLE5012B SPI SCK/MISO/MOSI |
| PA8/PA9/PA10 | TIM1 AF6 | 三相 PWM 输出 |
| PA11/PA12 | FDCAN1 AF9 | CAN RX/TX |
| PA15 | I2C1 AF4 OD | I2C SCL |
| PB7 | I2C1 AF4 OD | I2C SDA，启用 FastModePlus |
| PB1 | GPIO output | DRV8311 内部电流传感器相关使能 |
| PB2 | GPIO output | 电机驱动输出使能 |
| PB4 | GPIO output | CAN STB |
| PB5 | TIM3_CH2 AF2 | SK6812/WS2812 PWM-DMA 输出 |
| PB9 | GPIO output | 调试脉冲，`Loop_FOC()` 进出拉高/拉低 |
| PB11/PB12/PB13/PB14/PB15 | GPIO output | OLED RST/CS/SCK/DC/MOSI，软件 SPI |
| PC6 | GPIO input pull-up | 系统按键 `SYS_SW` |

## TIM1

`TIM1` 是电机控制核心定时器。

配置：

- Prescaler：`3-1`
- CounterMode：Center-aligned 1
- Period：`1000-1`
- CH1/CH2/CH3：三相 PWM
- CH4：PWM2，用作 ADC 触发源
- MasterOutputTrigger：`TIM_TRGO_OC4REF`
- Update IRQ：优先级 0

运行方式：

- `InitMysys()` 启动 CH1/CH2/CH3 PWM。
- `TIM1->CCR4=995` 设置 ADC 采样触发点。
- `__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE)` 打开更新中断。
- `MotorDriverProcess()` 根据 SVM 结果写 `TIM1->CCR1/CCR2/CCR3`。

## ADC1 + DMA

ADC1 配置：

- 12-bit，右对齐。
- 扫描 6 个 regular conversion。
- 外部触发：`ADC_EXTERNALTRIG_T1_TRGO` 上升沿。
- DMA circular。
- DMA1 Channel1，低优先级。

通道顺序：

| Rank | 通道 | 代码用途 |
| --- | --- | --- |
| 1 | ADC_CHANNEL_2 | `adc1_convbuf[0]`，相电流 A |
| 2 | ADC_CHANNEL_3 | `adc1_convbuf[1]`，相电流 B |
| 3 | ADC_CHANNEL_4 | `adc1_convbuf[2]`，相电流 C |
| 4 | ADC_CHANNEL_1 | `adc1_convbuf[3]`，输入电压 |
| 5 | TEMPSENSOR | `adc1_convbuf[4]`，内部温度 |
| 6 | VREFINT | `adc1_convbuf[5]`，温度计算参考 |

`MyADCInit()` 启动 ADC 校准和 DMA；`MyADCZeroCal()` 记录三相电流零点；`MyAdcProcess()` 更新 `ia/ib/ic` 和温度原始值。

## SPI1

SPI1 用于 TLE5012B 编码器：

- Master。
- 16-bit data。
- CPOL high，CPHA second edge。
- NSS software。
- Prescaler 16。

`tle5012b.c` 直接访问 SPI 寄存器发送两个 16-bit word：

```text
Tx: 0x8021, 0xffff
Rx: 第二个 word 的低 15 位换算为角度
```

`MotorDriverProcess()` 每次 FOC 处理都会调用 `EncoderGetAngle()` 读取当前角度。

## I2C1

I2C1 有两种角色：

| 角色 | 入口 | 说明 |
| --- | --- | --- |
| 本机从机 | `user_i2c_init()` + `i2c1_it_enable()` | 以 `i2c_address[0]` 作为 7-bit 地址，响应寄存器协议 |
| CAN 桥接主机 | `I2C1_Start()` + `I2C_Read_Bytes/I2C_Write_Bytes` | CAN 命令临时禁用从机后主动访问外部 I2C 设备 |

默认 CubeMX 初始化里地址为 `0x64`，业务初始化会按 Flash 中保存的地址重配。

## FDCAN1

FDCAN1 在业务初始化中使用 `user_fdcan_init()`：

- Classic CAN frame。
- Extended ID。
- Normal mode。
- Auto retransmission disabled。
- filter 按 `can_id` 和 mask `0xFF` 过滤。
- RX FIFO0 new message interrupt。

波特率通过 `bps_index` 选择：

| `bps_index` | `NominalPrescaler` | UI 文案 |
| --- | --- | --- |
| 0 | 1 | 1 Mbps |
| 1 | 2 | 500 Kbps |
| 2 | 8 | 125 Kbps |

## TIM3 + DMA

TIM3 CH2 用于 SK6812/WS2812 LED：

- Period：209。
- DMA1 Channel3，memory-to-peripheral，halfword。
- `ws2812_show()` 将 GRB bit 展开成 PWM 占空比序列，并通过 `HAL_TIM_PWM_Start_DMA()` 发送。
- `HAL_TIM_PWM_PulseFinishedCallback()` 停止 TIM3 CH2，确保发送结束后输出停止。

## OLED

OLED 是 64x48 SSD1306，使用 U8g2 软件 4-wire SPI：

- PB11：RST
- PB12：CS，代码注释表示默认接地，实际 GPIO 已配置
- PB13：SCK
- PB14：DC
- PB15：MOSI

初始化函数：

```c
u8g2_Setup_ssd1306_64x48_er_f(&u8g2, U8G2_R2, u8x8_byte_4wire_sw_spi, u8x8_stm32_gpio_and_delay);
```
