# 维护注意事项

## CubeMX 再生成

CubeMX 可能重新生成 `Core/Src/stm32g4xx_it.c`、外设初始化文件和 `cmake/stm32cubemx/CMakeLists.txt`。再生成后重点检查：

- `TIM1_UP_TIM16_IRQHandler()` 是否仍在 USER CODE 区调用 `MysysFastLoopISR()`，且没有再次调用 `HAL_TIM_IRQHandler()`。
- `I2C1_EV_IRQHandler()` 是否仍调用 `i2c1_event_irq_handler()`。
- `I2C1_ER_IRQHandler()` 是否仍调用 `i2c1_error_irq_handler()`。
- 根 `CMakeLists.txt` 是否仍加入 `App/src/app_rtos.cpp`、FreeRTOS kernel/port、`Core/Src/flash.c`、`Core/Src/i2c_ex.c`、`MyFile/src/*.c` 和裁剪后的 U8g2 源。

公开 IRQ 入口函数不要复制到 `MyFile` 或其他源文件中，否则链接时会出现重复定义。正确做法是：

```text
stm32g4xx_it.c        只保留公开 IRQ 入口
mysys.c/i2c_ex.c      暴露 helper 函数
IRQ USER CODE 区       调 helper
```

## U8g2 Flash 占用

U8g2 的字体、图标和不必要源文件会显著增加 Flash。当前工程已经在根 `CMakeLists.txt` 中显式列出 U8g2 源文件，避免全量 glob。

新增显示功能时建议：

1. 先复用已有字体，例如 `u8g2_font_4x6_tr`、`u8g2_font_5x7_tr`、`u8g2_font_6x10_tr`、`u8g2_font_6x13_tr`。
2. 如果只需要 ASCII，优先选择 `_tr` 或其他受限字体，而不是全字符集字体。
3. 对简单图标优先用 `u8g2_DrawLine/DrawBox/DrawPixel` 画出来。
4. 用 map/size 看实际链接体积，而不是仅看源码文件大小。

常用命令：

```powershell
arm-none-eabi-size -A build\Debug\ROLLERCAN.elf
arm-none-eabi-nm --print-size --size-sort --radix=d build\Debug\ROLLERCAN.elf
```

## 实时上下文约束

TIM1 更新中断里会运行 FOC 和控制逻辑，注意：

- 不要在 TIM1 中断路径里做 Flash 擦写。
- 不要做长时间阻塞 I2C/CAN 操作。
- OLED 绘制和 WS2812 发送应留在主循环。
- 增加滤波或计算时要考虑 FOC 频率和中断耗时。

CAN callback 当前会直接执行一些 I2C 桥接读写。若后续遇到实时性问题，可以考虑改为主循环任务队列。

## 单位和缩放

协议和内部变量混用整数缩放，改动前先确认单位：

| 变量 | 外部缩放 | 内部用途 |
| --- | --- | --- |
| `speed_point` | /100 | RPM setpoint |
| `pos_point` | /100 | degree setpoint |
| `current_point` | /100 | mA current setpoint |
| `max_speed_current` | /100 | mA output limit |
| `max_pos_current` | /100 | mA output limit |
| PID Kp/Kd | /100000 | float gain |
| PID Ki | /10000000 | float gain |
| 速度反馈 | *100 | int32 readback |
| 位置反馈 | *100 | int32 readback |
| 电流反馈 | *100 | int32 readback |

## 保护状态

错误位：

| bit | 名称 | 含义 |
| --- | --- | --- |
| 0 | `ERR_OVER_VOLTAGE` | 过压 |
| 1 | `ERR_STALLED` | 堵转 |
| 2 | `ERR_OVER_VALUE` | 位置越界 |

保护态模式：

- `MODE_SPEED_ERR_PROTECT`
- `MODE_POS_ERR_PROTECT`

保存 Flash 前会把保护态改回普通速度/位置模式，避免开机直接进入保护态。

## 调试建议

- 电机不转：先看 `sys_status`、`error_code`、`motor_output`、`motor_mode`，再看 `GPIOB PB2` 驱动使能。
- 电流异常：先确认 `MyADCZeroCal()` 时电机无电流，再检查 ADC DMA buffer 和相电流方向。
- 角度异常：先看 SPI1/TLE5012B 原始角度，再看 `angle_cal_offset` 和 `angle_corrected`。
- CAN 收不到：确认 `can_id` filter、`bps_index`、扩展帧和 Classic CAN。
- I2C 读写异常：确认 7-bit 地址是否左移、从机中断是否启用、STOP 是否被触发。
- UI 卡住菜单：菜单本身是阻塞循环，会临时进入 Dial 模式，退出后才回主 UI。
