# 维护注意事项

## CubeMX 再生成

本工程已使用 CubeMX 6.17.0、STM32CubeG4 1.6.3 实际执行连续再生成验证。`ROLLERCAN.ioc` 必须保持：

- `ProjectManager.KeepUserCode=true`，保留 IRQ 和外设初始化中的用户区。
- `ProjectManager.DeletePrevious=false`，减少生成器清理无关输出的范围。CubeMX 仍会重建
  `Middlewares`，因此手工 FreeRTOS kernel/port 固定放在生成器管理范围之外的
  `ThirdParty/FreeRTOS-Kernel`。

CubeMX 可能重新生成 `Core/Src/stm32g4xx_it.c`、外设初始化文件、默认
`STM32G431XX_FLASH.ld` 和 `cmake/stm32cubemx/CMakeLists.txt`。实际应用固定使用
`linker/ROLLERCAN_APP.ld`，因此默认链接脚本被恢复为 `0x08000000/128K` 不影响 CMake 构建。
再生成后重点检查：

- `TIM1_UP_TIM16_IRQHandler()` 只能由 `stm32g4xx_it.c` 定义一次；其 `IRQn 0`
  用户区调用 `MysysFastLoopISR()` 后必须 `return`，不能落入 CubeMX 生成的
  `HAL_TIM_IRQHandler()`。
- DMA2 Channel 1/2 的 `IRQn 0` 用户区是否仍分别调用
  `EncoderHandleDmaRxIRQ()` / `EncoderHandleDmaTxIRQ()` 并提前 `return`。
- `USER CODE BEGIN PM` 中的 `SVC_Handler` / `PendSV_Handler` 重命名宏是否仍存在，
  确保实际中断向量只由 FreeRTOS port 提供。
- `I2C1_EV_IRQHandler()` 是否仍调用 `i2c1_event_irq_handler()`。
- `I2C1_ER_IRQHandler()` 是否仍调用 `i2c1_error_irq_handler()`。
- 根 `CMakeLists.txt` 是否仍加入 `App/src/app_rtos.cpp`、
  `ThirdParty/FreeRTOS-Kernel`、`Core/Src/flash.c`、`Core/Src/i2c_ex.c`、
  `MyFile/src/*.c` 和裁剪后的 U8g2 源。
- `cmake/gcc-arm-none-eabi.cmake` 和 `cmake/starm-clang.cmake` 是否仍引用
  `linker/ROLLERCAN_APP.ld`。

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

TIM1 update ISR 只启动编码器 DMA；DMA2 RX 完成 ISR 提交角度并接续 FOC。1 kHz 外环、保护和 SmartKnob 正常运行时由 ControlTask 负责。注意：

- 不要在 TIM1/DMA2 快环路径里做 Flash 擦写。
- 不要做长时间阻塞 I2C/CAN 操作。
- OLED 绘制和 WS2812 发送应留在 MaintenanceTask。
- 增加滤波或计算时要考虑 FOC 频率和中断耗时。

FDCAN callback 只通知 CommunicationTask；帧读取、回复发送和 CAN-I2C 桥接都在该任务中执行。I2C1 从机回调仍是尚未完全任务化的例外，其 IRQ 优先级必须低于 TIM1/DMA2 快环。

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
- CAN 收不到：确认 `can_id` filter、固定 1/5 Mbit/s 时序、扩展 ID、CAN-FD 与 BRS。
- I2C 读写异常：确认 7-bit 地址是否左移、从机中断是否启用、STOP 是否被触发。
- UI 卡住菜单：检查 LocalUi 非阻塞状态、私有导航模式和 `MysysHostConnected()`；上位机接管时菜单应立即取消并回仪表盘。
