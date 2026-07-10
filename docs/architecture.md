# 系统架构

## 分层

ROLLERCAN 固件可以按四层理解：

| 层级 | 目录/文件 | 职责 |
| --- | --- | --- |
| 启动与芯片支持 | `startup_stm32g431xx.s`, `system_stm32g4xx.c`, `STM32G431XX_FLASH.ld` | 向量表、系统时钟基础、链接布局 |
| CubeMX 外设层 | `Core/Src/*.c`, `Core/Inc/*.h`, `cmake/stm32cubemx` | GPIO、ADC、DMA、TIM、SPI、I2C、FDCAN 初始化和 IRQ 入口 |
| RTOS 应用层 | `App/src/app_rtos.cpp`, `App/inc/app_rtos.h` | C++17 静态任务、控制邮箱和 C/C++ 边界 |
| 业务控制层 | `MyFile/src/*.c`, `MyFile/inc/*.h`, `Core/Src/flash.c`, `Core/Src/i2c_ex.c` | 电机控制、通信协议、显示菜单、灯效、参数保存 |
| 第三方库 | `Middlewares/Third_Party/FreeRTOS`, `U8g2_lib`, CMSIS/HAL/LL | 调度、OLED 绘图、ARM 数学函数和 STM32 驱动 |

## 运行上下文

工程使用静态 FreeRTOS 任务和少量中断上下文：

| 上下文 | 入口 | 主要动作 |
| --- | --- | --- |
| ControlTask | `App/src/app_rtos.cpp` | 1 kHz 外环、保护、速度/位置 PID、SmartKnob 和本机命令执行 |
| CommunicationTask | `App/src/app_rtos.cpp` | FDCAN FIFO、帧解码、回复发送和 CAN-I2C 桥接 |
| MaintenanceTask | `App/src/app_rtos.cpp` -> `LoopMysysOnce()` | I2C 超时恢复、按钮扫描、OLED 刷新、WS2812 灯效 |
| StorageTask | `App/src/app_rtos.cpp` -> `MysysStorageOnce()` | 安全状态下的 Flash 写回 |
| TIM1 更新中断 | `TIM1_UP_TIM16_IRQHandler()` -> `MysysFastLoopISR()` | 约 18.67 kHz FOC 和 ADC 数据处理 |
| I2C1 事件中断 | `I2C1_EV_IRQHandler()` -> `i2c1_event_irq_handler()` | I2C 从机收发、事务完成回调到 `Slave_Complete_Callback()` |
| I2C1 错误中断 | `I2C1_ER_IRQHandler()` -> `i2c1_error_irq_handler()` | 复位并重新初始化 I2C 从机 |
| FDCAN 接收中断 | `FDCAN1_IT0_IRQHandler()` -> HAL callback | 只通知 CommunicationTask 并记录 FIFO loss |
| DMA 中断 | TIM3 CH2 DMA 等 | WS2812 PWM-DMA 发送完成；ADC DMA HT/TC IRQ 已关闭 |

## 核心数据流

```text
ADC1 DMA -> adc1_convbuf
  -> MyAdcProcess
  -> ia/ib/ic, internal_temp_raw
  -> MotorDriverProcess
  -> FastSensorSnapshot
  -> Loop_Control

TLE5012B SPI -> EncoderGetAngle
  -> MotorDriverProcess
  -> angle_corrected / eangle_get
  -> FOC Park/Clarke/SVM
  -> TIM1 CCR1/CCR2/CCR3

CAN 命令 -> FDCAN ISR notification -> CommunicationTask
  -> 本机命令静态队列 -> ControlTask
  -> 回复静态队列 -> CommunicationTask -> FDCAN TX
按键命令 -> 静态控制邮箱 -> ControlTask
  -> motor_output / motor_mode / setpoint / PID / protection flags
  -> MotorDriverSetMode / MotorDriverSetCurrentReal
  -> FastControlCommandSnapshot
  -> TIM1 ISR 周期边界应用驱动模式和电流目标

系统状态 -> u8g2_disp_fun / ws2812
  -> OLED 页面、通信闪烁、运行模式图标、RGB 灯效
```

调度器启动后，CAN 和按键路径遵守 ControlTask 单写者模型。I2C 从机完成回调仍直接写控制状态，是当前按计划暂未迁移的边界。

`App/src/fast_control_link.cpp` 使用奇偶 sequence 和内存屏障传递快照。命令发布期间只关闭极短时间的全局中断，FOC ISR 不等待任务完成；传感器读取若恰好被 FOC 更新打断，会有限次重读并在失败时保留上一份有效快照。

## 重要全局状态

| 变量 | 定义位置 | 含义 |
| --- | --- | --- |
| `sys_status` | `MyFile/src/mysys.c` | `SYS_STANDBY`、`SYS_RUNNING`、`SYS_ERROR` |
| `motor_mode` | `MyFile/src/mysys.c` | 速度、位置、电流、Dial，以及保护态 |
| `motor_output` | `MyFile/src/mysys.c` | 外部命令控制的电机开关 |
| `speed_point`, `pos_point`, `current_point` | `MyFile/src/mysys.c` | 通信协议写入的目标值，整数缩放后进入控制器 |
| `pid_ctrl_speed_t`, `pid_ctrl_pos_t` | `MyFile/src/mysys.c` | 速度环和位置环 PID 对象 |
| `mechanical_angle`, `mechanical_rad` | `MyFile/src/mysys.c` | 多圈机械角度和弧度 |
| `ph_crrent_lpf`, `vol_lpf` | `MyFile/src/mysys.c` | 相电流和母线电压低通值 |
| `can_id`, `bps_index` | `Core/Src/fdcan.c`, `mysys.c` | CAN 节点 ID 和波特率索引 |
| `i2c_address[0]` | `Core/Src/main.c` | 本机 I2C 7-bit 地址 |
| `comm_type` | `MyFile/src/u8g2_disp_fun.c` | I2C、CAN、CAN->I2C 三种通信模式 |

## CubeMX 与手写代码边界

`Core/Src/*.c` 大部分是 CubeMX 生成文件，但本工程在几个文件的 USER CODE 区加入了业务逻辑：

- `main.c`：启动前 SRAM 向量表重映射、Flash 参数初始化/写回、I2C 从机协议分发。
- `stm32g4xx_it.c`：中断入口中调用手写 helper。
- `fdcan.c`：CAN 协议解析和桥接逻辑在 USER CODE 区。
- `i2c.c`：除 CubeMX 初始化外，还提供 I2C 主机读写 helper。
- `flash.c`, `i2c_ex.c`：手写扩展模块，被根 CMake 手动加入 target。

维护时应避免把公开 IRQ 入口重复定义到 `MyFile` 中；公开入口留在 `stm32g4xx_it.c`，业务逻辑放 helper。
