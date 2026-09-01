# 系统架构

## 分层

ROLLERCAN 固件可以按四层理解：

| 层级 | 目录/文件 | 职责 |
| --- | --- | --- |
| 启动与芯片支持 | `startup_stm32g431xx.s`, `system_stm32g4xx.c`, `STM32G431XX_FLASH.ld` | 向量表、系统时钟基础、链接布局 |
| CubeMX 外设层 | `Core/Src/*.c`, `Core/Inc/*.h`, `cmake/stm32cubemx` | GPIO、ADC、DMA、TIM、SPI、I2C、FDCAN 初始化和 IRQ 入口 |
| RTOS 应用层 | `App/src/app_rtos.cpp`, `App/inc/app_rtos.h` | C++17 静态任务、控制邮箱和 C/C++ 边界 |
| 业务控制层 | `MyFile/src/*.c`, `MyFile/inc/*.h`, `Core/Src/flash.c`, `Core/Src/i2c_ex.c` | 电机控制、本地 UI、主机控制权、通信协议、灯效、完整快照保存 |
| 第三方库 | `ThirdParty/FreeRTOS-Kernel`, `U8g2_lib`, CMSIS/HAL/LL | 调度、OLED 绘图、ARM 数学函数和 STM32 驱动 |

## 运行上下文

工程使用静态 FreeRTOS 任务和少量中断上下文：

| 上下文 | 入口 | 主要动作 |
| --- | --- | --- |
| ControlTask | `App/src/app_rtos.cpp` | 1 kHz 外环、保护、速度/位置 PID、SmartKnob 和本机命令执行 |
| CommunicationTask | `App/src/app_rtos.cpp` | 固定 CAN 运行时创建；排空 FDCAN、转交本机命令、发送响应与遥测，桥接命令就地处理 |
| MaintenanceTask | `App/src/app_rtos.cpp` -> `LoopMysysOnce()` | 非阻塞按钮扫描、LocalUi 状态机、OLED 刷新、WS2812 灯效 |
| StorageTask | `App/src/app_rtos.cpp` -> `MysysStorageOnce()` | 安全状态下的 Flash 写回 |
| TIM1 更新中断 | `TIM1_UP_TIM16_IRQHandler()` -> `MysysFastLoopISR()` | 约 18.67 kHz 启动 TLE5012B SPI1 DMA |
| DMA2 Channel 1 中断 | `DMA2_Channel1_IRQHandler()` -> `MysysFastLoopOnEncoderSampleFromISR()` | 提交同周期角度并接续 FOC、ADC 与 PWM 更新 |
| I2C1 事件中断 | `I2C1_EV_IRQHandler()` -> `i2c1_event_irq_handler()` | I2C 从机收发、事务完成回调到 `Slave_Complete_Callback()` |
| I2C1 错误中断 | `I2C1_ER_IRQHandler()` -> `i2c1_error_irq_handler()` | 复位并重新初始化 I2C 从机 |
| FDCAN 接收中断 | `FDCAN1_IT0_IRQHandler()` -> HAL callback | 只通知 CommunicationTask 并记录 FIFO loss |
| 其他 DMA 中断 | TIM3 CH2 DMA 等 | WS2812 PWM-DMA 发送完成；ADC DMA HT/TC IRQ 已关闭 |

## 核心数据流

```text
ADC1 DMA -> adc1_convbuf
  -> MyAdcProcess
  -> ia/ib/ic, internal_temp_raw
  -> MotorDriverProcess
  -> FastSensorSnapshot
  -> Loop_Control

TIM1 update ISR -> TLE5012B SPI1 DMA2 RX/TX
  -> DMA2 RX complete ISR -> EncoderGetLatestAngle
  -> MotorDriverProcess
  -> angle_corrected / eangle_get
  -> FOC Park/Clarke/SVM
  -> TIM1 CCR1/CCR2/CCR3

PC6 按键 -> MaintenanceTask / LocalUiTask
旋钮挡位 -> LocalUi 菜单状态机
  -> 本机命令静态队列 -> ControlTask
  -> 进入/退出导航模式、应用预设、启停力反馈
  -> motor_output / motor_mode / SmartKnob profile / protection flags
  -> MotorDriverSetMode / MotorDriverSetCurrentReal
  -> FastControlCommandSnapshot
  -> TIM1 ISR 周期边界应用驱动模式和电流目标

SmartKnobRuntimeState + 系统状态 -> LocalUiTask / ws2812
  -> 圆形速度表、挡位位置、菜单、保护覆盖提示、RGB 灯效

FDCAN RX -> CommunicationTask
  -> 本机命令静态队列 -> ControlTask
  -> host_control 接管/事务/3 s 超时
  -> 与 LocalUi 共用 SmartKnob 状态
  -> 响应队列 / 主动遥测 -> CommunicationTask -> FDCAN TX
```

调度器启动后，本地 UI 和本机 CAN 命令都遵守 ControlTask 单写者模型。CommunicationTask 不直接修改模式或电机状态；CAN-I2C 桥接 `19–22` 不进入本机控制权状态机。I2C 从机业务不在当前 `COMM_TYPE_CAN` 路径启动。

`App/src/fast_control_link.cpp` 使用奇偶 sequence 和内存屏障传递快照。命令发布期间只关闭极短时间的全局中断，FOC ISR 不等待任务完成；传感器读取若恰好被 FOC 更新打断，会有限次重读并在失败时保留上一份有效快照。

## 重要全局状态

| 变量 | 定义位置 | 含义 |
| --- | --- | --- |
| `sys_status` | `MyFile/src/mysys.c` | `SYS_STANDBY`、`SYS_RUNNING`、`SYS_ERROR` |
| `motor_mode` | `MyFile/src/mysys.c` | 速度、位置、电流、Dial，以及保护态 |
| `motor_output` | `MyFile/src/mysys.c` | 本地 UI 或当前上位机控制的力反馈开关 |
| `speed_point`, `pos_point`, `current_point` | `MyFile/src/mysys.c` | 协议速度/位置/电流目标；本地默认使用 Dial |
| `pid_ctrl_speed_t`, `pid_ctrl_pos_t` | `MyFile/src/mysys.c` | 速度环和位置环 PID 对象 |
| `mechanical_angle`, `mechanical_rad` | `MyFile/src/mysys.c` | 多圈机械角度和弧度 |
| `ph_crrent_lpf`, `vol_lpf` | `MyFile/src/mysys.c` | 相电流和母线电压低通值 |
| `can_id`, `bps_index` | `Core/Src/fdcan.c`, `mysys.c` | CAN 节点 ID 和波特率索引 |
| `i2c_address[0]` | `Core/Src/main.c` | 本机 I2C 7-bit 地址 |
| `comm_type` | `MyFile/src/u8g2_disp_fun.c` | 当前运行时固定为 `COMM_TYPE_CAN` |

## CubeMX 与手写代码边界

`Core/Src/*.c` 大部分是 CubeMX 生成文件，但本工程在几个文件的 USER CODE 区加入了业务逻辑：

- `main.c`：启动前将 `SCB->VTOR` 指向 `0x08002000` 的完整应用向量表、Flash 参数初始化/写回、I2C 从机协议分发；CAN 收发器只在读取保存的节点 ID 之前短暂保持 standby，随后由 `InitMysys()` 调用 `user_fdcan_init()`。
- `stm32g4xx_it.c`：中断入口中调用手写 helper。
- `fdcan.c`：CAN 协议解析和桥接逻辑在 USER CODE 区。
- `i2c.c`：除 CubeMX 初始化外，还提供 I2C 主机读写 helper。
- `flash.c`, `i2c_ex.c`：手写扩展模块，被根 CMake 手动加入 target。

维护时应避免把公开 IRQ 入口重复定义到 `MyFile` 中；公开入口留在 `stm32g4xx_it.c`，业务逻辑放 helper。
