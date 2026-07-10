# 启动与运行流程

## 上电启动

`Core/Src/main.c` 的 `main()` 是唯一应用入口。

1. `IAP_Set()` 将应用区 `0x08002000` 的向量复制到 SRAM，并 remap 到 `0x00000000`。
2. `HAL_Init()` 初始化 HAL、SysTick 和 Flash 接口。
3. `SystemClock_Config()` 使用 HSI+PLL 配置系统时钟。
4. 依次初始化 GPIO、DMA、ADC1、TIM1、SPI1、TIM3、I2C1 和 FDCAN1。
5. `InitMysys()` 完成业务和电机硬件初始化。
6. `sk6812_init(PIXEL_MAX)` 初始化 RGB 缓冲。
7. `App_StartScheduler()` 创建全静态 FreeRTOS 对象并启动调度器；正常情况下不返回。

## `InitMysys()` 的实时启动顺序

`MyFile/src/mysys.c` 的 `InitMysys()` 必须先准备 FOC 的全部依赖，再开启 TIM1 update IRQ：

1. 清零角度和控制状态。
2. `MyADCInit()` 启动 ADC 校准和 circular DMA。
3. 设置 TIM1 CH4 ADC 触发点。
4. `MotorDriverInit()` 初始化 FOC 状态。
5. `EncoderInit()` 使能 TLE5012B 使用的 SPI1。
6. 启动 TIM1 CH1/CH2/CH3 PWM。
7. 延时 20 ms 后执行 `MyADCZeroCal()`。
8. 清除 TIM1 update pending flag，然后开启 update IRQ。
9. 读取 Flash 配置并初始化 OLED；按键按住上电时可进入阻塞式本地配置菜单。
10. 初始化 PID、I2C/FDCAN 通信和启动界面。

顺序 4～8 不可随意交换。若 SPI1 启用前进入 `Loop_FOC()`，编码器传输会等不到 RXNE。当前 SPI 轮询虽然已有超时，不再永久锁死，但启动数据仍然无效。

## FreeRTOS 运行上下文

| 上下文 | 周期/触发 | 职责 |
| --- | --- | --- |
| TIM1 update ISR | 约 18.67 kHz | 只运行 `Loop_FOC()`；不调用 FreeRTOS API |
| ControlTask | 1 kHz + 控制邮箱唤醒 | `Loop_Control()`、模式控制、保护、SmartKnob 和本机命令执行 |
| CommunicationTask | FDCAN/回复唤醒 | 读取 RX FIFO、发送回复、隔离 CAN-I2C 桥接 |
| MaintenanceTask | 10 ms | I2C 超时恢复、按钮、OLED、RGB、通信慢速维护 |
| StorageTask | 20 ms | 电机安全状态下的 Flash 写回 |
| FDCAN ISR | RX new-message / message-lost | 只通知 CommunicationTask 并记录硬件 loss |
| I2C1 ISR/回调 | I2C 事务 | 现阶段仍直接执行 I2C 从机协议，是尚未迁移的例外 |

## TIM1 快环

公开中断入口位于 `Core/Src/stm32g4xx_it.c`：

```c
void TIM1_UP_TIM16_IRQHandler(void)
{
  MysysFastLoopISR();
}
```

TIM1 使用 center-aligned PWM，repetition counter 为 2。PWM 和 ADC 触发周期保持不变，update IRQ 降到约 18.67 kHz。`MysysFastLoopISR()` 每次执行 `Loop_FOC()`。

调度器启动前，ISR 还会用整数相位累加器短暂运行旧外环；ControlTask 调用 `MysysControlTaskBegin()` 后关闭这条兼容路径。因此稳态时 ISR 不再运行 `Loop_Control()`、速度/位置 PID 或 SmartKnob。

## 1 kHz 控制路径

```text
ControlTask
  -> 消费按键/未来生产者提交的控制命令邮箱
  -> 到达 1 ms 截止时间
       -> Loop_Control
            -> 机械角度和速度估计
            -> 电流/电压/温度低通
            -> 过压、越界和堵转检测
       -> MysysRunModeController
            -> speed_pid / pos_pid / current / handle_smart_knob
  -> 截止时间预算内处理最多 4 个 CAN 帧
```

ControlTask 使用绝对 tick 截止时间。若 CAN-I2C 桥接等遗留阻塞操作耗时过长，不会补跑多个过期控制步，而是从下一个真实截止时间继续。

## 模式状态机

| 模式 | 值 | 行为 |
| --- | ---: | --- |
| `MODE_SPEED` | 1 | 速度闭环，PID 输出相电流目标 |
| `MODE_POS` | 2 | 位置闭环，PID 输出相电流目标 |
| `MODE_CURRENT` | 3 | 通信直接设置电流目标 |
| `MODE_DIAL` | 4 | SmartKnob/Dial 手感模式 |
| `MODE_SPEED_ERR_PROTECT` | 5 | 速度模式堵转保护态 |
| `MODE_POS_ERR_PROTECT` | 6 | 位置模式堵转保护态 |

电机驱动底层状态由 `MotorDriverSetMode()` 设置，包括 OFF、RUN 和编码器校准模式。

## 外部命令路径

```text
CAN 扩展帧
  -> FDCAN1_IT0_IRQHandler
  -> HAL_FDCAN_RxFifo0Callback
  -> App_NotifyCanRxFromISR
  -> CommunicationTask / FDCAN_ReadPendingCommand
  -> 本机命令静态队列
  -> ControlTask / FDCAN_ProcessCommand
  -> 回复静态队列
  -> CommunicationTask / FDCAN_SendResponse

按键长按
  -> MaintenanceTask / LoopMysysOnce
  -> App_PostControlCommand
  -> 静态控制命令邮箱
  -> ControlTask / MysysCycleMode

I2C 主机写寄存器（暂未迁移）
  -> I2C1_EV_IRQHandler
  -> i2c1_event_irq_handler
  -> Slave_Complete_Callback
  -> 直接修改控制状态
```

最终结构中，I2C 也应只解码事务并提交控制命令；本阶段按计划不深化 I2C 电机桥接。
