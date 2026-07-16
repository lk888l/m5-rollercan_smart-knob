# C++ / FreeRTOS 重构说明

## 当前状态

当前版本已完成第四阶段通信隔离：TIM1 中断只启动编码器 DMA，DMA2 RX 完成 ISR 提交同周期角度并接续 FOC；ControlTask 独占外环和本机业务状态；CommunicationTask 独占 FDCAN FIFO、帧发送和阻塞式 CAN-I2C 桥接。ControlTask 与 FOC ISR 之间继续通过带序号的命令/传感器快照交换数据。

已经完成：

- CMake 同时启用 C11、C++17 和 ASM；C++ 关闭 exceptions、RTTI 和线程安全静态初始化。
- 内置 STM32CubeG4 1.6.3 配套的 FreeRTOS Kernel 10.3.1。
- 所有任务、任务栈、TCB 和控制命令队列使用静态分配；未启用 FreeRTOS 软件定时器。
- FreeRTOS tick 为 1 kHz；SysTick 同时维护 HAL tick。
- TIM1 repetition counter 从 0 改为 2，update IRQ 从约 56 kHz 降到约 18.67 kHz。
- ADC DMA 保持 circular 搬运，但关闭未使用的 half-transfer 和 transfer-complete IRQ。
- FDCAN ISR 只唤醒 CommunicationTask，不在中断中读取 FIFO、解析协议或发送回复。
- `Loop_Control()`、速度/位置 PID、保护状态机和 SmartKnob 统一在 1 kHz ControlTask 中执行。
- 按键模式切换通过容量为 8 的静态控制命令邮箱提交，不再由 MaintenanceTask 直接修改 `motor_mode`。
- Flash 写回拆到 StorageTask，并在 `SYS_RUNNING` 时延后。
- TLE5012B 改为 DMA2 Channel 1 RX + Channel 2 TX 的两字 normal 传输；TIM1 启动事务，RX 完成 ISR 提交同周期角度并接续 FOC，异常时保留上次有效角度。
- `FastControlCommandSnapshot` 将驱动模式和 Iq 目标提交到 FOC 周期边界执行。
- `FastSensorSnapshot` 将编码器、电流、母线 ADC 和温度作为一致样本发布给 ControlTask。
- DWT CYCCNT 记录 FOC、ControlTask、CAN 单帧耗时和 1 kHz 周期抖动。
- 四个应用任务每秒记录一次实际 stack high-water mark。
- FDCAN ISR 改为唤醒 CommunicationTask，不再直接唤醒 ControlTask。
- 本机 CAN 命令经固定容量队列交给 ControlTask，回复再经独立静态队列返回 CommunicationTask。
- CAN-I2C 桥接命令 `19–22` 只在 CommunicationTask 执行，不再阻塞 1 kHz 控制任务。
- FDCAN TX 只由 CommunicationTask 调用；启动调度器前的同步兼容路径除外。
- CAN ID/波特率重配置也由 CommunicationTask 执行；StorageTask 只消费持久化标志。

## 任务配置

| 任务 | 优先级 | 静态栈 | 触发方式 | 当前职责 |
| --- | ---: | ---: | --- | --- |
| ControlTask | 5 | 896 words | 1 ms 绝对截止时间；控制邮箱可提前唤醒 | 外环、PID、保护、SmartKnob 和本机命令执行 |
| CommunicationTask | 3 | 768 words | FDCAN ISR / TX 回复 notification | RX FIFO、帧解码、TX 和 CAN-I2C 桥接 |
| MaintenanceTask | 2 | 768 words | 10 ms `vTaskDelayUntil` | 按键、OLED、RGB、I2C 恢复和慢速维护 |
| StorageTask | 1 | 384 words | 20 ms `vTaskDelayUntil` | 安全状态下的 Flash 写回 |
| IdleTask | 0 | 128 words | FreeRTOS 内核 | 空闲任务 |

任务栈单位是 FreeRTOS `StackType_t`，在 Cortex-M4F 上一个 word 为 4 字节。当前栈按迁移阶段保守配置，上板后应使用 `uxTaskGetStackHighWaterMark()` 测量再收紧。

CommunicationTask 只拥有传输和桥接，不直接修改本机控制状态。本机命令由 ControlTask 执行，因此不需要给 PID、模式和保护状态加互斥锁。

## 1 kHz ControlTask 调度

ControlTask 使用 FreeRTOS tick 的绝对 1 ms 截止时间，不使用软件定时器：

1. 等待下一个控制截止时间；控制命令可用 task notification 提前唤醒。
2. 到达截止时间时优先执行一次 `MysysControlStep()`，不补跑陈旧控制周期。
3. 在下一个截止时间前最多执行 4 条本机命令；剩余命令保留到下次唤醒。
4. 命令产生的 CAN 回复只写入静态 TX 队列，不在 ControlTask 调用 HAL FDCAN。

以下全局计数器可直接在调试器观察：

| 变量 | 含义 | 正常预期 |
| --- | --- | --- |
| `app_control_step_count` | 已执行的 1 kHz 控制步数 | 每秒约增加 1000 |
| `app_control_deadline_miss_count` | 跨过的控制截止周期数 | 稳态应保持 0 |
| `app_control_can_frame_count` | ControlTask 已处理的 CAN 帧数 | 随接收流量增加 |
| `app_control_command_drop_count` | 控制邮箱满导致的丢弃数 | 应保持 0 |
| `app_control_stack_min_words` | ControlTask 最小剩余栈 | 大于安全余量 |
| `app_maintenance_stack_min_words` | MaintenanceTask 最小剩余栈 | 大于安全余量 |
| `app_storage_stack_min_words` | StorageTask 最小剩余栈 | 大于安全余量 |
| `app_communication_stack_min_words` | CommunicationTask 最小剩余栈 | 大于安全余量 |
| `app_can_rx_count` | CommunicationTask 读取的总帧数 | 随总线流量增加 |
| `app_can_bridge_count` | 已执行的桥接帧数 | 仅使用桥接时增加 |
| `app_can_rx_drop_count` | HAL 读取或本机命令入队失败 | 应保持 0 |
| `app_can_tx_drop_count` | 回复队列或硬件 TX 入队失败 | 应保持 0 |
| `app_can_hw_fifo_loss_count` | FDCAN 硬件报告 FIFO message lost | 必须保持 0 |
| `app_can_rx_fifo_high_water` | RX FIFO 观测到的最高填充量 | 应低于硬件容量 |
| `app_can_control_queue_high_water` | 本机控制队列最高占用 | 应小于 8 |
| `app_can_response_queue_high_water` | 回复队列最高占用 | 应小于 8 |

CAN-I2C 桥接命令仍使用阻塞式 I2C，但已经隔离到优先级 3 的 CommunicationTask。桥接可能增加 CAN FIFO 压力和自身响应延迟，但不应再增加 ControlTask deadline miss。

## CAN 任务和静态通道

```text
FDCAN ISR
  -> notify CommunicationTask
  -> FDCAN_ReadPendingCommand
       -> command 19-22: CommunicationTask 执行桥接并直接回复
       -> other commands: enqueue ControlCommand
  -> ControlTask 在控制截止时间之后执行本机命令
  -> enqueue CanProtocolResponse
  -> notify CommunicationTask
  -> FDCAN_SendResponse
```

命令和回复队列容量均为 8。CommunicationTask 每次最多读取 16 帧；ControlTask 每个 pass 最多执行 4 条本机命令。FDCAN new-message 和 message-lost notification 都已启用。

CAN ID 或波特率变更的回复发送完成后，CommunicationTask 才会 DeInit/重新初始化 FDCAN，并设置延后写回标志。MaintenanceTask 和 StorageTask 不再操作 FDCAN handle。

## FOC / ControlTask 双向快照

ControlTask 不再直接修改 `currentloop_enable`、电流 PI 积分项、驱动 GPIO 或最终的 `iq_curr_pi_target`：

1. `MotorDriverSetMode()` 更新业务可见状态，并通过 `FastControlPublishDriverMode()` 发布期望驱动模式。
2. `MotorDriverSetCurrentReal/Adc()` 完成限幅和单位换算，然后发布 Iq 目标。
3. `MotorDriverPrepareCycleFromISR()` 在 TIM1 周期边界消费最新完整命令；模式变化、驱动 GPIO、PI 清零和 Iq 目标更新不会因 DMA 空档后移。
4. FOC 和 ADC 处理完成后发布 `FastSensorSnapshot`。
5. `Loop_Control()` 读取一致快照，并用快照内的原始编码器值完成速度展开。

快照使用奇偶 sequence 和 `DMB` 屏障。命令写入用极短 PRIMASK 临界区支持 ControlTask 和尚未迁移的 I2C 回调两个生产者；FOC ISR 遇到写入中的奇数 sequence 时不会自旋，只在下个约 53.6 μs 周期重试。传感器读取最多尝试三次，失败时保留上一份有效快照。

调试器可读取 `fast_control_command_apply_count`、`fast_sensor_read_retry_count` 和 `fast_sensor_read_failure_count`。正常运行时命令应用计数会随控制输出变化而增加；传感器失败计数应保持 0。

## DWT 运行时间指标

STM32G431 当前为 168 MHz，因此周期值除以 168 即约为微秒：

| 变量 | 含义 | 参考边界 |
| --- | --- | --- |
| `runtime_foc_last_cycles` | 最近一次从 TIM1 触发到 FOC 快照发布的总延迟 | 小于约 9000 cycles |
| `runtime_foc_max_cycles` | TIM1 到 FOC 完成的最大总延迟 | 必须明显小于约 9000 cycles |
| `runtime_foc_cpu_last_cycles` | 最近一次 TIM1 启动段与 DMA 完成/FOC 段的 CPU 活跃周期之和 | 应明显低于轮询基线 |
| `runtime_foc_cpu_max_cycles` | 快环 CPU 活跃周期最大值 | 与总延迟之间应保留 SPI 搬运空档 |
| `runtime_encoder_dma_last_cycles` | 最近一次 TIM1 到编码器角度提交的 DMA 延迟 | 约为 32-bit 线时长加固定中断开销 |
| `runtime_encoder_dma_max_cycles` | 编码器 DMA 最大延迟 | 不应逼近 9000 cycles |
| `runtime_tim1_isr_last/max_cycles` | TIM1 启动段 ISR 的最近/最大 CPU 活跃周期 | 应远低于完整 FOC 路径 |
| `runtime_encoder_dma_isr_last/max_cycles` | DMA 完成、FOC 和兼容调度段 ISR 的最近/最大 CPU 活跃周期 | 最大值必须低于约 9000 cycles |
| `runtime_control_last_cycles` | 最近一次控制步耗时 | 小于 168000 cycles |
| `runtime_control_max_cycles` | 控制步最大耗时 | 应保留 CAN/抢占余量 |
| `runtime_control_period_min_cycles` | 1 kHz 控制起点最短周期 | 接近 168000 cycles |
| `runtime_control_period_max_cycles` | 1 kHz 控制起点最长周期 | 接近 168000 cycles |
| `runtime_control_jitter_max_cycles` | 相对 1 ms 的最大绝对周期偏差 | 越小越好 |
| `runtime_can_frame_max_cycles` | 单帧 CAN 协议最大处理耗时 | 不应逼近 168000 cycles |
| `runtime_can_bridge_max_cycles` | CommunicationTask 单条桥接最大耗时 | 可超过 1 ms，但不得影响控制 deadline |

## 控制周期重新离散化

原 `Loop_Control()` 平均约 5.6 kHz，原模式控制和 SmartKnob 平均约 `56000 / 11 = 5.09 kHz`。迁移到 1 kHz 时同步修改了隐含采样周期的参数：

- 速度换算从旧调用频率常数改为 `60 * 1000`，角速度换算改为 `2π * 1000`。
- 电流、电压、温度和速度差低通按 1 ms 采样周期重新计算 2 Hz 一阶 LPF 系数。
- 速度/位置 PID 对旧离散增益应用约 `5.09` 的周期比例：Ki 乘该比例、Kd 除该比例；协议和 Flash 中保存/显示的增益值不变。
- SmartKnob 的固定每步 alpha 使用 `1 - (1 - alpha_old)^(legacy_rate / 1000)` 转换，保持原滤波器的实际时间常数。
- SmartKnob 自身 PID 继续使用 `micros()` 得到的动态 `Ts`。

这保证的是离散时间量纲和一阶时间常数的连续性，不等于上板后的闭环响应必然完全相同。速度环、位置环和手感仍需要用 Release 固件实测。

## 中断边界

### SysTick 启动顺序

HAL 在 FreeRTOS 调度器启动前就开启 SysTick。向量表中的 `SysTick_Handler()` 始终先调用 `HAL_IncTick()`，只有 FreeRTOS task list 初始化完成后才进入 `xPortSysTickHandler()`，否则会在启动阶段进入 `xTaskIncrementTick()` 并访问无效链表。

### TIM1 / FOC

`TIM1_UP_TIM16_IRQHandler()` 只调用 `MysysFastLoopISR()` 启动 SPI1 DMA。DMA2 Channel 1 RX 完成后调用 `MysysFastLoopOnEncoderSampleFromISR()` 接续 FOC。调度器启动后这两个快环 ISR：

- 两个快环 IRQ 的优先级均为 0，不调用任何 FreeRTOS API。
- 每次约 18.67 kHz update IRQ 启动一次两字读取，RX 完成 ISR 使用同周期新角度执行一次 `Loop_FOC()`。
- 不执行 `Loop_Control()`、速度/位置 PID、保护状态机或 SmartKnob。
- 不访问 OLED、Flash、CAN 或阻塞式业务接口。

在 ControlTask 接管之前，初始化阶段仍用整数相位累加器短暂运行旧外环，这是为了保持 ADC 零点校准和启动行为。`MysysControlTaskBegin()` 先停止 ISR 侧兼容调度，再切换 1 kHz 离散参数。

TIM1 update IRQ 必须最后开启：`MotorDriverInit()`、`EncoderInit()`、DMA2/SPI1、PWM 启动和 ADC 零点采样完成后，清除 pending update flag，再允许启动编码器 DMA。否则首次事务不会产生有效的 RX 完成事件。

### 编码器 DMA

SPI1 保持 Mode 1、16-bit、5.25 Mbit/s。DMA2 RX/TX 均为 normal 模式，每周期搬运 `{0x8021, 0xFFFF}` 两个 halfword；循环 DMA 不满足 TLE5012B 的 CS 分帧要求。RX 通道启用 TC/TE，中断完成后等待 `BSY` 有界清零并拉高 CS；TX 通道只启用 TE，正常完成不会额外进入 ISR。两个 DMA2 IRQ 和 TIM1 update 均为优先级 0。它们都不能调用 FreeRTOS API，因此不存在编码器任务通知路径。

调试器可读取 `encoder_dma_start_count`、`encoder_dma_complete_count`、`encoder_dma_error_count`、`encoder_dma_overlap_count`、`encoder_dma_stale_sample_count`、`encoder_dma_unexpected_irq_count`、`fast_loop_late_start_count` 和 `fast_loop_sync_timeout_count`。错误、重叠、陈旧样本、意外 IRQ 和同步超时正常运行时都应保持 0；启动兼容外环可能让 late-start 增加一次，但不得持续增长。

### ADC DMA

ADC DMA 保持 circular 模式，FOC 直接读取 DMA 缓冲区。当前没有使用 half/full callback，因此关闭 HT/TC interrupt，只保留硬件搬运和 DMA 错误处理能力。

### FDCAN

FDCAN IRQ 优先级为 6，满足 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` 的约束。ISR 只调用 `App_NotifyCanRxFromISR()` 并记录硬件 FIFO loss；FIFO 读取和回复由 CommunicationTask 完成。本机状态修改仍由 ControlTask 完成。调度器启动前保留同步处理作为启动兼容路径。

## 单写者边界

调度器启动后，以下状态应只由 ControlTask 修改：

- `motor_mode` 和运行/保护状态。
- 速度、位置、电流 setpoint。
- 速度/位置 PID 参数、积分项和输出。
- SmartKnob 状态和期望电流；最终 FOC 电流目标由 ISR 消费快照后写入。
- CAN 命令引起的电机启停和故障恢复。

当前已有的生产者路径：

- FDCAN ISR：只通知 CommunicationTask。
- CommunicationTask：解码帧并排队，不直接修改本机 PID、模式或保护状态。
- MaintenanceTask 按键：向静态邮箱提交 `APP_CONTROL_COMMAND_CYCLE_MODE`。
- 开机阻塞式 OLED 配置菜单：发生在调度器启动前，可直接设置启动配置。

I2C 从机 `Slave_Complete_Callback()` 仍会直接写控制状态，是明确保留的未迁移边界。后续深化 I2C 时，应把寄存器事务解码成同一控制命令邮箱，而不是在 I2C ISR/回调中直接操作 PID 和电机状态。

## 上板验证清单

本阶段应使用 Release 固件验证：

1. PB9 FOC 脉冲仍约为 18.67 kHz，脉宽表示 DMA 完成后的纯 FOC 计算段；逻辑分析仪同时确认 PA4 CS 内恰好有 32 个 5.25 MHz SCK。
2. `app_control_step_count` 每秒约增加 1000，空闲和连续 CAN 下 `app_control_deadline_miss_count` 保持 0。
3. `runtime_foc_max_cycles` 明显小于约 9000；`runtime_foc_cpu_*` 相比轮询基线每周期至少减少 450 cycles；`runtime_control_period_min/max_cycles` 围绕 168000。
4. 比较空闲、连续 CAN 和四种模式下的 `runtime_control_jitter_max_cycles`。
5. `fast_sensor_read_failure_count` 保持 0；ADC DMA 持续更新，编码器 DMA 的 error/overlap/stale/unexpected 计数和 `encoder_spi_timeout_count` 均不增加。
6. 连续 CAN 通信时四个 drop/loss 计数保持 0，三个 queue/FIFO high-water 不触顶。
7. 本机控制流量下 `runtime_can_frame_max_cycles` 稳定；桥接流量只影响 `runtime_can_bridge_max_cycles`。
8. `app_control_command_drop_count` 保持 0，长按切换模式每次只切换一次。
9. 分别验证速度、位置、电流、Dial/SmartKnob 四种模式；重点确认启停瞬间没有非预期电流。
10. 验证编码器校准、过压、位置越界、速度堵转、位置堵转和自动恢复状态机。
11. 运行各模式和 CAN 压力一段时间后读取四个 `app_*_stack_min_words`，再决定是否缩栈。

## 构建结果

重构前 Release 基线：

```text
text 82832, data 1296, bss 5376
```

当前第四阶段 Release：

```text
text 92200, data 1352, bss 18376
RAM   19728 / 32576 bytes (60.56%)
Flash 93564 / 112640 bytes (83.06%)
```

当前 Debug 也能链接，但 Flash 已占 `106352 / 112640` 字节（94.42%）。上板行为、CAN 压力、DWT 最大值和 SmartKnob 手感应以 Release 固件为准。
