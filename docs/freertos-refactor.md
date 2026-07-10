# C++ / FreeRTOS 重构说明

## 当前状态

当前版本已完成第二阶段迁移：TIM1 中断只保留 FOC 快环，原外环、保护状态机和 SmartKnob 已迁移到 1 kHz ControlTask。调度器启动后，CAN 协议和按键控制命令也在 ControlTask 上下文中执行，形成控制状态的单写者边界。

已经完成：

- CMake 同时启用 C11、C++17 和 ASM；C++ 关闭 exceptions、RTTI 和线程安全静态初始化。
- 内置 STM32CubeG4 1.6.3 配套的 FreeRTOS Kernel 10.3.1。
- 所有任务、任务栈、TCB 和控制命令队列使用静态分配；未启用 FreeRTOS 软件定时器。
- FreeRTOS tick 为 1 kHz；SysTick 同时维护 HAL tick。
- TIM1 repetition counter 从 0 改为 2，update IRQ 从约 56 kHz 降到约 18.67 kHz。
- ADC DMA 保持 circular 搬运，但关闭未使用的 half-transfer 和 transfer-complete IRQ。
- FDCAN ISR 只唤醒 ControlTask，不再在中断中解析协议或发送回复。
- `Loop_Control()`、速度/位置 PID、保护状态机和 SmartKnob 统一在 1 kHz ControlTask 中执行。
- 按键模式切换通过容量为 8 的静态控制命令邮箱提交，不再由 MaintenanceTask 直接修改 `motor_mode`。
- Flash 写回拆到 StorageTask，并在 `SYS_RUNNING` 时延后。
- TLE5012B SPI TXE/RXNE 轮询加入固定周期超时，异常时保留上次有效角度，避免锁死最高优先级 ISR。

## 任务配置

| 任务 | 优先级 | 静态栈 | 触发方式 | 当前职责 |
| --- | ---: | ---: | --- | --- |
| ControlTask | 5 | 896 words | 1 ms 绝对截止时间；CAN/邮箱可提前唤醒 | 外环、PID、保护、SmartKnob、控制命令、CAN 协议 |
| MaintenanceTask | 2 | 768 words | 10 ms `vTaskDelayUntil` | 按键、OLED、RGB、I2C 恢复和慢速维护 |
| StorageTask | 1 | 384 words | 20 ms `vTaskDelayUntil` | 安全状态下的 Flash 写回 |
| IdleTask | 0 | 128 words | FreeRTOS 内核 | 空闲任务 |

任务栈单位是 FreeRTOS `StackType_t`，在 Cortex-M4F 上一个 word 为 4 字节。当前栈按迁移阶段保守配置，上板后应使用 `uxTaskGetStackHighWaterMark()` 测量再收紧。

工程没有 CommunicationTask。CAN 协议有修改控制状态的能力，所以协议解析与控制步放在同一个 ControlTask 中，避免再引入锁或跨任务共享 PID 状态。

## 1 kHz ControlTask 调度

ControlTask 使用 FreeRTOS tick 的绝对 1 ms 截止时间，不使用软件定时器：

1. 等待下一个控制截止时间；CAN RX 或控制命令可以用 task notification 提前唤醒。
2. 先消费固定容量控制命令邮箱。
3. 到达截止时间时执行一次 `MysysControlStep()`，不补跑陈旧控制周期。
4. 在下一个截止时间前最多处理 4 个 CAN 帧；每帧后重新检查 tick，避免无界清空 FIFO 挤占控制步。

以下全局计数器可直接在调试器观察：

| 变量 | 含义 | 正常预期 |
| --- | --- | --- |
| `app_control_step_count` | 已执行的 1 kHz 控制步数 | 每秒约增加 1000 |
| `app_control_deadline_miss_count` | 跨过的控制截止周期数 | 稳态应保持 0 |
| `app_control_can_frame_count` | ControlTask 已处理的 CAN 帧数 | 随接收流量增加 |
| `app_control_command_drop_count` | 控制邮箱满导致的丢弃数 | 应保持 0 |

CAN-I2C 桥接命令仍可能执行阻塞式 I2C 操作，这是本阶段按要求暂不深化的例外；若使用该命令，应预期 deadline miss 计数可能增加。

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

`TIM1_UP_TIM16_IRQHandler()` 只调用 `MysysFastLoopISR()`。调度器启动后该路径：

- 优先级为 0，不调用任何 FreeRTOS API。
- 每次约 18.67 kHz update IRQ 执行一次 `Loop_FOC()`。
- 不执行 `Loop_Control()`、速度/位置 PID、保护状态机或 SmartKnob。
- 不访问 OLED、Flash、CAN 或阻塞式业务接口。

在 ControlTask 接管之前，初始化阶段仍用整数相位累加器短暂运行旧外环，这是为了保持 ADC 零点校准和启动行为。`MysysControlTaskBegin()` 先停止 ISR 侧兼容调度，再切换 1 kHz 离散参数。

TIM1 update IRQ 必须最后开启：`MotorDriverInit()`、`EncoderInit()`、PWM 启动和 ADC 零点采样完成后，清除 pending update flag，再允许进入 FOC。否则 SPI1 尚未使能时首次编码器读取会永远等待 RXNE。

### ADC DMA

ADC DMA 保持 circular 模式，FOC 直接读取 DMA 缓冲区。当前没有使用 half/full callback，因此关闭 HT/TC interrupt，只保留硬件搬运和 DMA 错误处理能力。

### FDCAN

FDCAN IRQ 优先级为 6，满足 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` 的约束。ISR 只调用 `App_NotifyCanRxFromISR()`；协议解析和回复由 ControlTask 完成。调度器启动前保留同步处理作为启动兼容路径。

## 单写者边界

调度器启动后，以下状态应只由 ControlTask 修改：

- `motor_mode` 和运行/保护状态。
- 速度、位置、电流 setpoint。
- 速度/位置 PID 参数、积分项和输出。
- SmartKnob 状态和电流目标。
- CAN 命令引起的电机启停和故障恢复。

当前已有的生产者路径：

- FDCAN ISR：只通知 ControlTask；CAN FIFO 在 ControlTask 中解析。
- MaintenanceTask 按键：向静态邮箱提交 `APP_CONTROL_COMMAND_CYCLE_MODE`。
- 开机阻塞式 OLED 配置菜单：发生在调度器启动前，可直接设置启动配置。

I2C 从机 `Slave_Complete_Callback()` 仍会直接写控制状态，是明确保留的未迁移边界。后续深化 I2C 时，应把寄存器事务解码成同一控制命令邮箱，而不是在 I2C ISR/回调中直接操作 PID 和电机状态。

## 上板验证清单

本阶段应使用 Release 固件验证：

1. PB9 FOC 脉冲仍约为 18.67 kHz，脉宽和最大抖动正常。
2. `app_control_step_count` 每秒约增加 1000，空闲和连续 CAN 下 `app_control_deadline_miss_count` 保持 0。
3. ADC DMA 缓冲持续更新，TLE5012B 正常时 `encoder_spi_timeout_count` 不增加。
4. 连续 CAN 通信无 RX FIFO overflow，回复延迟稳定，`app_control_can_frame_count` 与接收量一致。
5. `app_control_command_drop_count` 保持 0，长按切换模式每次只切换一次。
6. 分别验证速度、位置、电流、Dial/SmartKnob 四种模式；重点比较速度响应、位置保持和 detent 手感。
7. 验证过压、位置越界、速度堵转、位置堵转和自动恢复状态机。
8. 电机运行时请求保存，确认 Flash 写入被延后；停机后确认参数落盘。
9. 测量三个任务和 IdleTask 的 stack high-water mark，并确认 stack overflow hook 未触发。

## 构建结果

重构前 Release 基线：

```text
text 82832, data 1296, bss 5376
```

当前第二阶段 Release：

```text
text 90184, data 1324, bss 14836
RAM   16160 / 32576 bytes (49.61%)
Flash 91520 / 112640 bytes (81.25%)
```

当前 Debug 也能链接，但 Flash 已占 `104148 / 112640` 字节（92.46%）。上板行为和 SmartKnob 手感应以 Release 固件为准。
