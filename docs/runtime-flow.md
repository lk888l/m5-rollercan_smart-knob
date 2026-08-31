# 启动与运行流程

## 上电启动

`Core/Src/main.c` 的 `main()` 是唯一应用入口。

1. `IAP_Set()` 将 `SCB->VTOR` 指向应用区 `0x08002000` 的完整向量表。不能只复制旧实现中的 48 个向量；DMA2 Channel 1/2 为 IRQ 56/57，必须保留更高编号的中断入口。
2. `HAL_Init()` 初始化 HAL、SysTick 和 Flash 接口。
3. `SystemClock_Config()` 使用 HSI+PLL 配置系统时钟。
4. 依次初始化 GPIO、DMA、ADC1、TIM1、SPI1、TIM3 和 I2C1；将 CAN 收发器置于 standby，不初始化 FDCAN1。
5. `InitMysys()` 完成业务和电机硬件初始化。
6. `sk6812_init(PIXEL_MAX)` 初始化静态 RGB 缓冲，`LocalRgbInitialize()` 初始化本地状态灯。
7. `App_StartScheduler()` 创建全静态 FreeRTOS 对象并启动调度器；正常情况下不返回。

## `InitMysys()` 的实时启动顺序

`MyFile/src/mysys.c` 的 `InitMysys()` 必须先准备 FOC 的全部依赖，再开启 TIM1 update IRQ：

1. 清零角度和控制状态。
2. `MyADCInit()` 启动 ADC 校准和 circular DMA。
3. 设置 TIM1 CH4 ADC 触发点。
4. `MotorDriverInit()` 初始化 FOC 状态。
5. `EncoderInit()` 使能 TLE5012B 使用的 SPI1，并准备 DMA2 RX/TX 固定缓冲与通道。
6. `EncoderPrimeDmaRead()` 在 IRQ 关闭时执行一次有界 DMA 预热并提交首个有效角度。
7. 启动 TIM1 CH1/CH2/CH3 PWM。
8. 延时 20 ms 后执行 `MyADCZeroCal()`。
9. 保持 TIM1 连续运行，等待并清除一个自然 repetition update，再开启 update IRQ。
10. 读取 Flash 配置，加载编码器 offset 后重置多圈跟踪，避免 offset 生效前后的角度差被误计为旋转。
11. 加载本地 SmartKnob profile，固定 `MODE_DIAL`/`COMM_TYPE_NONE`，初始化 OLED 仪表盘并自动启动力反馈。

顺序 4～9 不可随意交换。若 SPI1 和 DMA2 通道准备完成前开启 TIM1 update IRQ，首次编码器事务无法产生有效 DMA 完成事件。直接停表并从 CNT=0 重启中心对齐 TIM1 也可能保留下降方向并制造极短首周期，因此这里对齐到持续运行中的自然 update。

## FreeRTOS 运行上下文

| 上下文 | 周期/触发 | 职责 |
| --- | --- | --- |
| TIM1 update ISR | 约 18.67 kHz | 拉低编码器 CS 并启动 DMA2 RX/TX；不调用 FreeRTOS API |
| DMA2 RX ISR | 每次编码器事务完成 | 提交本周期角度并运行 `Loop_FOC()`；不调用 FreeRTOS API |
| ControlTask | 1 kHz + 控制邮箱唤醒 | `Loop_Control()`、模式控制、保护、SmartKnob 和本机命令执行 |
| CommunicationTask | 条件创建 | 仅 `comm_type != COMM_TYPE_NONE` 时创建；当前本地配置不创建 |
| MaintenanceTask | 10 ms | 非阻塞按钮事件、LocalUi、OLED 20 Hz 刷新、LocalRgb 状态灯与 DMA service |
| StorageTask | 20 ms | 电机安全状态下的 Flash 写回 |
| FDCAN ISR | 当前不启用 | 保留的旧协议入口；本地启动路径未初始化外设 |
| I2C1 ISR/回调 | 当前无业务配置 | 旧从机协议源码保留，但 `COMM_TYPE_NONE` 不启动该通信路径 |

## TIM1 快环

公开中断入口位于 `Core/Src/stm32g4xx_it.c`：

```c
void TIM1_UP_TIM16_IRQHandler(void)
{
  if ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) &&
      (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET)) {
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    MysysFastLoopISR();
  }
}
```

TIM1 使用 center-aligned PWM，repetition counter 为 2。PWM 和 ADC 触发周期保持不变，update IRQ 降到约 18.67 kHz。`MysysFastLoopISR()` 只启动两字 SPI DMA；约 6.1 μs 后，DMA2 Channel 1 完成中断提交新角度并调用 `MysysFastLoopOnEncoderSampleFromISR()` 执行 `Loop_FOC()`。因此 FOC 使用的是同一 TIM1 周期发起的采样，而不是上一周期角度。

入口还检查 TIM1 当前方向、CNT 到下一边界的余量以及 NVIC pending。启动兼容外环若让某个 update 严重迟到，该事件会记入 `fast_loop_late_start_count` 并被丢弃，避免启动一个必然跨界的 SPI 帧；下一个正常 update 自动恢复。TIM1 同时启用 debug-freeze，调试器 halt CPU 时不会让定时器继续推进并制造虚假 overlap。

调度器启动前，DMA 完成后的 FOC 还会用整数相位累加器短暂运行旧外环；ControlTask 调用 `MysysControlTaskBegin()` 后关闭这条兼容路径。因此稳态时快环 ISR 不再运行 `Loop_Control()`、速度/位置 PID 或 SmartKnob。

## 1 kHz 控制路径

```text
ControlTask
  -> 消费 LocalUi 提交的进入菜单、退出菜单、启停命令
  -> 到达 1 ms 截止时间
       -> Loop_Control
            -> 机械角度和速度估计
            -> 电流/电压/温度低通
            -> 过压、越界和堵转检测
       -> MysysRunModeController
            -> MODE_DIAL / handle_smart_knob
```

ControlTask 使用绝对 tick 截止时间，不会补跑多个过期控制步。菜单操作也只通过静态队列改变控制状态，OLED 绘制不会进入 1 kHz 控制路径。

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

## 本地输入路径

```text
PC6 按键长按/双击
  -> MaintenanceTask / LoopMysysOnce
  -> button_update / LocalUiTask
  -> App_PostControlCommand
  -> 静态控制命令邮箱
  -> ControlTask
       -> MysysLocalMenuEnter / MysysLocalMenuExit
       -> MysysLocalToggleOutput

旋钮转动
  -> 1 kHz SmartKnob 更新 current_position
  -> LocalUiTask 读取位置差
  -> 菜单光标移动或参数增减
```

旧 CAN/I2C 命令解析仍在源码中，便于以后恢复兼容；当前入口不初始化 FDCAN、不创建 CommunicationTask，并把 `comm_type` 固定为 `COMM_TYPE_NONE`，因此运行不依赖外部总线。
