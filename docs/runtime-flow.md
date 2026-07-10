# 启动与运行流程

## 上电启动

`Core/Src/main.c` 的 `main()` 是唯一应用入口。

1. `IAP_Set()` 将应用区 `0x08002000` 的前 48 个向量复制到 SRAM，并把 SRAM remap 到 `0x00000000`。
2. `HAL_Init()` 初始化 HAL、SysTick 和 Flash 接口。
3. `SystemClock_Config()` 使用 HSI+PLL，配置 SYSCLK、AHB、APB。
4. 依次执行 CubeMX 外设初始化：
   - `MX_GPIO_Init()`
   - `MX_DMA_Init()`
   - `MX_ADC1_Init()`
   - `MX_TIM1_Init()`
   - `MX_SPI1_Init()`
   - `MX_TIM3_Init()`
   - `MX_I2C1_Init()`
   - `MX_FDCAN1_Init()`
5. `InitMysys()` 初始化业务系统。
6. `sk6812_init(PIXEL_MAX)` 为 RGB 灯色缓存分配内存。
7. `while (1)` 中调用 `LoopMysys()`。

注意：`LoopMysys()` 内部自己也是无限循环，所以 `main()` 的外层 `while` 实际不会重复多次返回。

## `InitMysys()` 做什么

`MyFile/src/mysys.c` 的 `InitMysys()` 是业务初始化中心：

1. 清零机械角度、目标角度、控制计数器。
2. `GPIOB->BSRR=1<<1` 使能 DRV8311 内部电流传感器相关引脚。
3. `MyADCInit()` 启动 ADC1 校准和 DMA 循环采样。
4. `TIM1->CCR4=995` 设置 TIM1 CH4 作为 ADC 触发点。
5. 启动 TIM1 CH1/CH2/CH3 三相 PWM。
6. 打开 TIM1 update 中断，用于 FOC 和外层控制循环调度。
7. `MotorDriverInit()` 初始化 FOC 电流环状态。
8. `MyADCZeroCal()` 以当前 ADC 采样作为三相电流零点。
9. `EncoderInit()` 初始化 TLE5012B SPI 编码器。
10. `init_flash_data()` 读取或写入默认配置。
11. `u8g2Init(&u8g2)` 初始化 SSD1306 OLED。
12. 如果开机时 `SYS_SW` 被按下，则进入配置菜单。
13. `init_pid()` 根据当前 PID 索引初始化速度环和位置环。
14. 根据 `comm_type` 启用 I2C、CAN 或 CAN-I2C 双模式。
15. `u8g2_disp_init()` 显示启动 logo 和基础 UI。

## 主循环 `LoopMysys()`

主循环不做实时电机控制，它负责慢速任务：

- 检测 I2C STOP 超时，必要时重新初始化 I2C1。
- 检测 `flash_data_write_back_flag`，把参数写回 Flash。
- 检测 `can_change_flag`，重新初始化 CAN 并保存 CAN ID。
- 扫描按键：
  - 短按切换 OLED 页面。
  - 长长按且允许模式切换时，循环切换 `motor_mode`。
- 更新 OLED 模式标识、页面标识和通信闪烁标识。
- 根据错误状态强制显示 OVP、JAM、RANGE 页面。
- 消费外部写入的 RGB 颜色队列。
- CAN 波特率延迟切换。
- 根据 `dis_show_flag` 绘制当前页面。
- 调用 `ws2812_flash()` 输出 RGB 状态灯。
- 每 1 秒翻转 `running_index` 和 `status_flag`，供屏幕闪烁使用。

## TIM1 实时调度

TIM1 更新中断入口在 `Core/Src/stm32g4xx_it.c`：

```c
void TIM1_UP_TIM16_IRQHandler(void)
{
  mysys_tim1_update_handler();
  HAL_TIM_IRQHandler(&htim1);
}
```

`mysys_tim1_update_handler()` 做三个分频调度：

| 计数器 | 执行动作 | 作用 |
| --- | --- | --- |
| `counter_loop_foc` 到 2 | `Loop_FOC()` | 电机 FOC 电流环和 ADC 处理 |
| `counter_loop_control` 到 9 | `Loop_Control()` | 机械角度、速度、电压、电流、保护检测 |
| `pid_compute_counter` 到 10 | 模式控制 | 速度 PID、位置 PID、电流状态检查、Dial 手感 |

真实频率取决于 TIM1 update 频率。代码结构上，FOC 最快，其次是外层控制，PID/模式逻辑更慢。

## 模式状态机

`motor_mode` 定义在 `MyFile/inc/mysys.h`：

| 模式 | 值 | 行为 |
| --- | --- | --- |
| `MODE_SPEED` | 1 | 速度闭环，PID 输出相电流目标 |
| `MODE_POS` | 2 | 位置闭环，PID 输出相电流目标 |
| `MODE_CURRENT` | 3 | 通信直接设置电流目标 |
| `MODE_DIAL` | 4 | SmartKnob/Dial 手感模式 |
| `MODE_SPEED_ERR_PROTECT` | 5 | 速度模式堵转保护态 |
| `MODE_POS_ERR_PROTECT` | 6 | 位置模式堵转保护态 |

电机驱动底层模式由 `MotorDriverSetMode()` 设置：

| 底层模式 | 行为 |
| --- | --- |
| `MDRV_MODE_OFF` | 关闭驱动输出、清 PI 积分、根据错误设置 `sys_status` |
| `MDRV_MODE_RUN` | 使能驱动输出、电流环闭环运行、`sys_status=SYS_RUNNING` |
| `MDRV_MODE_ENC_CAL` | 进入编码器校准，注入固定电角度电流，完成后自动回到 OFF |

## 外部命令进入系统的路径

```text
I2C 主机写寄存器
  -> I2C1_EV_IRQHandler
  -> i2c1_event_irq_handler
  -> Slave_Complete_Callback
  -> 改 motor_mode / setpoint / PID / Flash / RGB

CAN 扩展帧
  -> FDCAN1_IT0_IRQHandler
  -> HAL_FDCAN_RxFifo0Callback
  -> 解析 cmd_id/cmd_para/option
  -> 本机控制或 I2C 桥接
```

这两条路径最终都会修改 `mysys.c` 中的全局状态，实时控制则在 TIM1 中断里读取这些状态。
