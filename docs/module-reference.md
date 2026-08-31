# 模块参考

## `Core/Src/main.c`

职责：

- 应用入口。
- 通过 `SCB->VTOR` 选择 `0x08002000` 的完整应用向量表。
- Flash 配置初始化/写回。
- I2C 从机寄存器协议分发。
- 系统时钟配置。

关键函数：

| 函数 | 运行方式 |
| --- | --- |
| `IAP_Set()` | `main()` 最早调用，将 `SCB->VTOR` 指向完整应用向量表 |
| `micros()` | 基于 SysTick 和 HAL tick 生成微秒时间，PID/SmartKnob 使用 |
| `init_flash_data()` | 启动时读取 Flash 或写入默认配置 |
| `flash_data_write_back()` | 保存当前配置到 Flash |
| `Slave_Complete_Callback()` | I2C 完整事务回调，解析寄存器读写 |
| `main()` | 初始化本地运行所需外设、保持 CAN standby，调用 `InitMysys()` 和 `App_StartScheduler()` |

## `MyFile/src/mysys.c`

职责：

- 系统状态中心。
- PID 参数、目标值、保护状态、本地 profile 和持久化状态的全局定义。
- 业务初始化、ControlTask 控制步和 TIM1 FOC 快环。

关键函数：

| 函数 | 运行方式 |
| --- | --- |
| `InitMysys()` | 上电初始化 ADC、PWM、电机、编码器、Flash、本地 SmartKnob profile 和 OLED |
| `LoopMysysOnce()` | MaintenanceTask 调用的慢速单步函数 |
| `MysysStorageOnce()` | StorageTask 调用的安全写回单步函数 |
| `Loop_FOC()` | 约 18.67 kHz DMA2 RX 完成 ISR 调用，运行 FOC 和 ADC 处理 |
| `Loop_Control()` | 1 kHz ControlTask 调用，计算机械状态和保护 |
| `MysysControlStep()` | 1 kHz 外环、模式状态机和 SmartKnob 入口 |
| `MysysFastLoopISR()` | TIM1 update ISR helper，启动编码器 DMA |
| `MysysFastLoopOnEncoderSampleFromISR()` | DMA2 完成后提交同周期样本并接续 FOC |
| `speed_pid()` / `pos_pid()` | ControlTask 中的速度/位置外环，输出电流目标 |
| `crc8_MAXIM()` | 旧串口协议遗留 CRC helper |

## `MyFile/src/motordriver.c`

职责：

- FOC 电流环。
- 编码器角度修正和电角度换算。
- Clarke/Park、PI、逆 Park、SVM。
- 电机底层模式控制和编码器校准。

关键接口：

| 函数 | 作用 |
| --- | --- |
| `MotorDriverInit()` | 清电流环和校准状态 |
| `MotorDriverProcess()` | FOC 主处理，更新 PWM |
| `MotorDriverSetMode()` | OFF/RUN/ENC_CAL |
| `MotorDriverSetCurrentReal()` | 以 mA 设置相电流目标 |
| `MotorDriverGetMechanicalAngle()` | 返回单圈角度，单位 0.1 度 |
| `IsMotorDriverEncCalBusy()` | 查询编码器校准忙状态 |
| `GetMotorDriverEncCalOffset()` | 获取校准得到的 offset |

## `MyFile/src/myadc.c`

职责：

- 启动 ADC1 DMA。
- 三相电流零点校准。
- 从 DMA buffer 提取相电流、输入电压、温度。

运行：

- `MyADCInit()` 在 `InitMysys()` 中启动 ADC 校准和 DMA。
- `MyADCZeroCal()` 在驱动启动后记录零点。
- `MyAdcProcess()` 在 `Loop_FOC()` 中调用。

## `MyFile/src/tle5012b.c`

职责：

- 使用 SPI1 + DMA2 两字 normal 传输读取 TLE5012B 编码器。
- 通过 PA4 软件控制 CS。
- 处理 RX 完成、TX/RX 错误、超时和陈旧样本诊断。

运行：

- `EncoderInit()` 在 `InitMysys()` 中调用。
- `EncoderStartDmaReadFromISR()` 被 TIM1 update ISR 高频调用。
- `EncoderGetLatestAngle()` 在 DMA 完成后由 `MotorDriverProcess()` 读取，不发起 SPI 事务。

## `MyFile/src/encoder.c`

职责：

- 将单圈编码器角度展开为连续计数。
- 提供用于菜单导航的上/下事件和用于转速计算的计数差。

运行：

- `speed_encoder_update()` 在 `Loop_Control()` 中调用，用于速度估计。
- `encoder_update()` 当前主要作为普通编码器事件 helper，菜单实际使用 Dial 的 `current_position` 映射出 `encoder_up/down`。

## `MyFile/src/pid_controller.c`

职责：

- 平台无关 PID 控制器。
- 被速度环和位置环复用。

特点：

- 使用 `micros()` 计算实际采样间隔。
- 输出限幅。
- 积分限幅。
- 对输出变化率做 ramp 限制。
- D 项为 derivative on measurement。

## `MyFile/src/smart_knob.c`

职责：

- Dial/SmartKnob 手感算法。
- 根据机械弧度、速度和 detent 配置输出电流扭矩。

运行：

- `init_smart_knob()` 在上电加载本地预设时调用。
- `handle_smart_knob()` 由 1 kHz ControlTask 的 Dial 分支调用。
- `smart_knob_enter_navigation_mode()` 进入私有 `0xFF` 菜单导航挡位。
- `current_position` 是正常挡位位置；导航期间 LocalUi 读取其增量作为光标输入。

## `MyFile/src/local_ui.c`

职责：

- 绘制 64×48 圆形速度表、挡位位置、模式和故障覆盖层。
- 管理 `MODE/FORCE/STEP/LIMIT/SAVE` 菜单状态机。
- 把长按、双击和旋钮挡位变化转换成本机控制命令。

运行：MaintenanceTask 每 10 ms 调用 `LocalUiTask()`；内部将 OLED 刷新限制为 20 Hz。UI 只投递命令，不直接修改电机控制状态。

## `Core/Src/i2c.c`

职责：

- I2C1 CubeMX 初始化。
- 业务版 `user_i2c_init()`。
- I2C 主机读写 helper，用于 CAN-I2C 桥接。

关键点：

- `MX_I2C1_Init()` 默认地址是 `0x64`。
- `user_i2c_init()` 使用 Flash 中的 `i2c_address[0]`。
- `I2C_Read_Bytes()` 使用 repeated start：先写寄存器，再读数据。

## `Core/Src/i2c_ex.c`

职责：

- I2C1 从机中断收发状态机。
- 收包缓存和待发送缓存。
- I2C 错误恢复。

运行：

- 地址匹配时根据方向启用 RX 或 TX 中断。
- STOP 或下一次地址匹配前，如果收到数据，则调用 `Slave_Complete_Callback()`。
- 错误中断会反初始化并重新初始化 I2C1。

## `Core/Src/fdcan.c`

职责：

- FDCAN1 初始化。
- CAN 扩展帧协议解析。
- 本机控制命令。
- CAN-I2C 桥接命令。

运行：

当前本地入口不调用 `MX_FDCAN1_Init()`，`comm_type` 固定为 `COMM_TYPE_NONE`，CommunicationTask 不创建。以下是恢复旧总线功能时才会进入的路径：

- `user_fdcan_init()` 按 `can_id` 和 `bps_index` 设置 filter 和波特率。
- `HAL_FDCAN_RxFifo0Callback()` 只通知 CommunicationTask 并记录 FIFO loss。
- CommunicationTask 调用 `FDCAN_ReadPendingCommand()`，将本机命令排队给 ControlTask；命令 `19–22` 的桥接操作留在 CommunicationTask。
- ControlTask 调用 `FDCAN_ProcessCommand()` 修改本机状态，并将回复排队回 CommunicationTask。
- CAN ID/波特率重配置由 CommunicationTask 执行，Flash 保存由 StorageTask 延后完成。

## `Core/Src/flash.c`

职责：

- 片内 Flash page 59 的简单 EEPROM 包格式。
- 读、写、检查 package head 和 length。

运行：

- `readPackedMessageFromFlash()` 启动时读取配置。
- `writeMessageToFlash()` 保存配置，擦除 page 59 后写入 doubleword。

## `MyFile/src/oled_u8g2.c`

职责：

- U8g2 到 STM32 GPIO 的适配。
- OLED 软件 SPI、RST、DC 控制。
- 初始化 64x48 SSD1306。

运行：

- `u8g2Init()` 在 `InitMysys()` 中调用。

## `MyFile/src/u8g2_disp_fun.c`（旧界面）

职责：

- 旧 OLED 协议页面和阻塞式配置菜单。
- 通信/运行状态显示 helper。
- RGB 灯效策略。

运行：

- `u8g2_disp_init()` 仍用于启动显示初始化。
- 正常运行不再调用旧四页状态界面或旧阻塞菜单；本地 UI 由 `local_ui.c` 接管。
- 旧 `ws2812_flash()` 仅作为协议界面遗留代码保留，本地状态灯不再调用。

## `MyFile/src/button.c`

职责：

- PC6 按键非阻塞去抖。
- 生成延迟单击、双击、1.2 s 即时长按和旧 5 s 兼容事件。

运行：

- MaintenanceTask 通过 `LoopMysysOnce()` 每 10 ms 调用 `button_update()`。

## `MyFile/src/ws2812.c`

职责：

- SK6812/WS2812 两颗灯珠的静态颜色缓存和 PWM-DMA 发送。
- DMA busy/dirty 状态、丢回调超时恢复和诊断计数。

运行：

- `sk6812_init()` 清理静态颜色与帧缓存，不再使用堆分配。
- `neopixel_set_color()` 设置单颗灯颜色并应用亮度百分比。
- `ws2812_show()` 标记更新；`ws2812_service()` 仅在颜色变化且 DMA 空闲时发送 GRB 帧。
- `ws2812_on_pwm_complete()` 在 TIM3 CH2 完成回调中停止输出并释放 busy 状态。

## `MyFile/src/local_rgb.c`

职责：

- 将运行、暂停、菜单、编辑、保存和故障映射为唯一且确定的灯效。
- 生成低亮紫色呼吸与红色双闪时序。
- 保存提示至少保持 700 ms，避免 Flash 写入过快导致肉眼看不到。

运行：MaintenanceTask 每 10 ms 调用 `LocalRgbTask()`；效果计算上限为 50 Hz，颜色未变化时不会重发 DMA。

## `Core/Src/tim.c`

职责：

- TIM1 三相 PWM 和 ADC 触发配置。
- TIM3 WS2812 PWM-DMA 配置。

运行：

- TIM1 更新中断触发控制任务。
- TIM3 CH2 DMA 发送 LED 波形。

## `Core/Src/adc.c`

职责：

- ADC1 6 通道扫描、DMA circular 和 TIM1 TRGO 触发配置。

运行：

- DMA buffer 被 `myadc.c` 读取。

## `Core/Src/spi.c`

职责：

- SPI1 master 16-bit 初始化，用于 TLE5012B。

## `Core/Src/gpio.c`

职责：

- 所有普通 GPIO 和复用 GPIO 的基础初始化。

## `Core/Src/dma.c`

职责：

- DMA1、DMA2 和 DMAMUX 时钟。
- ADC DMA、TIM3 DMA 以及 SPI1 RX/TX DMA 通道和中断优先级。

## `Core/Src/stm32g4xx_it.c`

职责：

- Cortex-M 和外设中断入口。

维护重点：

- 公开 IRQ 函数名应只在这个文件中定义。
- 业务逻辑通过 helper 进入，例如 `MysysFastLoopISR()`、`i2c1_event_irq_handler()`。

## `U8g2_lib`

职责：

- U8g2/u8x8 显示库裁剪源码。

当前 CMake 只链接显示所需的部分 `.c` 文件和当前页面实际引用的字体。新增 UI 功能时，优先复用已有字体；确实要增加字体或源码时，用 map 文件确认 Flash 增量。
