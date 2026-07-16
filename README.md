# ROLLERCAN 固件文档

ROLLERCAN 是一个基于 STM32G431 的无刷电机控制固件。工程由 STM32CubeMX 生成的 HAL/LL 外设初始化代码、`MyFile` 目录下的业务控制模块、裁剪后的 `U8g2_lib` 显示库，以及 CMake/MDK 两套工程入口组成。

固件的核心职责是：

- 通过 TIM1 三相 PWM 和 ADC 采样运行 FOC 电流环。
- 通过 TLE5012B SPI 编码器得到转子角度和机械位置。
- 提供速度、位置、电流和 Dial/SmartKnob 四类运行模式。
- 通过 I2C 从机寄存器协议和 CAN 扩展帧协议收发控制命令。
- 在 64x48 SSD1306 OLED 和 2 颗 SK6812/WS2812 灯珠上显示状态、菜单和告警。
- 将 I2C 地址、CAN ID、通信模式、PID 参数和保护开关等配置保存到片内 Flash。

## 快速入口

| 文档 | 内容 |
| --- | --- |
| [系统架构](docs/architecture.md) | 目录结构、模块分层、数据流、上下文边界 |
| [C++/FreeRTOS 重构](docs/freertos-refactor.md) | 当前迁移状态、任务/ISR 边界、频率和后续硬件验证门槛 |
| [启动与运行流程](docs/runtime-flow.md) | 上电初始化、主循环、TIM1 中断调度、模式切换 |
| [构建与烧录](docs/build-and-flash.md) | CMake/MDK 工程、工具链、构建命令、产物 |
| [外设与引脚](docs/peripherals.md) | TIM/ADC/SPI/I2C/FDCAN/GPIO/DMA 的用途 |
| [控制链路](docs/control-loop.md) | FOC、电流环、速度环、位置环、电流模式、Dial 模式 |
| [固件 SmartKnob](docs/smartknob-firmware.md) | 模块化模式、默认预设、CAN 在线配置和主动遥测 |
| [通信协议](docs/communication-protocol.md) | I2C 寄存器表、CAN 命令、CAN-I2C 桥接 |
| [显示与输入](docs/display-and-input.md) | OLED 页面、菜单、按键、灯效 |
| [持久化配置](docs/persistence.md) | Flash 数据布局、读写时机、保护状态保存 |
| [模块参考](docs/module-reference.md) | 每个源文件/模块的职责和运行方式 |
| [维护注意事项](docs/maintenance-notes.md) | CubeMX 再生成、U8g2 裁剪、调试建议 |

## 代码地图

```text
Core/
  Inc/, Src/          STM32CubeMX 生成的外设初始化、中断入口和系统文件
App/
  inc/, src/          C++17/FreeRTOS 静态任务和 C/C++ 边界
Middlewares/
  Third_Party/        STM32CubeG4 配套的 FreeRTOS 内核
MyFile/
  inc/, src/          手写业务模块：电机控制、系统调度、显示、按键、灯效、ADC、编码器等
U8g2_lib/             裁剪后的 U8g2/u8x8 显示库源码
cmake/                CMake 工具链和 CubeMX 生成的 CMake 子工程
MDK-ARM/              Keil/MDK 工程和旧构建产物
build/Debug/          CMake Debug 构建目录
ROLLERCAN.ioc         CubeMX 工程文件
STM32G431XX_FLASH.ld  GCC 链接脚本
```

## 一句话运行图

`main()` 完成外设初始化和 `InitMysys()` 后启动 FreeRTOS。1 kHz ControlTask 独占外环、保护状态机、SmartKnob 和本机控制状态；CommunicationTask 独占 FDCAN FIFO/发送并隔离 CAN-I2C 桥接；MaintenanceTask 负责 UI/按键/慢速维护，StorageTask 在电机停止后执行 Flash 写回。TIM1 中断以约 18.67 kHz 启动编码器 DMA，DMA2 RX 完成中断提交同周期角度并接续 FOC。ControlTask 与 FOC 通过带序号的驱动命令和传感器快照交换数据。

```text
上电
  -> HAL_Init/SystemClock_Config
  -> MX_GPIO/MX_DMA/MX_ADC/MX_TIM/MX_SPI/MX_I2C/MX_FDCAN
  -> InitMysys
       -> ADC DMA / TIM1 PWM / 电机驱动 / 编码器 / Flash 配置 / OLED / 通信
  -> App_StartScheduler
       -> ControlTask: 1 kHz 外环、保护、SmartKnob 和本机命令执行
       -> CommunicationTask: FDCAN RX/TX、帧解码和 CAN-I2C 桥接
       -> MaintenanceTask: 按键、显示、灯效、通信恢复
       -> StorageTask: 安全状态下的 Flash 写回

TIM1_UP_TIM16_IRQHandler
  -> MysysFastLoopISR
       -> 启动 TLE5012B SPI1 DMA2
DMA2_Channel1_IRQHandler
  -> 提交本周期编码器角度
  -> Loop_FOC
```

## 构建

当前 CMake 工程使用 Ninja 和 `arm-none-eabi-*` 工具链：

```powershell
cmake --preset Debug
cmake --build build\Debug
```

生成的主要产物位于 `build/Debug/ROLLERCAN.elf`，链接时会输出 `ROLLERCAN.map` 和内存占用。更多说明见 [构建与烧录](docs/build-and-flash.md)。

## 维护原则

- `Core/Src/stm32g4xx_it.c` 保留公开 IRQ 入口，TIM1 业务中断逻辑由 `MysysFastLoopISR()` 承担；该函数不得调用 FreeRTOS API。
- 调度器启动后，CAN 和按键命令必须通过静态队列进入 ControlTask；CommunicationTask 不得直接写 `motor_mode`、setpoint、PID 积分项或 fault state。
- ControlTask 与 TIM1 ISR 之间的数据必须通过 `fast_control_link` 交换；不要重新从任务直接修改 `currentloop_enable`、电流 PI 积分项或 PWM 驱动使能。
- `MyFile/src/mysys.c` 是系统状态、模式、PID 和保护逻辑的中心，改模式或单位时先从这里追数据流。
- `main.c` 中的 `Slave_Complete_Callback()` 是 I2C 寄存器协议入口；`Core/Src/fdcan.c` 是 CAN 协议和 CAN-I2C 桥接入口。
- `U8g2_lib` 已按当前 OLED 功能裁剪，增加字体或 U8g2 源文件前应先看 map/size。
