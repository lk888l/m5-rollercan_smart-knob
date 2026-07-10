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
| [启动与运行流程](docs/runtime-flow.md) | 上电初始化、主循环、TIM1 中断调度、模式切换 |
| [构建与烧录](docs/build-and-flash.md) | CMake/MDK 工程、工具链、构建命令、产物 |
| [外设与引脚](docs/peripherals.md) | TIM/ADC/SPI/I2C/FDCAN/GPIO/DMA 的用途 |
| [控制链路](docs/control-loop.md) | FOC、电流环、速度环、位置环、电流模式、Dial 模式 |
| [通信协议](docs/communication-protocol.md) | I2C 寄存器表、CAN 命令、CAN-I2C 桥接 |
| [显示与输入](docs/display-and-input.md) | OLED 页面、菜单、按键、灯效 |
| [持久化配置](docs/persistence.md) | Flash 数据布局、读写时机、保护状态保存 |
| [模块参考](docs/module-reference.md) | 每个源文件/模块的职责和运行方式 |
| [维护注意事项](docs/maintenance-notes.md) | CubeMX 再生成、U8g2 裁剪、调试建议 |

## 代码地图

```text
Core/
  Inc/, Src/          STM32CubeMX 生成的外设初始化、中断入口和系统文件
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

`main()` 完成外设初始化后调用 `InitMysys()`，随后进入 `LoopMysys()` 的常驻 UI/通信维护循环；真正的电机实时控制由 TIM1 更新中断进入 `mysys_tim1_update_handler()`，按分频执行 `Loop_FOC()`、`Loop_Control()` 和模式控制逻辑。

```text
上电
  -> HAL_Init/SystemClock_Config
  -> MX_GPIO/MX_DMA/MX_ADC/MX_TIM/MX_SPI/MX_I2C/MX_FDCAN
  -> InitMysys
       -> ADC DMA / TIM1 PWM / 电机驱动 / 编码器 / Flash 配置 / OLED / 通信
  -> LoopMysys
       -> 按键、显示、灯效、通信恢复、配置写回

TIM1_UP_TIM16_IRQHandler
  -> mysys_tim1_update_handler
       -> Loop_FOC
       -> Loop_Control
       -> speed_pid / pos_pid / handle_smart_knob
```

## 构建

当前 CMake 工程使用 Ninja 和 `arm-none-eabi-*` 工具链：

```powershell
cmake --preset Debug
cmake --build build\Debug
```

生成的主要产物位于 `build/Debug/ROLLERCAN.elf`，链接时会输出 `ROLLERCAN.map` 和内存占用。更多说明见 [构建与烧录](docs/build-and-flash.md)。

## 维护原则

- `Core/Src/stm32g4xx_it.c` 应保留公开 IRQ 入口，业务中断逻辑放到 helper 函数中调用，例如 `mysys_tim1_update_handler()` 和 `i2c1_event_irq_handler()`。
- `MyFile/src/mysys.c` 是系统状态、模式、PID 和保护逻辑的中心，改模式或单位时先从这里追数据流。
- `main.c` 中的 `Slave_Complete_Callback()` 是 I2C 寄存器协议入口；`Core/Src/fdcan.c` 是 CAN 协议和 CAN-I2C 桥接入口。
- `U8g2_lib` 已按当前 OLED 功能裁剪，增加字体或 U8g2 源文件前应先看 map/size。
