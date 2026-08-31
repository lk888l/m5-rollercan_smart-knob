# ROLLERCAN 固件文档

[English](README.md) | 简体中文

这是 M5 Stack RollerCAN 的本地直运行 SmartKnob 固件，基于 M5 的[官方开源固件](https://github.com/m5stack/M5Unit-RollerCAN-Internal-FW)修改。FOC、触感状态机、模式切换、参数设置和 OLED 都在 STM32G431 上完成，不需要 CANFD 上位机参与运行。

旧 I2C/CANFD 协议代码仍保留作参考，但当前入口不会初始化 FDCAN，CAN 收发任务也不会创建，外部收发器保持 standby。

固件的核心职责是：

- 通过 TIM1 三相 PWM 和 ADC 采样运行 FOC 电流环。
- 通过 TLE5012B SPI 编码器得到转子角度和机械位置。
- 在本地运行 12 种 Dial/SmartKnob 触感预设。
- 通过旋钮挡位、长按和双击操作本地菜单。
- 在 64×48 SSD1306 OLED 上显示圆形速度表、挡位、模式和告警。
- 用稳定的 RGB 呼吸/常亮/双闪区分运行、暂停、菜单、保存和故障。
- 将模式、力度、挡位角、电流上限和编码器 offset 保存到片内 Flash。

## 快速入口

| 文档 | 内容 |
| --- | --- |
| [系统架构](docs/architecture.md) | 目录结构、模块分层、数据流、上下文边界 |
| [C++/FreeRTOS 重构](docs/freertos-refactor.md) | 当前迁移状态、任务/ISR 边界、频率和后续硬件验证门槛 |
| [启动与运行流程](docs/runtime-flow.md) | 上电初始化、主循环、TIM1 中断调度、模式切换 |
| [构建与烧录](docs/build-and-flash.md) | CMake/MDK 工程、工具链、构建命令、产物 |
| [外设与引脚](docs/peripherals.md) | TIM/ADC/SPI/I2C/FDCAN/GPIO/DMA 的用途 |
| [控制链路](docs/control-loop.md) | FOC、电流环、速度环、位置环、电流模式、Dial 模式 |
| [固件 SmartKnob](docs/smartknob-firmware.md) | 本地预设、运行配置、导航挡位、电流限制和旧 CAN 接口参考 |
| [通信协议](docs/communication-protocol.md) | I2C 寄存器表、CAN 命令、CAN-I2C 桥接 |
| [显示与输入](docs/display-and-input.md) | 圆形仪表盘、本地菜单、长按/双击、参数范围和安全边界 |
| [持久化配置](docs/persistence.md) | Flash 数据布局、编码器 offset 保存、读写时机、保护状态保存 |
| [模块参考](docs/module-reference.md) | 每个源文件/模块的职责和运行方式 |
| [维护注意事项](docs/maintenance-notes.md) | CubeMX 再生成、U8g2 裁剪、调试建议 |

## 代码地图

```text
Core/
  Inc/, Src/          STM32CubeMX 生成的外设初始化、中断入口和系统文件
App/
  inc/, src/          C++17/FreeRTOS 静态任务和 C/C++ 边界
ThirdParty/
  FreeRTOS-Kernel/    CubeMX 管理范围外的 FreeRTOS 内核和 Cortex-M4F port
MyFile/
  inc/, src/          手写业务模块：电机控制、系统调度、显示、按键、灯效、ADC、编码器等
U8g2_lib/             裁剪后的 U8g2/u8x8 显示库源码
cmake/                CMake 工具链和 CubeMX 生成的 CMake 子工程
linker/
  ROLLERCAN_APP.ld    CMake 实际使用的应用链接脚本
MDK-ARM/              Keil/MDK 工程和旧构建产物
build/Debug/          CMake Debug 构建目录
ROLLERCAN.ioc         CubeMX 工程文件
STM32G431XX_FLASH.ld  CubeMX 生成的默认链接脚本，CMake 不使用
```

## 一句话运行图

`main()` 完成外设初始化和 `InitMysys()` 后启动 FreeRTOS。1 kHz ControlTask 独占保护状态机、SmartKnob 和本机控制状态；MaintenanceTask 负责按键、OLED 和灯效；StorageTask 在电机卸力后保存本地配置。当前不创建 CommunicationTask。TIM1 中断以约 18.67 kHz 启动编码器 DMA，DMA2 RX 完成中断提交同周期角度并接续 FOC。

```text
上电
  -> HAL_Init/SystemClock_Config
  -> MX_GPIO/MX_DMA/MX_ADC/MX_TIM/MX_SPI/MX_I2C
  -> InitMysys
       -> ADC DMA / TIM1 PWM / 电机驱动 / 编码器 / Flash 本地配置 / OLED
       -> 加载编码器 offset 后重置多圈跟踪并锚定当前触感中心
       -> 启动本地 Dial 输出，CAN 收发器保持 standby
  -> App_StartScheduler
       -> ControlTask: 1 kHz 外环、保护、SmartKnob 和本机命令执行
       -> MaintenanceTask: 长按/双击、仪表盘、本地菜单和灯效
       -> StorageTask: 安全状态下的 Flash 写回

TIM1_UP_TIM16_IRQHandler
  -> MysysFastLoopISR
       -> 启动 TLE5012B SPI1 DMA2
DMA2_Channel1_IRQHandler
  -> 提交本周期编码器角度
  -> Loop_FOC
```

## 本地操作

- 正常界面双击：启用/暂停力反馈。
- 正常界面长按约 1.2 秒：进入菜单。
- 菜单旋转：用独立的 12° 挡位移动光标或调整数值。
- 菜单双击：进入或确认；参数页长按取消；根菜单长按保存退出。
- 菜单可设置 `MODE`、`FORCE`、`STEP` 和 `LIMIT`。

保存时固件先清零电流、关闭驱动，完成 Flash 擦写和校验后再恢复进入菜单前的运行/暂停状态。完整范围和模式缩写见[显示与输入](docs/display-and-input.md)。

## CAN FD 总线配置

以下内容是保留的旧协议参考。当前本地直运行入口不调用 `MX_FDCAN1_Init()`，也不创建 CommunicationTask；如需恢复总线控制，需要同时恢复外设初始化和通信任务。

默认 `bps_index=0` 时，固件 CAN 总线配置与以下命令精确一致：

```bash
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 sjw 5 dsjw 3 fd on
```

FDCAN 使用由 160 MHz 系统 PLL 分频得到的 80 MHz PLLQ 时钟。仲裁段预分频为
1，位时间为 `1 + 63 + 16` TQ，SJW 为 5，精确得到 1 Mbit/s 和 80%
采样点；数据段预分频为 1，位时间为 `1 + 11 + 4` TQ，DSJW 为 3，
精确得到 5 Mbit/s 和 75% 采样点。协议发送帧为开启速率切换的 8 字节
CAN FD 扩展 ID 帧。

原有波特率选项仍然保留：`bps_index=1` 和 `bps_index=2` 只会将仲裁段
分别改为 500 kbit/s 和 125 kbit/s，此时上位机 CAN 接口也必须使用相同
仲裁速率；数据段仍保持 5 Mbit/s。

PLL、FDCAN 位时序、TIM1 和 TIM3 参数也已写入 `ROLLERCAN.ioc`，因此使用
STM32CubeMX 重新生成工程时会保留这些设置。系统时钟从 168 MHz 调整到
160 MHz 后，使用 TIM1 周期 952 和 TIM3 周期 200 进行补偿：FOC 中断仍约为
18.67 kHz，灯带数据率仍为 800 kHz；SPI1 现运行于 5.0 Mbit/s。

## 构建

当前 CMake 工程使用 Ninja 和 `arm-none-eabi-*` 工具链：

```powershell
cmake --preset Debug
cmake --build build\Debug
```

生成的主要产物位于 `build/Debug/ROLLERCAN.elf`，链接时会输出 `ROLLERCAN.map` 和内存占用。更多说明见 [构建与烧录](docs/build-and-flash.md)。

## 地址偏移与烧录产物

为了兼容原版固件的启动布局，CMake 构建没有把应用链接到 STM32 的 Flash/RAM 起点，而是保留了以下区域：

| 区域 | 地址 | 用途 |
| --- | --- | --- |
| Flash 启动区 | `0x08000000..0x08001FFF` | 预留 8 KiB，供原版 bootloader/启动入口使用 |
| Flash 应用区 | `0x08002000..0x0801D7FF` | ROLLERCAN 应用，长度 `0x1B800`（110 KiB） |
| Flash 配置页 | `0x0801D800..0x0801DFFF` | 持久化设置，包括编码器校准 offset；应用链接器不会放置代码到这里 |
| RAM 保留区 | `0x20000000..0x200000BF` | 预留 `0xC0` 字节，兼容原版启动交接数据 |
| RAM 应用区 | `0x200000C0..0x20007FFF` | 应用实际使用的 RAM |

偏移由 [linker/ROLLERCAN_APP.ld](linker/ROLLERCAN_APP.ld) 定义：

```ld
MEMORY
{
  RAM   (xrw) : ORIGIN = 0x200000C0, LENGTH = 0x7F40
  FLASH (rx)  : ORIGIN = 0x08002000, LENGTH = 0x1B800
}
```

CMake 工具链通过 `-T` 明确选择这份用户维护的链接脚本：

```cmake
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/linker/ROLLERCAN_APP.ld\"")
```

这段配置位于 `cmake/gcc-arm-none-eabi.cmake` 和 `cmake/starm-clang.cmake`。CubeMX 可以重新生成根目录的 `STM32G431XX_FLASH.ld`，但 CMake 不使用该默认脚本，所以重新生成不会取消上述偏移。

根 `CMakeLists.txt` 在链接后通过 `objcopy` 生成三种可烧录产物：

```cmake
# 保留 ELF 中的地址，生成应用 HEX
arm-none-eabi-objcopy -O ihex ROLLERCAN.elf ROLLERCAN.hex

# 生成不携带地址信息的纯应用 BIN
arm-none-eabi-objcopy -O binary ROLLERCAN.elf ROLLERCAN.bin

# 从应用提取中断向量，并在 0x08000000 增加一份镜像
# 最终生成 ROLLERCAN_standalone.hex
```

烧录时按设备启动方式选择，不能把多个方案混用：

| 使用场景 | 应烧录的文件 | 烧录地址/注意事项 |
| --- | --- | --- |
| 设备保留原版 bootloader | `build/<Debug或Release>/ROLLERCAN.hex` | HEX 自带地址，应用写入 `0x08002000`；不要烧 `standalone.hex` 覆盖 bootloader |
| 设备保留原版 bootloader，烧纯 BIN | `build/<Debug或Release>/ROLLERCAN.bin` | 必须手动指定地址 `0x08002000`，绝不能指定 `0x08000000` |
| 使用 ST-LINK/GDB 下载调试且已有 bootloader | `build/<Debug或Release>/ROLLERCAN.elf` | ELF 自带段地址和调试符号，工具会把应用写入 `0x08002000` |
| 整片擦除、设备上没有 bootloader，需要直接冷启动 | `build/<Debug或Release>/ROLLERCAN_standalone.hex` | 同时写入 `0x08000000` 的向量镜像和 `0x08002000` 的应用；每次更新都应重新烧完整 standalone HEX |

最常见的选择是：保留原版启动程序时烧 `ROLLERCAN.hex`；开发板整片擦除后独立运行时烧 `ROLLERCAN_standalone.hex`。`ROLLERCAN.bin` 本身不包含地址，只有在烧录工具中明确填写 `0x08002000` 才能使用。仓库中 `MDK-ARM/ROLLERCAN/` 下的文件是旧 MDK 构建产物，不应代替当前 `build/Debug` 或 `build/Release` 里的 CMake 产物。

## 维护原则

- `Core/Src/stm32g4xx_it.c` 保留公开 IRQ 入口，TIM1 业务中断逻辑由 `MysysFastLoopISR()` 承担；该函数不得调用 FreeRTOS API。
- 调度器启动后，CAN 和按键命令必须通过静态队列进入 ControlTask；CommunicationTask 不得直接写 `motor_mode`、setpoint、PID 积分项或 fault state。
- ControlTask 与 TIM1 ISR 之间的数据必须通过 `fast_control_link` 交换；不要重新从任务直接修改 `currentloop_enable`、电流 PI 积分项或 PWM 驱动使能。
- `MyFile/src/mysys.c` 是系统状态、模式、PID 和保护逻辑的中心，改模式或单位时先从这里追数据流。
- `main.c` 中的 `Slave_Complete_Callback()` 是 I2C 寄存器协议入口；`Core/Src/fdcan.c` 是 CAN 协议和 CAN-I2C 桥接入口。
- `U8g2_lib` 已按当前 OLED 功能裁剪，增加字体或 U8g2 源文件前应先看 map/size。


## Related Link

See also examples using conventional methods here.

- [Unit RollerCAN & Datasheet](https://docs.m5stack.com/en/unit/Unit-RollerCAN)

## Related Project

This project references the following open source projects.

- [smartknob](https://github.com/scottbez1/smartknob)
- [PID_Controller](https://github.com/tcleg/PID_Controller)
- [u8g2](https://github.com/olikraus/u8g2)

## License

- [smartknob][] Copyright (c) 2022 Scott Bezek and licensed under Apache License, Version 2.0 License.
- [PID_Controller][] Copyright (c) 2013-2014 tcleg and licensed under GPLv3 License.
- [u8g2][] Copyright (c) 2016 olikraus and licensed under BSD License.

[smartknob]: https://github.com/scottbez1/smartknob
[PID_Controller]: https://github.com/tcleg/PID_Controller
[u8g2]: https://github.com/olikraus/u8g2
