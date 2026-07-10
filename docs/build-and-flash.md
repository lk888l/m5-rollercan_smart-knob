# 构建与烧录

## 工程入口

本仓库同时保留两套工程入口：

| 入口 | 文件 | 用途 |
| --- | --- | --- |
| CMake/Ninja | `CMakeLists.txt`, `CMakePresets.json`, `cmake/` | 当前推荐的可脚本化构建 |
| Keil/MDK | `MDK-ARM/ROLLERCAN.uvprojx` | 兼容原有 MDK 工程和调试流程 |
| CubeMX | `ROLLERCAN.ioc` | 外设配置源文件，用于再生成 `Core/` 和 `cmake/stm32cubemx/` |

## CMake 构建

前提：

- `cmake` 和 Ninja 可用。
- `arm-none-eabi-gcc/g++/objcopy/size` 在 PATH 中，或由现有 build cache 指到工具链。
- 当前目录为项目根目录。

常用命令：

```powershell
cmake --preset Debug
cmake --build build\Debug
```

Release：

```powershell
cmake --preset Release
cmake --build build\Release
```

已有 `build/Debug` 时，也可以直接：

```powershell
cmake --build build\Debug
```

## CMake 工程组成

根 `CMakeLists.txt`：

- 设置 C11。
- 添加 `cmake/stm32cubemx` 子目录。
- `file(GLOB MYFILE_SOURCES "MyFile/src/*.c")` 收集手写业务源。
- 显式列出当前需要参与链接的 U8g2 源文件。
- 手动加入 `Core/Src/flash.c` 和 `Core/Src/i2c_ex.c`。
- 添加 `MyFile/inc`、`U8g2_lib`、CMSIS-DSP include 路径。

`cmake/stm32cubemx/CMakeLists.txt`：

- 设置 `USE_FULL_LL_DRIVER`、`USE_HAL_DRIVER`、`STM32G431xx`。
- 把 CubeMX 生成的应用源加入目标。
- 建立 `STM32_Drivers` object library。
- 链接 STM32 HAL/LL 驱动和数学库 `m`。

工具链文件 `cmake/gcc-arm-none-eabi.cmake`：

- 目标 CPU：Cortex-M4。
- FPU：`fpv4-sp-d16`。
- ABI：hard-float。
- Debug：`-O0 -g3`。
- Release：`-Os -g0`。
- 链接脚本：`STM32G431XX_FLASH.ld`。
- 链接选项包含 `--gc-sections`、map 文件输出和内存占用打印。

## 构建产物

CMake Debug 常见产物：

| 文件 | 说明 |
| --- | --- |
| `build/Debug/ROLLERCAN.elf` | GCC 链接后的固件 ELF |
| `build/Debug/ROLLERCAN.map` | 链接 map，用于查看符号和 Flash/RAM 占用 |
| `build/Debug/compile_commands.json` | clangd/编辑器索引数据库 |

CMake 当前没有在脚本里显式添加 `.hex` 或 `.bin` 生成规则；如果需要，可用 `arm-none-eabi-objcopy` 从 ELF 生成。

示例：

```powershell
arm-none-eabi-objcopy -O ihex build\Debug\ROLLERCAN.elf build\Debug\ROLLERCAN.hex
arm-none-eabi-objcopy -O binary build\Debug\ROLLERCAN.elf build\Debug\ROLLERCAN.bin
```

## 烧录

仓库没有固定烧录脚本。常见选择：

- STM32CubeProgrammer：加载 `ROLLERCAN.elf`、`.hex` 或 `.bin`。
- ST-LINK CLI/OpenOCD：按本地调试器配置烧录。
- MDK：打开 `MDK-ARM/ROLLERCAN.uvprojx` 后使用 IDE 的下载功能。

如果烧录 `.bin`，注意链接脚本和 `IAP_Set()` 暗示应用向量表使用 `APPLICATION_ADDRESS = 0x08002000`。确认启动链路时要同时检查 bootloader、烧录偏移和向量表 remap 逻辑。

## 尺寸分析

链接器已打开：

```text
-Wl,-Map=ROLLERCAN.map
-Wl,--gc-sections
-Wl,--print-memory-usage
```

常用检查：

```powershell
arm-none-eabi-size -A build\Debug\ROLLERCAN.elf
arm-none-eabi-nm --print-size --size-sort --radix=d build\Debug\ROLLERCAN.elf
```

U8g2 字体和图标是 Flash 占用的重要来源。新增字体、图标或 U8g2 源文件前，建议先通过 map/size 确认实际链接影响。
