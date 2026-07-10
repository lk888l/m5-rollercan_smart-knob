# 显示、按键与灯效

## OLED 初始化

OLED 由 `MyFile/src/oled_u8g2.c` 和 `MyFile/src/u8g2_disp_fun.c` 驱动。

底层：

- `u8g2_Setup_ssd1306_64x48_er_f()`
- 分辨率 64x48。
- `U8G2_R2` 旋转。
- 软件 4-wire SPI。

启动显示：

1. `u8g2Init(&u8g2)` 初始化驱动和显示。
2. `u8g2_disp_init()` 显示 logo 约 1 秒。
3. 绘制左侧页面指示、状态区和基础 UI。

## 常驻页面

`dis_show_flag` 定义在 `mysys.h`：

| 页面 | 值 | 函数 | 内容 |
| --- | --- | --- | --- |
| `DIS_INFO` | 0 | `u8g2_disp_info()` | 通信模式、模式、ID/地址、电压等信息 |
| `DIS_GRAPHY` | 1 | `u8g2_disp_all()` | 速度、位置、电流图形化显示 |
| `DIS_CHAR` | 2 | `u8g2_disp_char()` | 当前模式的主要数值字符显示 |
| `DIS_PID` | 3 | `u8g2_disp_pid()` | 当前模式对应 PID 参数 |
| `DIS_OVP` | 5 | `u8g2_disp_ovp()` | 过压错误页 |
| `DIS_STALL` | 6 | `u8g2_disp_stall()` | 堵转错误页 |
| `DIS_OVER_VALUE` | 7 | `u8g2_disp_over_value()` | 位置越界错误页 |

主循环中，短按按键在 `DIS_INFO` 到 `DIS_PID` 之间循环；一旦存在错误，`LoopMysys()` 会强制显示对应错误页。

## 状态栏

`u8g2_disp_update_mode()`：

- 左上角显示模式字母：
  - `S`：速度模式。
  - `P`：位置模式。
  - `C`：电流模式。
  - `E`：Dial/encoder 模式。
- 运行状态下随 `running_index` 闪烁。

`u8g2_disp_update_comm()`：

- 左下角显示 `I2C` 或 `CAN`。
- `comm_flash_count` 非 0 时做反色闪烁。
- I2C 通信把 RGB 闪烁色设置为紫色系，CAN/CAN-I2C 设置为蓝绿色系。

## 按键

按键模块在 `MyFile/src/button.c`：

- 输入引脚：`SYS_SW_Pin`，PC6，内部上拉。
- `read_button_status()` 做简单稳定计数滤波。
- `button_update()` 输出事件：
  - `was_click`：按下 100-500ms 后释放。
  - `was_longpress`：按下 2-5s 后释放。
  - `is_longlongpressed`：持续按下超过 5s 的即时事件。
  - `was_longlongpress`：超过 5s 后释放。

主循环使用方式：

- `was_click`：切换常驻显示页。
- `is_longlongpressed` 且 `mode_switch_flag` 为 1：循环切换运行模式。

## 配置菜单

开机时按住 `SYS_SW`，`InitMysys()` 会进入 `u8g2_disp_menu_init()` 和 `u8g2_disp_menu_update()`。

菜单运行特点：

- 菜单期间调用 `MotorDriverSetMode(MDRV_MODE_RUN)`，并把 `motor_mode` 临时切到 `MODE_DIAL`。
- `handle_smart_knob()` 在 TIM1 控制分支里继续运行。
- `u8g2_disp_menu_update()` 通过 `current_position` 的增减映射成 `encoder_up/down`，用旋钮导航。
- 单击确认或退出。
- 退出时关闭电机，恢复 Flash 中保存的原模式。

主菜单项：

| 菜单项 | 函数 | 保存内容 |
| --- | --- | --- |
| Quit | `u8g2_disp_menu_0_1()` | 退出菜单 |
| COM | `u8g2_disp_menu_2_1()` | I2C / CAN / CAN->I2C |
| I2C ADDR | `u8g2_disp_menu_3_1()` | 7-bit I2C 地址 |
| CAN ID | `u8g2_disp_menu_1_1()` | CAN 节点 ID |
| POS PID | `u8g2_disp_menu_4_1()` | 位置 PID 参数组索引 |
| SPEED PID | `u8g2_disp_menu_5_1()` | 速度 PID 参数组索引 |
| BPS | `u8g2_disp_menu_6_1()` | CAN 波特率索引 |
| RGB% | `u8g2_disp_menu_7_1()` | RGB 亮度 |
| RGB | `u8g2_disp_menu_8_1()` | 系统默认/用户自定义 RGB |
| JAM | `u8g2_disp_menu_9_1()` | 堵转保护开关 |
| RANGE | `u8g2_disp_menu_10_1()` | 位置越界保护开关 |

确认菜单设置时，代码会短暂恢复 `flash_data[1]` 的电机模式并调用 `flash_data_write_back()`，再重新进入 Dial 菜单模式。

## RGB 灯效

灯效由 `MyFile/src/ws2812.c` 和 `u8g2_disp_fun.c` 控制。

底层：

- `PIXEL_MAX = 2`。
- `neopixel_set_color(num, color)` 接收 `0xRRGGBB`。
- 实际发送按 SK6812/WS2812 的 GRB 顺序展开。
- `brightness_index` 作为 0-100 百分比缩放。
- `ws2812_show()` 使用 TIM3 CH2 PWM-DMA 发送。

系统默认颜色：

| 状态/模式 | 颜色 |
| --- | --- |
| 错误 | 红色 `0x320000` |
| 速度模式 | 绿色 `0x003200` |
| 位置模式 | 蓝色 `0x000032` |
| 电流模式 | 黄绿色 `0x323200` |
| Dial 模式 | 紫色 `0x100010` |

`rgb_flash_flag` 为 1 时闪烁；`rgb_flash_slow` 为 1 时慢闪，否则快闪。`rgb_show_mode=1` 时外部写入的用户颜色队列会覆盖系统默认灯效。
