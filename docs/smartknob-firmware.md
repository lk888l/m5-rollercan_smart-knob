# 固件 SmartKnob

## 目标与边界

SmartKnob 触感环完全运行在 STM32 固件中：ControlTask 以 1 kHz 读取本机机械角度和速度、更新 detent 状态机并直接给出 q 轴电流目标。上位机只负责切换预设、在线修改参数和显示固件主动上报的状态，不参与逐周期电流闭环。

这样 CAN 延迟、Windows 调度抖动或上位机暂时卡顿不会进入触感闭环。原有约 18.67 kHz FOC 电流环仍位于 TIM1 ISR，SmartKnob 不修改电流环和 PWM 实现。

## 模块

| 文件 | 职责 |
| --- | --- |
| `MyFile/inc/smart_knob.h` | 配置、调参、运行状态和 CAN 参数接口 |
| `MyFile/inc/smart_knob_modes.h` | 模式枚举和默认模式选择 |
| `MyFile/src/smart_knob_modes.c` | 每个模式独立的预设参数表 |
| `MyFile/src/smart_knob.c` | 1 kHz detent/endstop/current 算法、在线配置和遥测快照 |
| `Core/Src/fdcan.c` | CAN function 转发和主动遥测帧发送 |

默认 SmartKnob 预设只需要修改：

```c
// MyFile/inc/smart_knob_modes.h
#define SMART_KNOB_DEFAULT_MODE SMART_KNOB_MODE_COARSE_STRONG
```

固件的默认电机运行模式是 `MODE_DIAL`。Flash 中已有的合法运行模式仍会在启动时覆盖首次出厂默认值；SmartKnob 的预设和在线调参当前是运行期配置，复位后回到上述编译期默认预设。

## 内置模式

模式索引和 `SmartKnobMode` 枚举一致：

| 索引 | 枚举 | 用途 |
| --- | --- | --- |
| 0 | `SMART_KNOB_MODE_CUSTOM` | 上位机可编辑的自定义模式 |
| 1 | `SMART_KNOB_MODE_UNBOUNDED_SMOOTH` | 无边界、无刻度 |
| 2 | `SMART_KNOB_MODE_BOUNDED_SMOOTH` | 0–10，有端点、无刻度 |
| 3 | `SMART_KNOB_MODE_MULTI_REV_SMOOTH` | 多圈范围、无刻度 |
| 4 | `SMART_KNOB_MODE_ON_OFF_STRONG` | 两档强刻度 |
| 5 | `SMART_KNOB_MODE_RETURN_TO_CENTER` | 单中心回弹 |
| 6 | `SMART_KNOB_MODE_FINE_SMOOTH` | 细分值、无刻度 |
| 7 | `SMART_KNOB_MODE_FINE_DETENTS` | 细分值、带 click 电流脉冲 |
| 8 | `SMART_KNOB_MODE_COARSE_STRONG` | 粗分度强刻度，默认 |
| 9 | `SMART_KNOB_MODE_COARSE_WEAK` | 粗分度弱刻度，带 click |
| 10 | `SMART_KNOB_MODE_MAGNETIC` | 仅指定位置有磁吸刻度 |
| 11 | `SMART_KNOB_MODE_RETURN_TO_CENTER_DETENTS` | 带刻度的回中模式 |

每个模式都是独立的 `SmartKnobModeConfig`，包含上游 SmartKnob 的位置、范围、宽度、detent/endstop、snap 和磁吸位置字段，以及本电机的 P/D、电流缩放、电流限幅、摩擦补偿和 click 电流字段。修改一个模式不会改变其他模式。

## 电流计算和保护

固件使用以下电流型触感表达式：

```text
input = -angle_to_detent_center + dead_zone_adjustment
pid = clamp(P * input - D * shaft_velocity, -10, 10)
current = current_scale * pid + friction_current + click_current
```

- 到达 min/max 后，P 改用 `endstop_strength_unit * 4`。
- 磁吸模式在非指定位置不施加 detent 弹簧。
- click 使用 2 ms 正向 + 2 ms 反向的双相电流脉冲。
- 单中心回弹模式在 20–45 rad/s 区间逐步削弱与旋转同向的加速电流，保留反向阻尼电流，避免长行程回正持续加速。
- 速度达到 60 rad/s 后进入高速保护并把电流目标归零；只有速度降到 40 rad/s 后才退出，避免阈值附近反复启停。
- 电流同时受模式 `current_limit_a`、`max_current_permille` 和硬件 1.2 A 上限约束。
- 绝对值不超过 0.06 A 的目标进入输出死区。
- 编码器单步不连续或进入 Dial 后 300 ms 稳定期内，电流目标归零。无效样本仍会推进原始位置基线，连续两个合理样本后重新同步滤波位置，因此单次跳变或真实高速转动不会永久锁死触感输出。

## CAN 参数

仍使用原协议的 function read/write：`cmd=0x11` 读取，`cmd=0x12` 写入；function index 位于 `data[0..1]`，写值或读回值位于 `data[4..7]`，均为 little-endian `int32`。

### 模式与主动遥测

| Function | 含义 | 值 |
| --- | --- | --- |
| `0x8001` | 当前 SmartKnob 预设 | `0..11` |
| `0x8002` | 主动遥测开关 | 0 关闭，非 0 打开 |
| `0x8003` | 主动遥测频率 | 1–100 Hz，固件限幅 |
| `0x8004` | 遥测目标主机 ID | 0–255 |
| `0x8005` | 模式数量，只读 | 当前为 12 |
| `0x8006` | 遥测协议版本，只读 | 当前为 1 |

默认主动遥测打开、频率 50 Hz、目标主机 ID 为 0。写 `0x8002` 或 `0x8003` 时，固件也会记录该请求帧 ID 中的主机 ID；`0x8004` 可以显式覆盖目标 ID。

### 当前模式调参

除 `max_current_permille` 外，以下浮点值均按 `value / 1000` 解码：

| Function | 含义 | 单位 |
| --- | --- | --- |
| `0x8101` | P gain | - |
| `0x8102` | D gain | - |
| `0x8103` | PID 输出电流缩放 | A |
| `0x8104` | 模式电流限幅 | A |
| `0x8105` | 硬件电流限幅比例 | 0–1000 ‰ |
| `0x8106` | 摩擦补偿电流 | A |
| `0x8107` | click 脉冲电流 | A |

`0x8101–0x8107` 修改当前活动模式，并在本次运行中保留；切走再切回该模式仍使用修改后的值。

### 自定义模式

| Function | 含义 | 缩放 |
| --- | --- | --- |
| `0x8201` | position | int32 |
| `0x8202` | min_position | int32 |
| `0x8203` | max_position | int32 |
| `0x8204` | position width | degree × 1000 |
| `0x8205` | detent strength | × 1000 |
| `0x8206` | endstop strength | × 1000 |
| `0x8207` | snap point | × 1000 |
| `0x8208` | snap bias | × 1000 |
| `0x8209` | click current | A × 1000 |
| `0x820A` | friction current | A × 1000 |
| `0x820B` | current scale | A × 1000 |
| `0x820C` | P gain | × 1000 |
| `0x820D` | D gain | × 1000 |
| `0x820E` | LED hue | 0–255 |

## 主动遥测帧

每个采样周期主动发送两帧 Classic CAN 扩展帧。ID 布局为：

```text
bits 28..24  cmd: 0x17=logical state, 0x18=motion/current
bits 23..16  sequence: 两帧使用同一个递增序号
bits 15..8   source node ID
bits 7..0    destination host ID
```

`cmd=0x17` 数据：

| 字节 | 内容 |
| --- | --- |
| 0 | 活动预设索引 |
| 1 | flags：bit0 Dial、bit1 actual motor output、bit2 endstop、bit3 encoder valid、bit4 telemetry、bit5 high-speed latch、bit6 any fault、bit7 overvoltage |
| 2..5 | `current_position`, int32 |
| 6..7 | `sub_position_unit * 10000`, int16 |

`cmd=0x18` 数据：

| 字节 | 内容 |
| --- | --- |
| 0..3 | 展开机械角度，degree × 100，int32 |
| 4..5 | 指令电流，mA，int16 |
| 6..7 | 实测电流，mA，int16 |

上位机应按 sequence 配对两帧，但不应等待请求-响应；丢失某一帧时直接使用下一组最新状态。

bit1 来自 FOC 电流环的实际使能状态，而不是上位机最后写入的开关请求；因此过压保护关闭驱动后，bit1 会清零。bit7 可直接区分过压与其他 fault，输入电压仍可通过原协议 `0x7034` 读取。

## 推荐上位机启动顺序

```text
write 0x8001 = preset index       先选活动模式
write 0x8201..0x820E              若为 Custom，更新配置
write 0x8101..0x8107              更新当前模式手感和安全参数
write 0x7005 = 4                  进入 MODE_DIAL
write 0x8004 = host id
write 0x8003 = telemetry Hz
write 0x8002 = 1                  打开主动遥测
write 0x7004 = 1                  打开电机输出
```

停止电机使用 `0x7004 = 0`。遥测开关独立于电机输出，可继续上报停止状态，也可以由上位机写 `0x8002 = 0` 关闭。
