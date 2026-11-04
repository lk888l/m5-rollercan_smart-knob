# 控制链路

## 电机控制总体结构

```text
通信/菜单写入目标值
  -> speed_point / pos_point / current_point
  -> ControlTask（1 kHz）
       -> Loop_Control：角度/速度估计、低通和保护
       -> 模式控制：speed_pid / pos_pid / current / handle_smart_knob
       -> 发布 FastControlCommandSnapshot

TIM1 update ISR（约 18.67 kHz）
  -> 消费最新驱动模式和电流目标
  -> 启动 TLE5012B 两字 DMA 读取
  -> DMA2 RX 完成 ISR
       -> 提交本周期编码器角度
       -> Loop_FOC
       -> MotorDriverProcess
            -> 编码器角度
            -> Clarke/Park
            -> 电流 PI
            -> Inverse Park
            -> SVM
            -> TIM1 CCR1/2/3
       -> MyAdcProcess
       -> 发布 FastSensorSnapshot
```

## FOC 电流环

FOC 实现在 `MyFile/src/motordriver.c`。

关键步骤：

1. 使用 DMA2 RX 完成中断刚提交的 `EncoderGetLatestAngle()`；SPI 搬运不占用 CPU 轮询时间。
2. 用 `angle_offset` 修正机械零点。
3. 将机械角度换算为电角度 `eangle_get`，并加 90 度相位偏置。
4. `Clarke_Park(ia, ib, ic)` 将三相电流转换到 d/q 轴。
5. 如果 `currentloop_enable` 为 1，`CurrentLoopCalc()` 分别计算 Id/Iq PI。
6. `vbusLimitCalc()` 按母线电压限制 d/q 电压。
7. `InversePark()` 转回 alpha/beta。
8. `SVMGenerate()` 计算三相 PWM 占空比并写入 TIM1 CCR。

当前电流目标主要是 Iq：

- `id_curr_pi_target` 默认 0。
- `iq_curr_pi_target` 由 `MotorDriverSetCurrentAdc()` 或 `MotorDriverSetCurrentReal()` 设置。
- `MotorDriverSetCurrentReal(phase_current)` 的单位是 mA，内部乘以 1.25 得到 ADC/PI 目标。
- 普通电流、速度和位置模式保留 60 mA 静音区；SmartKnob 通过连续电流入口绕过该幅值死区，避免弹簧控制在阈值两侧反复通断。
- 目标严格等于零时，FOC ISR 同时清零 Id/Iq PI 状态和 `ud/uq`，避免电流环继续追踪零点采样偏置；遥测中的实测电流仍可能显示 ADC 的小幅零点噪声。
- 无摩擦模式正常运行时（包括静止）通过 `MotorDriverSetTorqueCurrent()` 发布同一快照中的零电流跟踪标志，即使 Iq 目标为 0 也继续运行 PI 以抵消反电动势。否则零电压 PWM 会产生电磁制动。故障、无效采样和退出本模式时仍走停止路径。
- 实际限幅为 `[-1200, 1200]` mA。

## 编码器校准

底层模式 `MDRV_MODE_ENC_CAL` 会让 `MotorDriverProcess()` 进入校准分支：

1. 第一次进入时清计时器。
2. 约 28000 个 FOC 周期内固定 `eAngle_360 = 0`，并设置 `iq_curr_pi_target = 1200.0f`。
3. 结束时记录当前 `angle_get` 为 `motor_driver_cal_encoder_offset`。
4. 清电流目标，关闭驱动，退出校准。

外部可通过 I2C `0xF1` 启动校准，通过 `0xF3` 读取忙状态，通过 `0xF2` 保存校准 offset。

本地按键设置路径也复用同一校准流程：进入菜单前先执行一次不保存的临时校准；菜单 `CAL` 在二次确认后重新校准、应用并保存 offset。两条本地路径都使用 3 秒超时，超时时会清除校准状态、电流目标并立即关闭驱动。

## 机械角度与速度估计

`Loop_Control()` 做机械侧状态估计：

- 每个控制步读取一份一致的 `FastSensorSnapshot`；若读取与 FOC 更新碰撞，则保留上一份有效快照。

- `motion_estimator.c` 参考 `stm32g431-foc-canfd` 的 `AngleTracker`，将机械运动估计从系统调度中分离，可在主机上独立测试。
- 位置与快速度共同使用快照中的完整 14-bit、已修正方向和零点的编码器计数，避免把 0.1 度显示角度用于触感控制。
- 按 16384 count 的最短差展开整数圈数，再由圈数和单圈绝对计数构造 `mechanical_rad`；首次采样不会误判跨圈。
- 只有新的有效快照会推进估计；速度和滤波系数使用 `micros()` 的实际间隔。重复时间不消费位移，超过 20 ms 的间隔清零速度并重新同步到最近圈角度。
- `mechanical_angle` 由 `mechanical_rad` 换算为度，编码器校准时统一重置运动估计。
- `speed_encoder_update()` 维护展开后的编码器计数。
- 计数差按 1 kHz 调用率低通后换算：
  - `motor_rpm = diff / 16383 * (60 * 1000)`
  - `motor_rps = diff / 16383 * (2 * PI * 1000)`

速度、电流、电压和温度低通的系数已按 1 ms 采样周期重新计算。速度/位置 PID 仍保存原协议中的离散增益，但运行系数按原约 5.09 kHz 到 1 kHz 的周期比例转换：Ki 约乘 5.09，Kd 约除 5.09。

所有 SmartKnob 模式及菜单导航均使用 `motor_rps_fast`，时间常数为 8 ms。阻尼、静止判定、高速保护和 SmartKnob 遥测使用同一路速度；传统速度 PID 和 RPM 读数继续使用原有 2 Hz `motor_rps` 路径。原先 coarse 使用约 80 ms 时间常数的慢速度，快速反弹时 D 项可能仍保留旧速度符号而继续加速。修复与实机记录见 [coarse 释放验证](coarse-release-validation.md)。

## 速度模式

速度模式入口在 1 kHz `MysysRunModeController()`：

```text
MODE_SPEED 且 SYS_RUNNING
  -> 根据 setpoint 生成堵转阈值
  -> speed_pid()
```

`speed_pid()`：

- `pid_ctrl_speed_t.input = motor_rpm`
- `pid_ctrl_speed_t.setpoint = speed_point / 100.0`
- `PIDCompute()` 输出电流目标，单位 mA。
- 输出写入 `MotorDriverSetCurrentReal(pid_ctrl_speed_t.output)`。
- 当启用堵转保护时，若速度误差大且电流大，持续超时后进入 `MODE_SPEED_ERR_PROTECT`。

通信缩放：

| 字段 | 缩放 |
| --- | --- |
| `speed_point` | 外部 int32 / 100 = RPM 目标 |
| `max_speed_current` | 外部 int32 / 100 = 最大电流 mA |
| 速度 Kp | int / 100000 |
| 速度 Ki | int / 10000000 |
| 速度 Kd | int / 100000 |

## 位置模式

位置模式同样由 1 kHz `MysysRunModeController()` 调用：

```text
MODE_POS 且 SYS_RUNNING
  -> 根据 setpoint 生成堵转阈值
  -> pos_pid()
```

`pos_pid()`：

- `pid_ctrl_pos_t.input = mechanical_angle`
- `pid_ctrl_pos_t.setpoint = pos_point / 100.0`
- `PIDCompute()` 输出电流目标。
- 输出写入 `MotorDriverSetCurrentReal(pid_ctrl_pos_t.output)`。
- 带有积分溢出保护和堵转保护。

通信缩放：

| 字段 | 缩放 |
| --- | --- |
| `pos_point` | 外部 int32 / 100 = 角度目标，单位度 |
| `max_pos_current` | 外部 int32 / 100 = 最大电流 mA |
| 位置 Kp | int / 100000 |
| 位置 Ki | int / 10000000 |
| 位置 Kd | int / 100000 |

## 电流模式

电流模式不跑外层 PID。通信写入 `current_point` 后：

```text
current_set = current_point / 100.0
MotorDriverSetCurrentReal(current_set)
```

`current_point` 被限制在 `[-120000, 120000]`，也就是 `[-1200.00, 1200.00]` mA。

ControlTask 的电流模式分支主要根据 `ph_crrent_lpf / current_point_float` 控制 RGB 快慢闪状态；目标为零时不会执行除法。

## Dial/SmartKnob 模式

Dial 模式由 `MyFile/src/smart_knob.c` 和 `MyFile/src/smart_knob_modes.c` 共同实现：

- `init_smart_knob()` 在首次进入时加载编译期默认预设，后续进入时保留活动预设并重新锚定 detent 中心。
- `handle_smart_knob()` 以 1 kHz 使用本机 `mechanical_rad` 和 `motor_rps` 更新 detent/endstop 状态机。
- 触感输出使用 `(P * position_error - D * velocity) * current_scale`，再叠加摩擦补偿和可选双相 click 电流。
- 电流同时受模式限幅、0–1000 ‰ 安全比例和底层 1.2 A 硬限制约束，最后调用 SmartKnob 专用的 `MotorDriverSetCurrentRealContinuous()`；其他控制模式仍调用带 60 mA 静音区的 `MotorDriverSetCurrentReal()`。
- 只有无边界模式会在低速静止后慢慢修正 detent 中心；有边界模式保持固定中心网格。编码器跳变或进入模式后的稳定期输出 0，跳变保护连续取得两个合理样本后会重新同步，不会停留在永久无效状态。
- 单中心回弹模式从 20 rad/s 开始削弱加速电流、45 rad/s 时完全取消加速电流；普通 SmartKnob 模式的高速保护在 180 rad/s 进入、120 rad/s 退出。对应的编码器增量校验范围为 240 rad/s，防止采样校验先于高速保护取消效果。1 kHz 控制步最多处理 16 次跨挡，覆盖高速下的 1° 细挡位。无摩擦模式 12 不设保持速度上限或按转速衰减助力，不使用普通模式的高速锁定；其编码器位置增量仍根据快速度估计校验，电流限幅与故障保护仍生效。实机验证见 [高速上限验证](smartknob-speed-limit-validation.md)及[无摩擦模式](frictionless-mode.md)。

无摩擦模式的平均损耗前馈经过 `frictionless_hold.c` 的柔性速度保持：滑行参考固定，PI 抵消平均损耗模型偏差；明显重新拨动时释放旧参考，重新获取速度。没有触摸传感器，切换依据运动估计。角度补偿和可选虚拟惯量在均值控制之外叠加，均值控制预算预留这两项电流。停止、换向、采样失效和故障均清除参考及积分。详见[无摩擦模式](frictionless-mode.md)。
- `smart_knob_modes.h` 的 `SMART_KNOB_DEFAULT_MODE` 是唯一的默认预设选择点；各预设配置互相独立。
- CAN 参数切换模式或修改手感仍在 ControlTask 执行；CommunicationTask 只读取一致快照并主动发送遥测。

模式表、CAN function、参数缩放和主动遥测帧详见 [固件 SmartKnob](smartknob-firmware.md)。

## 保护逻辑

| 保护 | 条件 | 行为 |
| --- | --- | --- |
| 过压 | `vol_lpf > 1800` | 设置 `ERR_OVER_VOLTAGE`，关闭驱动，显示 OVP |
| 过压恢复 | `vol_lpf <= 1750` | 清过压错误，回到待机；`0x7040=1` 时稳定 300 ms 后清零目标、重同步 Dial 并恢复驱动 |
| 位置越界 | `abs(mechanical_angle * 100) > MY_INT32_MAX` 且保护开启 | 设置 `ERR_OVER_VALUE`，关闭驱动，显示 RANGE |
| 速度堵转 | 速度误差超过阈值且电流超过 500mA 持续超时 | 进入 `MODE_SPEED_ERR_PROTECT` |
| 位置堵转 | 位置误差超过阈值且电流超过 500mA 持续超时 | 进入 `MODE_POS_ERR_PROTECT` |

过压自动释放默认关闭，`0x7040` 可在运行期读取或设置为 0/1。自动恢复只在电机输出请求仍有效、无其他错误且运行模式合法时执行；否则保持待机，等待显式重新使能。

堵转保护态会尝试自动恢复，恢复尝试次数由 `err_recover_try_max` 限制。当前菜单和协议能开关堵转保护，但最大尝试次数等部分 CAN function index 只定义了枚举，未完整接入写分支。
