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
  -> Loop_FOC
       -> 消费最新驱动模式和电流目标
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

1. `EncoderGetAngle()` 读取 TLE5012B 原始角度。
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
- 实际限幅为 `[-1200, 1200]` mA。

## 编码器校准

底层模式 `MDRV_MODE_ENC_CAL` 会让 `MotorDriverProcess()` 进入校准分支：

1. 第一次进入时清计时器。
2. 约 28000 个 FOC 周期内固定 `eAngle_360 = 0`，并设置 `iq_curr_pi_target = 1200.0f`。
3. 结束时记录当前 `angle_get` 为 `motor_driver_cal_encoder_offset`。
4. 清电流目标，关闭驱动，退出校准。

外部可通过 I2C `0xF1` 启动校准，通过 `0xF3` 读取忙状态，通过 `0xF2` 保存校准 offset。

## 机械角度与速度估计

`Loop_Control()` 做机械侧状态估计：

- 每个控制步读取一份一致的 `FastSensorSnapshot`；若读取与 FOC 更新碰撞，则保留上一份有效快照。

- `MotorDriverGetMechanicalAngle()` 返回单圈角度，单位为 0.1 度。
- 通过跨越 0/360 度边界判断 `mechanical_turns` 增减。
- `mechanical_angle = 360 * mechanical_turns + encoder_absolute_angle_new`。
- `mechanical_rad = mechanical_angle * PI / 180`。
- `speed_encoder_update()` 维护展开后的编码器计数。
- 计数差按 1 kHz 调用率低通后换算：
  - `motor_rpm = diff / 16383 * (60 * 1000)`
  - `motor_rps = diff / 16383 * (2 * PI * 1000)`

速度、电流、电压和温度低通的系数已按 1 ms 采样周期重新计算。速度/位置 PID 仍保存原协议中的离散增益，但运行系数按原约 5.09 kHz 到 1 kHz 的周期比例转换：Ki 约乘 5.09，Kd 约除 5.09。

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
- 电流同时受模式限幅、0–1000 ‰ 安全比例和底层 1.2 A 硬限制约束，最后调用 `MotorDriverSetCurrentReal()`。
- 低速时慢慢修正 detent 中心；编码器跳变、进入模式后的稳定期以及高速转动时输出 0。
- `smart_knob_modes.h` 的 `SMART_KNOB_DEFAULT_MODE` 是唯一的默认预设选择点；各预设配置互相独立。
- CAN 参数切换模式或修改手感仍在 ControlTask 执行；CommunicationTask 只读取一致快照并主动发送遥测。

模式表、CAN function、参数缩放和主动遥测帧详见 [固件 SmartKnob](smartknob-firmware.md)。

## 保护逻辑

| 保护 | 条件 | 行为 |
| --- | --- | --- |
| 过压 | `vol_lpf > 1800` | 设置 `ERR_OVER_VOLTAGE`，关闭驱动，显示 OVP |
| 过压恢复 | `vol_lpf <= 1750` | 清过压错误，回到待机 |
| 位置越界 | `abs(mechanical_angle * 100) > MY_INT32_MAX` 且保护开启 | 设置 `ERR_OVER_VALUE`，关闭驱动，显示 RANGE |
| 速度堵转 | 速度误差超过阈值且电流超过 500mA 持续超时 | 进入 `MODE_SPEED_ERR_PROTECT` |
| 位置堵转 | 位置误差超过阈值且电流超过 500mA 持续超时 | 进入 `MODE_POS_ERR_PROTECT` |

堵转保护态会尝试自动恢复，恢复尝试次数由 `err_recover_try_max` 限制。当前菜单和协议能开关堵转保护，但最大尝试次数等部分 CAN function index 只定义了枚举，未完整接入写分支。
