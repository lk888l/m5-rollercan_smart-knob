# 控制链路

## 电机控制总体结构

```text
通信/菜单写入目标值
  -> speed_point / pos_point / current_point
  -> TIM1 中断调度
       -> Loop_FOC
            -> MotorDriverProcess
               -> 编码器角度
               -> Clarke/Park
               -> 电流 PI
               -> Inverse Park
               -> SVM
               -> TIM1 CCR1/2/3
            -> MyAdcProcess
       -> Loop_Control
            -> 多圈机械角度
            -> 转速估计
            -> 电流/电压/温度低通
            -> 过压/越界/堵转保护
       -> 模式控制
            -> speed_pid / pos_pid / current / handle_smart_knob
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

- `MotorDriverGetMechanicalAngle()` 返回单圈角度，单位为 0.1 度。
- 通过跨越 0/360 度边界判断 `mechanical_turns` 增减。
- `mechanical_angle = 360 * mechanical_turns + encoder_absolute_angle_new`。
- `mechanical_rad = mechanical_angle * PI / 180`。
- `speed_encoder_update()` 维护展开后的编码器计数。
- 计数差低通后换算：
  - `motor_rpm = diff / 16383 * 336000`
  - `motor_rps = diff / 16383 * 2016000 * PI / 180`

## 速度模式

速度模式入口在 `mysys_tim1_update_handler()`：

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

位置模式入口：

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

TIM1 模式分支里，电流模式主要根据 `ph_crrent_lpf / current_point_float` 控制 RGB 快慢闪状态。

## Dial/SmartKnob 模式

Dial 模式在 `MyFile/src/smart_knob.c`：

- `init_smart_knob()` 将当前机械弧度设为 detent 中心，并重置 PID 状态。
- `handle_smart_knob()` 使用 `mechanical_rad`、`motor_rps` 和 `current_detent_center` 计算力反馈。
- 低速且靠近 detent 时会慢慢修正 detent 中心。
- 旋转超过 snap point 后更新 `current_position`。
- 超过速度阈值时输出 0，避免高速时正反馈。
- 否则通过 `knob_pid()` 计算扭矩并调用 `MotorDriverSetCurrentReal(torque)`。

默认配置：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `min_position` | 0 | 下限 |
| `max_position` | -1 | 小于 min，表示无界 |
| `position_width_radians` | 约 8.23 度 | detent 间距 |
| `detent_strength_unit` | 2 | detent 强度 |
| `snap_point` | 1.1 | 越过该比例后换档 |

## 保护逻辑

| 保护 | 条件 | 行为 |
| --- | --- | --- |
| 过压 | `vol_lpf > 1800` | 设置 `ERR_OVER_VOLTAGE`，关闭驱动，显示 OVP |
| 过压恢复 | `vol_lpf <= 1750` | 清过压错误，回到待机 |
| 位置越界 | `abs(mechanical_angle * 100) > MY_INT32_MAX` 且保护开启 | 设置 `ERR_OVER_VALUE`，关闭驱动，显示 RANGE |
| 速度堵转 | 速度误差超过阈值且电流超过 500mA 持续超时 | 进入 `MODE_SPEED_ERR_PROTECT` |
| 位置堵转 | 位置误差超过阈值且电流超过 500mA 持续超时 | 进入 `MODE_POS_ERR_PROTECT` |

堵转保护态会尝试自动恢复，恢复尝试次数由 `err_recover_try_max` 限制。当前菜单和协议能开关堵转保护，但最大尝试次数等部分 CAN function index 只定义了枚举，未完整接入写分支。
