# 通信协议

## 通信模式

`comm_type` 定义在 `MyFile/inc/u8g2_disp_fun.h`：

| 值 | 名称 | 行为 |
| --- | --- | --- |
| 0 | `COMM_TYPE_NONE` | 未使用 |
| 1 | `COMM_TYPE_I2C` | 本机启用 I2C 从机协议 |
| 2 | `COMM_TYPE_CAN` | 本机启用 CAN 协议 |
| 3 | `COMM_TYPE_CAN_I2C` | 本机同时启用 I2C 和 CAN，CAN 可桥接 I2C |

当前产品运行时固定使用 `COMM_TYPE_CAN`。`InitMysys()` 仍读取旧布局以迁移编码器、PID 和 CAN ID，但会覆盖旧 `comm_type` 与 `bps_index`，随后以 1 Mbit/s 仲裁段、5 Mbit/s 数据段启动 FDCAN。I2C 桥接命令 `19–22` 保留，且不会触发本机 OLED 控制权接管。

## I2C 从机协议

I2C 从机由 `Core/Src/i2c_ex.c` 处理收发，中断完成后回调 `Core/Src/main.c` 的 `Slave_Complete_Callback()`。

写事务格式：

```text
[寄存器地址][数据0][数据1]...
```

读事务格式：

```text
先写 1 字节寄存器地址，然后主机读 N 字节
```

寄存器表：

| 地址范围 | 长度 | 读/写 | 含义 | 缩放/备注 |
| --- | --- | --- | --- | --- |
| `0x00` | 1 | R/W | 电机输出开关 | 0 关闭，非 0 运行 |
| `0x01` | 1 | R/W | 运行模式 | 1 速度，2 位置，3 电流，4 Dial |
| `0x0A` | 1 | R/W | 位置越界保护开关 | 0/1 |
| `0x0C` | 1 | R | 系统状态 | `SYS_*` |
| `0x0D` | 1 | R | 错误码 | `ERR_*` bit |
| `0x0E` | 1 | R/W | 按键切换模式允许位 | `mode_switch_flag` |
| `0x0F` | 1 | R/W | 堵转保护开关 | 0/1 |
| `0x10` | 1 | R/W | CAN ID | 0-255 |
| `0x11` | 1 | R/W | CAN 波特率索引 | 0=1M, 1=500K, 2=125K |
| `0x12` | 1 | R/W | RGB 亮度百分比 | 0-100 |
| `0x20-0x23` | 4 | R/W | 位置模式最大电流 | int32 / 100 = mA |
| `0x30-0x32` | 3 | R/W | RGB 颜色 | R/G/B packed 到低 24 bit |
| `0x33` | 1 | R/W | RGB 模式 | 0 系统默认，1 用户自定义 |
| `0x34-0x37` | 4 | R | 输入电压 | int32，内部单位约为 0.01V |
| `0x38-0x3B` | 4 | R | 内部温度 | int32，摄氏度 |
| `0x3C-0x3F` | 4 | R/W | Dial 计数位置 | int32 |
| `0x40-0x43` | 4 | R/W | 速度目标 | int32 / 100 = RPM |
| `0x50-0x53` | 4 | R/W | 速度模式最大电流 | int32 / 100 = mA |
| `0x60-0x63` | 4 | R | 速度反馈 | int32 = `motor_rpm * 100` |
| `0x70-0x73` | 4 | R/W | 速度 Kp | int / 100000 |
| `0x74-0x77` | 4 | R/W | 速度 Ki | int / 10000000 |
| `0x78-0x7B` | 4 | R/W | 速度 Kd | int / 100000 |
| `0x80-0x83` | 4 | R/W | 位置目标 | int32 / 100 = degree |
| `0x90-0x93` | 4 | R | 位置反馈 | int32 = `mechanical_angle * 100` |
| `0xA0-0xA3` | 4 | R/W | 位置 Kp | int / 100000 |
| `0xA4-0xA7` | 4 | R/W | 位置 Ki | int / 10000000 |
| `0xA8-0xAB` | 4 | R/W | 位置 Kd | int / 100000 |
| `0xB0-0xB3` | 4 | R/W | 电流目标 | int32 / 100 = mA |
| `0xC0-0xC3` | 4 | R | 电流反馈 | int32 = `ph_crrent_lpf * 100` |
| `0xF0` | 1 | W | 保存配置到 Flash | 写 1 生效 |
| `0xF1` | 1 | W | 启动编码器校准 | 写非 0 生效 |
| `0xF2` | 1 | W | 保存编码器 offset | 写非 0 生效 |
| `0xF3` | 1 | R | 编码器校准忙状态 | 0/1 |
| `0xFD` | 1 | W | 复位设备 | 写 1 后反初始化外设并 `NVIC_SystemReset()` |
| `0xFE` | 1 | R | 固件版本 | 当前 `FIRMWARE_VERSION=2` |
| `0xFF` | 1 | R/W | I2C 地址 | 写入小于 128 的 7-bit 地址并保存 |

`0x70-0x7B` 和 `0xA0-0xAB` 读出时会根据 `speed_pid_index` / `pos_pid_index` 选择用户自定义、轻载、中载或重载 PID 参数组。写入只更新用户自定义参数组，并在当前索引为 0 时立即应用。

## CAN 扩展帧格式

CAN 使用 CAN-FD、BRS、Extended ID 和 8 字节数据；接收路径会拒绝格式、BRS、ID 类型或 DLC 不匹配的帧。发送 ID 由 `FDCAN1_Send_Msg()` 生成：

```text
identifier = (cmd_id << 24) | (option << 8) | can_id
```

接收时：

| 字段 | 提取方式 | 含义 |
| --- | --- | --- |
| `cmd_id` | `(id >> 24) & 0x1F` | 命令类型 |
| `cmd_para` | `(id >> 16) & 0xFF` | 命令参数 |
| `option` | `(id >> 8) & 0xFFFF` | 选项/事务号/状态字段 |
| `can_id` | `id & 0xFF` | 节点 ID |

接收队列保留完整的 16 位 `option`；现有命令处理器延续旧协议行为，使用其低 8 位作为事务/主机标识，高 8 位主要用于响应状态。

## CAN 命令表

| `cmd_id` | 数据 | 行为 |
| --- | --- | --- |
| 0 | 空 | 发现/应答，返回本机 CAN ID |
| 3 | 空 | 电机开，成功后返回状态反馈 |
| 4 | 空 | 电机关，成功后返回状态反馈 |
| 7 | ID 中 `cmd_para` | 设置本机 CAN ID，主循环随后重新初始化 CAN 并写 Flash |
| 9 | 空 | 清除堵转保护状态 |
| 10 | 空 | 请求保存 Flash |
| 11 | ID 中 `cmd_para` | 兼容应答；运行时仍固定返回索引 0（1/5 Mbit/s） |
| 12 | 空 | 打开堵转保护 |
| 13 | 空 | 关闭堵转保护 |
| 14 | 空 | 打开位置越界保护 |
| 15 | 空 | 关闭位置越界保护 |
| 17 | data[0:1] function index | 读取本机 function |
| 18 | data[0:1] function index, data[4:7] int32 | 写入本机 function |
| 19 | data[0:1] function index, data[2:3] I2C address | 通过 I2C 读取远端 ROLLERCAN function |
| 20 | data[0:1] function index, data[2:3] I2C address, data[4:7] int32 | 通过 I2C 写远端 ROLLERCAN function |
| 21 | data[0] I2C address, data[1] length | 原始 I2C 读，最多 8 字节 |
| 22 | ID 中 `cmd_para` I2C address/stop bit, data[0] length, data[1..] payload | 原始 I2C 写，最多 7 字节 |

## CAN function index

`Core/Inc/fdcan.h` 定义的 `FUNC_*`：

| Function | 值 | 对应含义 |
| --- | --- | --- |
| `FUNC_SAVE_FLASH` | `0x7002` | 保存配置 |
| `FUNC_REMOVE_PROTECTION` | `0x7003` | 清保护 |
| `FUNC_ON_OFF` | `0x7004` | 电机开关 |
| `FUNC_RUN_MODE` | `0x7005` | 运行模式 |
| `FUNC_CURRENT_SETTING` | `0x7006` | 电流目标 |
| `FUNC_SPEED_SETTING` | `0x700A` | 速度目标 |
| `FUNC_POSITION_SETTING` | `0x7016` | 位置目标 |
| `FUNC_POSITION_MAX_CURRENT` | `0x7017` | 位置最大电流 |
| `FUNC_SPEED_MAX_CURRENT` | `0x7018` | 速度最大电流 |
| `FUNC_SPEED_KP/KI/KD` | `0x7020-0x7022` | 速度 PID |
| `FUNC_POSITION_KP/KI/KD` | `0x7023-0x7025` | 位置 PID |
| `FUNC_READBACK_SPEED` | `0x7030` | 速度反馈 |
| `FUNC_READBACK_POSITION` | `0x7031` | 位置反馈 |
| `FUNC_READBACK_CURRENT` | `0x7032` | 电流反馈 |
| `FUNC_DIAL_COUNTER` | `0x7033` | Dial 计数 |
| `FUNC_VIN` | `0x7034` | 输入电压 |
| `FUNC_TEMP` | `0x7035` | 温度 |
| `FUNC_OVERVOLTAGE_PROTECTION_RELEASE_MODE` | `0x7040` | 过压释放模式：0 手动，1 电压稳定后自动软恢复 |
| `FUNC_RGB_MODE` | `0x7050` | RGB 模式 |
| `FUNC_RGB_COLOR` | `0x7051` | RGB 颜色 |
| `FUNC_RGB_BRIGHTNESS` | `0x7052` | RGB 亮度 |

`0x7040` 已接入 function read/write；最大堵转尝试次数、堵转阈值和超时等其他枚举仍未完整接入 CAN 读写分支。

## CAN 状态反馈

`feedback_option_data()` 会把状态打包到响应 option：

```text
low byte: my_can_id
high byte:
  bits 0..2: error_code & 0x07
  bits 3..5: motor_mode
  bits 6..7: sys_status
```

响应 data：

| data 字节 | 内容 |
| --- | --- |
| 0..1 | int16 `motor_rpm` |
| 2..3 | int16 `mechanical_angle` |
| 4..5 | int16 `ph_crrent_lpf` |
| 6..7 | int16 `vol_lpf / 100` |

function read 响应使用：

```text
data[0..1] = function index
data[4..7] = int32 value
```

## 固件 SmartKnob 扩展

`0x8001–0x8304` 为固件本地 SmartKnob 的模式、调参、主动遥测和状态读取 function。SmartKnob 打开主动遥测后，不需要上位机轮询角度或电流：固件会用 `cmd=0x17/0x18` 成对推送逻辑状态和运动/电流状态。

完整的 function 表、缩放、帧字段、模式索引和推荐启动顺序见 [固件 SmartKnob](smartknob-firmware.md)。

## 上位机控制权

- `cmd=0` Ping 与 `cmd=17` function read 在未连接时只做发现，不接管 OLED。
- 第一个受支持的本机写命令接管控制；CAN-I2C 桥接 `19–22` 不接管。
- 接管后，本机 Ping、read、write 都刷新最后有效报文时间。
- 连续 3000 ms 没有有效本机报文时释放控制权，并把遥测开关、速率和目标 host ID 恢复为固件默认值。
- 上位机 Stop 不等于断开：只要报文继续到达，OLED 仍锁定，电机保持失能。

上位机配置事务按以下顺序识别：

```text
write 0x7004 = 0
write 0x7006 = 0
write 0x8001 = mode
write custom/tuning parameters
write 0x7005 = MODE_DIAL
write 0x7004 = 1
```

只有最后一次使能在无故障状态下真正成功，事务才成为可保存快照。启动中途掉线会恢复上一次有效快照；正常 Stop（先清零电流，再失能）或 3 秒断线会触发安全写回。
