# 持久化配置

## Flash 存储模块

Flash 读写在 `Core/Src/flash.c`：

- 使用 page 59。
- 起始地址宏为 `STM32G0xx_FLASH_PAGE59_STARTADDR`，实际地址是 `0x08000000 + 59 * 0x800`。
- 数据包头为 `0xAA55`。
- 头部 doubleword 同时保存 package head 和 length。
- 数据从起始地址 + 8 开始按 8 字节 doubleword 写入。

命名里仍保留 `STM32G0xx`，但当前工程目标芯片是 STM32G431，维护时按实际 Flash page 大小和链接脚本复核。

## 默认配置和读取

`init_flash_data()` 在 `InitMysys()` 中调用。

如果 Flash 中没有合法 package：

1. 使用编译期默认值填充 `flash_data`。
2. 只在 RAM 中建立默认的完整 SmartKnob 快照，不在启动过程中擦写 page 59。

如果 Flash 中有合法 package：

1. 记录实际 package 长度并读取到 512 字节 `flash_data` 缓冲。
2. 反序列化 I2C 地址、模式、编码器 offset、CAN ID、PID、通信模式、波特率、亮度、RGB、保护开关。
3. 应用编码器 offset。
4. 将整数 PID 转为浮点 PID。
5. 旧保护态值先按兼容逻辑归一化；产品运行路径随后统一进入 `MODE_DIAL`。

## Flash 数据布局

`FLASH_DATA_SIZE = 512`，数据包总占用为 8 字节 package header + 512 字节 data，远小于 page 59 的 2048 字节容量。前 48 字节保持旧布局，37-43 和 47 继续保存可供旧固件读取的基础 SmartKnob 投影。

| Byte | 内容 | 说明 |
| --- | --- | --- |
| 0 | I2C address | 7-bit 地址 |
| 1 | motor_mode | 降级兼容投影，写回固定为安全的 `MODE_DIAL` |
| 2-3 | angle_cal_offset | 编码器校准 offset，little-endian uint16 |
| 4 | can_id | CAN 节点 ID |
| 5-8 | speed Kp int | little-endian uint32 |
| 9-12 | speed Ki int | little-endian uint32 |
| 13-16 | speed Kd int | little-endian uint32 |
| 17-20 | position Kp int | little-endian uint32 |
| 21-24 | position Ki int | little-endian uint32 |
| 25-28 | position Kd int | little-endian uint32 |
| 29 | comm_type | 写回固定为 `COMM_TYPE_CAN`；启动时忽略旧通信模式 |
| 30 | speed_pid_index | 速度 PID 参数组 |
| 31 | pos_pid_index | 位置 PID 参数组 |
| 32 | bps_index | 写回固定为 0，即 1/5 Mbit/s CAN-FD+BRS |
| 33 | brightness_index | RGB 亮度百分比 |
| 34 | rgb_show_mode | 系统默认/用户自定义 |
| 35 | motor_stall_protection_flag | 堵转保护开关 |
| 36 | motor_overvalue_protection_flag | 位置越界保护开关 |
| 37 | local magic | 固定 `0x4C` |
| 38 | local profile version | 当前为 `1` |
| 39 | SmartKnob mode | 0-11 用户预设索引 |
| 40 | force percent | 25-125 |
| 41 | current limit / 10 mA | 10-45，即 100-450 mA |
| 42 | step width degrees | 1-60° |
| 43 | local checksum | `0xA5` 与 byte 39-42 的 XOR |
| 44-46 | 保留 | 当前写为 0/保持原值 |
| 47 | local marker | 固定 `0xD1` |

旧页面没有合法的 magic/version/marker/checksum 时，固件只在 RAM 中使用 `COARSE+ / 100% / 450 mA / 10°` 默认值。合法的旧四字段 profile 会先覆盖对应模式的默认值，再扩展成完整 RAM 快照；升级本身不会擦写 Flash。原 byte 2-3 编码器 offset 和 byte 4 CAN ID 会继续保留。

### 版本化扩展区

| Byte | 内容 |
| --- | --- |
| 48-51 | little-endian magic `SKN2` |
| 52 | 扩展版本，当前为 1 |
| 53 | 模式数，固定 12 |
| 54 | 当前模式 |
| 55 | flags，当前为 0 |
| 56-57 | payload 长度，固定 440 |
| 58-59 | 保留 |
| 60-63 | CRC32 |
| 64-447 | 12 个模式记录，每个 32 字节 |
| 448-503 | Custom 的 14 个 function 值 |
| 504-511 | 保留并写 0 |

每个模式记录依次保存挡位宽度（毫度）以及 `P`、`D`、`current_scale`、`current_limit`、`max_current_permille`、`friction`、`click` 七项 tuning。Custom 区按 `0x8201..0x820E` 的协议单位保存 position/min/max、宽度、detent/endstop、snap/bias、click/friction、scale、P/D 和 LED hue。除协议本来就是整数的 position、hue 和 permille 外，数值沿用 CAN function 的 `int32 × 1000` 表示，不直接保存带编译器填充的 C 结构体。

CRC32 覆盖扩展 header（不含 CRC 字段）和 440 字节 payload。magic、版本、长度、模式数、当前模式或 CRC 任一无效时，固件丢弃扩展区并回退到前 48 字节；不会使用半损坏的高级参数。

## 写回触发

`flash_data_write_back()` 会把当前全局配置序列化到 `flash_data` 并写入 Flash。

触发路径：

| 来源 | 条件 |
| --- | --- |
| 本地菜单 | 根菜单长按或选择 `SAVE`；只覆盖 dirty mask 标记的基础字段，随后保存完整快照 |
| 上位机 Stop | 配置至少成功 Dial 使能过一次；保存最近有效完整快照 |
| 上位机断线 | 连续 3 秒无有效报文；保存最近有效快照，半成品事务回退 |
| 兼容协议保存/ID 重配 | 延迟到驱动关闭，并使用上一次有效完整快照保护扩展区 |

本地菜单退出、上位机 Stop 或断线保存都会先清零电流并关闭驱动。StorageTask 仅在 `motor_output=0`、不处于 `SYS_RUNNING` 且 FOC 电流环已实际关闭时擦写，并回读 package header 与全部数据；成功后才更新 RAM 中的“已持久化快照”并按策略恢复输出。写入或回读失败会清除恢复请求、保留待重试快照和 `SAVE ERR`，并保持电机安全停机。

编码器校准 helper 和旧 `CAL` 页面仍保留在源码中，但当前本地菜单没有暴露校准项，避免普通运行时误触发电机主动转动。需要重新校准时应通过调试流程显式执行，并确认输出轴空载。

## 写入注意事项

- Flash 写入会擦除整个 page 59。
- `writeMessageToFlash()` 拒绝零长度、非 8 字节对齐或连同 8 字节 package header 超出 page 59 的数据；当前 512 字节布局满足约束。
- page 59 起始地址为 `0x0801D800`，应用链接区截止于 `0x0801D7FF`；构建后应继续检查 map，避免代码侵入配置页。
- byte 1 始终写成 `MODE_DIAL`，保证新固件重启和旧固件降级时都不会落在运行保护态。
