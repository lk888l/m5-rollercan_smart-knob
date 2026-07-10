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
2. 写入 page 59。

如果 Flash 中有合法 package：

1. 读取到 `flash_data`。
2. 反序列化 I2C 地址、模式、编码器 offset、CAN ID、PID、通信模式、波特率、亮度、RGB、保护开关。
3. 应用编码器 offset。
4. 将整数 PID 转为浮点 PID。
5. 如果保存的是保护态模式，则恢复为正常速度/位置模式。

## Flash 数据布局

`FLASH_DATA_SIZE = 48`，当前使用 0-36 字节。

| Byte | 内容 | 说明 |
| --- | --- | --- |
| 0 | I2C address | 7-bit 地址 |
| 1 | motor_mode | 运行模式，保护态会恢复为正常模式 |
| 2-3 | angle_cal_offset | 编码器校准 offset，little-endian uint16 |
| 4 | can_id | CAN 节点 ID |
| 5-8 | speed Kp int | little-endian uint32 |
| 9-12 | speed Ki int | little-endian uint32 |
| 13-16 | speed Kd int | little-endian uint32 |
| 17-20 | position Kp int | little-endian uint32 |
| 21-24 | position Ki int | little-endian uint32 |
| 25-28 | position Kd int | little-endian uint32 |
| 29 | comm_type | I2C/CAN/CAN-I2C |
| 30 | speed_pid_index | 速度 PID 参数组 |
| 31 | pos_pid_index | 位置 PID 参数组 |
| 32 | bps_index | CAN 波特率索引 |
| 33 | brightness_index | RGB 亮度百分比 |
| 34 | rgb_show_mode | 系统默认/用户自定义 |
| 35 | motor_stall_protection_flag | 堵转保护开关 |
| 36 | motor_overvalue_protection_flag | 位置越界保护开关 |
| 37-47 | 保留 | 当前未使用 |

## 写回触发

`flash_data_write_back()` 会把当前全局配置序列化到 `flash_data` 并写入 Flash。

触发路径：

| 来源 | 条件 |
| --- | --- |
| I2C | 写 `0xF0 = 1` |
| I2C | 写 `0xFF` 修改 I2C 地址 |
| I2C | 写 `0xF2 = 1` 保存编码器 offset |
| CAN | `cmd_id=10` 设置 `flash_data_write_back_flag` |
| CAN | `FUNC_SAVE_FLASH` 写非 0 |
| CAN | CommunicationTask 完成 CAN ID/波特率重配置后设置延后写回标志 |
| OLED 菜单 | 确认 COM、I2C ADDR、CAN ID、PID 组、BPS、RGB、JAM、RANGE 等设置 |

StorageTask 会检测 `flash_data_write_back_flag`，并在电机不处于 `SYS_RUNNING` 时写入，因此 CAN/CommunicationTask 不直接擦写 Flash。

## 写入注意事项

- Flash 写入会擦除整个 page 59。
- `writeMessageToFlash()` 当前只按 `length/8` 写完整 doubleword；`FLASH_DATA_SIZE=48` 可以被 8 整除，所以当前布局安全。
- 新增保存字段时应保持长度 8 字节对齐，或补上尾部非 8 字节写入逻辑。
- 写回前会把 `MODE_SPEED_ERR_PROTECT`/`MODE_POS_ERR_PROTECT` 改回普通速度/位置模式，避免下次开机停在保护态。
