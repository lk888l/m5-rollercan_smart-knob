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

`FLASH_DATA_SIZE = 48`。0-36 保留原协议配置布局，37-43 和 47 保存本地 SmartKnob 配置。

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
| 29 | comm_type | 本地直运行固定为 `COMM_TYPE_NONE`；旧协议值仍可读取迁移 |
| 30 | speed_pid_index | 速度 PID 参数组 |
| 31 | pos_pid_index | 位置 PID 参数组 |
| 32 | bps_index | CAN 波特率索引 |
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

旧页面没有合法的 magic/version/marker/checksum 时，固件只在 RAM 中使用 `COARSE+ / 100% / 450 mA / 8°` 默认值，不会仅为迁移主动擦写 Flash。用户第一次从本地菜单保存时写入新布局。原 byte 2-3 编码器 offset 会继续保留。

## 写回触发

`flash_data_write_back()` 会把当前全局配置序列化到 `flash_data` 并写入 Flash。

触发路径：

| 来源 | 条件 |
| --- | --- |
| 本地菜单 | 根菜单长按或选择 `SAVE`，保存模式、力度、挡位角和电流上限 |
| 旧 I2C/CAN 协议代码 | 相关入口仍在源码中，但本地直运行构建不初始化 FDCAN/I2C 从机，也不创建 CommunicationTask |

本地菜单退出时会先将电流目标清零并关闭驱动，再设置 `flash_data_write_back_flag`。StorageTask 在 `SYS_RUNNING` 之外擦写并校验 Flash；完成后按照进入菜单前的运行/暂停状态决定是否重新启动力反馈。

编码器校准 helper 和旧 `CAL` 页面仍保留在源码中，但当前本地菜单没有暴露校准项，避免普通运行时误触发电机主动转动。需要重新校准时应通过调试流程显式执行，并确认输出轴空载。

## 写入注意事项

- Flash 写入会擦除整个 page 59。
- `writeMessageToFlash()` 当前只按 `length/8` 写完整 doubleword；`FLASH_DATA_SIZE=48` 可以被 8 整除，所以当前布局安全。
- 新增保存字段时应保持长度 8 字节对齐，或补上尾部非 8 字节写入逻辑。
- 写回前会把 `MODE_SPEED_ERR_PROTECT`/`MODE_POS_ERR_PROTECT` 改回普通速度/位置模式，避免下次开机停在保护态。
