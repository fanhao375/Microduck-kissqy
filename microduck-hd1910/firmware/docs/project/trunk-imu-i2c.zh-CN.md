# 躯干 IMU 改接主板 I²C：实施记录

2026-10-11，分支 `hd1910-trunk-imu-i2c`，基于 kissqy/Microduck-kissqy `94328f5`（R17 v1.0.146）。由 microduck-replica 项目起草计划并实施。

**机制本身以 [`design/robotd-design.md`](../design/robotd-design.md) §2.6 为准。** 本页是那一天的实施记录：做了什么、和计划哪里不同、还有什么没在真机上验证过。它会过时，这是有意的。

## 为什么

R17 的 robotd 只认一种躯干 IMU：舵机总线上的 FT6 / imu_to_dxl 小板（ID 200）。很多复刻者手上是现成模块，直接接 Radxa Zero 3W 的 I²C。本分支加两种接法，`[trunk_imu] source` 三选一：

| 源 | `source` | 硬件 | 地址 |
|---|---|---|---|
| A | `bus`（默认，行为不变） | 总线小板 ID 200 | — |
| B | `bno08x` | BNO085/BNO086 模块 | 0x4A / 0x4B（默认 0x4B） |
| C | `lsm6dsv16x` | LSM6DSV16X 转接板（与小板同芯片） | 0x6A / 0x6B（必须显式填写） |

## 实施步骤

1. A 的黄金测试先行，再把 `SflpDecoder` 拆成 `TrunkFrame` + FT6 字节解析；黄金表原样通过。
2. `robotd-params` 加 `[trunk_imu]`，按源校验地址，模块源要求 `bus.protocol = "feetech"`。
3. `duck-control/src/bno08x.rs`：SHTP/SH-2 纯编解码与 `Bno08xAssembler`（移植 servo-web `test_imu_i2c.py` 的用例）。
3b. `duck-control/src/lsm6dsv16x.rs`：小板固件 `imu.c` 的 `configure_one_step` 逐条落到寄存器（引用 ST 驱动行号）做成 `BRING_UP` 数据表；FIFO 解码复用 A 的陀螺与半精度换算，同样字节逐位相同。
4. `duck-control/src/trunk_imu.rs`：`TrunkImuKind`、`HostImuSample`、`HostImu`、`HostImuGate`。
5. 标定按源存 mount：`[imu_mounts.bno08x]`、`[imu_mounts.lsm6dsv16x]`；只有 A 的旧文件读写不变。
6. `FeetechIo` 内部 `TrunkImu { Bus, Host }`；模块源只 Sync Read 15 颗舵机，严格模式。
7. `robotd/src/trunk_imu_i2c.rs`：读取线程与两个状态机，假总线测试。
8. `robotd/src/main.rs`：读取器只建一次，所有启动路径共用；按源 `validate_motion_for`；模块源统计不含 200。
9. 调试与只读模式的模块分支；`verify-imu` 写当前源槽位。模块源分两步：直立时倾角 ≤ 10°，再在 20 秒内把机身前倾 15–30° 并保持，重力须读成朝 +x（偏离正前 ≤ 20°，连续 5 帧）才写，并回显两步的重力、倾角、陀螺。
10. `packaging`：drop-in 加 `DeviceAllow=char-i2c rw`；`calibration-config.py` 按 robotd.toml 的源检查；重装保留 `imu_mounts`；运行回读核对模块 mount。

## 与计划的出入

- 第 4 步先于第 3 步提交（编解码器产出的是第 4 步的类型）。
- `HostImuSample` 带 `epoch_started_at`（稳定期从读取线程开新 epoch 起算）和 `accuracy`（BNO08x 的精度位，只给人看）。`HostImu` 要求 `Send + Sync`。
- epoch 变化只重置 gate，不清 `TrunkFrame` 的就绪计数，与 A“读丢不算重新收敛”一致；重新收敛由 1 秒稳定期兜住。
- C 的 FIFO 字原样传递（不再归一化），与 A 一致，由 `TrunkFrame` 统一归一化。
- 计划原文放在 `design/` 下，按仓库“一页一机制”的规则改为本页（`project/` 下的记录），机制写进 `robotd-design.md` §2.6。

## 评审后的修正（2026-10-12）

- 只验直立证明不了水平轴：绕竖直轴转 90°/180° 的 mount 照样“直立”，上机后俯仰被读成横滚或反号。`verify-imu` 加了前倾一步（见上第 9 步）。
- FIFO / 报告积压不再当成新数据发布：只有读到队头的那一拍才发布最新样本；LSM 积压超过 48 字（约 133 ms）清空并开新 epoch。
- 单次 I²C 失败或一个坏包只丢弃计数（`dropped`），连续 5 次传输失败或无新数据看门狗才重开。
- 新 epoch 的第一帧不再与断开前的两帧取中值。
- 两处错误文本的续行反斜杠丢失已修；重装时模块源不再把总线槽位从快照补成已确认。
- 控制台调试页的 IMU 来源按 busStatus 报的源显示（FT6 / BNO08x / LSM6DSV16X），不再写死 FT6。

## 尚未在真机验证

- 没有上过机。BNO085 和 LSM6DSV16X 模块都只对假总线跑过；i2c-dev 的两段读取、`I2C_RDWR` 寄存器读、BNO 的时钟拉伸都要实测。
- C 的 FIFO 是否支持跨字连读未验证；读取线程按固件做法每字一次 7 字节读取，解码器接受任意切分。
- BNO 的 Game RV 与 SFLP 融合不同，落脚冲击时重力瞬态可能不同——上机前做影子对比（robotd 跑 A 时用 servo-web 的 `imu_i2c.py` 同时记 B）。
- I²C 受舵机电流干扰时会重开，约 0.5 s 起的退避空窗加 1 秒稳定期，控制环先 coast 再 hold。
- 新板要启用对应 I²C overlay；servo-web 占着节点时 flock 失败，报错会点名。
- 升级时 `configure.py` 的合并保留 `[trunk_imu]`（有测试）；其他重建路径（`rebuild-config.py` 从 `.saved` 副本恢复）照搬整个文件，也应保留，但没有在板上走过一遍。
