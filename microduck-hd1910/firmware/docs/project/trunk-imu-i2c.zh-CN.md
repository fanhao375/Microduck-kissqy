<!-- Added 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; see IMU-MODULE.md at the repository root. -->
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
9. 调试与只读模式的模块分支；`verify-imu` 写当前源槽位。模块源分两步：直立时倾角 ≤ 5°（残余倾角作为 `residual_tilt_deg` 回显），再在 20 秒内把机身前倾 15–30° 并保持；前倾从直立读数算起，须 ≥ 12°、重力变化朝 +x（偏离正前 ≤ 20°），连续 5 帧读成前倾才写，连续 5 帧读成别的方向就拒绝，中途方向变了从头数。回显两步的重力、倾角、陀螺。
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

第二轮评审：

- 停滞看门狗放到每步之后统一检查：BNO 读到的全是乱码头（或乱码与 NACK 交替）时，以前提前返回跳过了 2 秒检查、乱码又把连续失败计数清零并零等待重读，会一直“streaming”空转；现在乱码按轮询间隔重读、不清连续失败计数，2 秒无新样本必重开。LSM 没有乱码分支，改用同一处检查。
- 前倾判定只认连续一致的读数：连续 5 帧朝前才通过，连续 5 帧朝别处才拒绝，中途变向从头数（以前 4 帧侧倾加 1 帧朝前就会存下转了 90° 的 mount）。
- 前倾方向从直立读数算起（重力变化量），直立门限收紧到 5°，残余倾角回显。
- 控制台 IMU 安装面板按当前源读写：模块源用标定报告新增的 `imu_mount`（只在模块源出现），并显示源名；总线源 A 的读法不变。

## 尚未在真机验证

- 没有上过机。BNO085 和 LSM6DSV16X 模块都只对假总线跑过；i2c-dev 的两段读取、`I2C_RDWR` 寄存器读、BNO 的时钟拉伸都要实测。
- C 的 FIFO 是否支持跨字连读未验证；读取线程按固件做法每字一次 7 字节读取，解码器接受任意切分。
- BNO 的 Game RV 与 SFLP 融合不同，落脚冲击时重力瞬态可能不同——上机前做影子对比（robotd 跑 A 时用 servo-web 的 `imu_i2c.py` 同时记 B）。
- I²C 受舵机电流干扰时会重开，约 0.5 s 起的退避空窗加 1 秒稳定期，控制环先 coast 再 hold。
- 新板要启用对应 I²C overlay；servo-web 占着节点时 flock 失败，报错会点名。
- 升级时 `configure.py` 的合并保留 `[trunk_imu]`（有测试）；其他重建路径（`rebuild-config.py` 从 `.saved` 副本恢复）照搬整个文件，也应保留，但没有在板上走过一遍。

## 第一次上真机（2026-10-12，BNO085，第一台）

装法：官方 `daemon-v0.15.1` 用本地发布文件安装 → 本分支打出的 R17 包（原版包换 robotd、robotctl、`calibration-config.py`、`rebuild-config.py`、`runtime.py`、`voice-setup.py`，重算 `SHA256SUMS`）→ `robotd.toml` 加 `[trunk_imu] source = "bno08x"`、`/dev/i2c-4`、`0x4B`。

已在真机确认：

- BNO085 识别成功：`part=10003676 version=(2, 6, 6) build=193`。两段读取和时钟拉伸在 RK3566 的 i2c-dev 上都正常，没有出现乱码或重开。
- 调试模式和运动模式都不读 ID 200，15 颗舵机 50 Hz 全应答；`robotctl health` 报 healthy。
- 安装时没有 HAT 声卡，`voice-setup.py` 的 `alsactl store aic3104` 会让整个升级失败，已修（`790b45d`）：没有这张声卡就跳过音频设置。
- 运动模式下 HOME（`robot.init`）写入 P5/D20 并回读确认，2 秒到位；启用策略原地保持 10 秒，机身倾斜从 6.6° 收到 3.3°。

真机上发现、还没解决的：

- **IMU 过期帧约 1.7%**（调试模式 2878 帧里 48 帧超过 50 ms；运动模式 `robotctl health` 报 1/52）。都是单帧，控制循环会 coast。当时 CPU 过热锁在 408 MHz，要等散热解决后复测，才能分清是降频还是读取线程本身的问题。
- **`verify-imu` 没法在站着时做**：它只在调试模式下可用，而调试模式启动时会卸力。手扶软鸭子做不到 ≤5°（读到 15.2°）。这次是用 servo-web 让鸭子站到 HOME，读 BNO085 加速度，按 verify-imu 同样的标准判定（站立 4.95°；前倾变化 16.2°，方向偏正前方 3.9°），再手动写入 `[imu_mounts.bno08x]`（默认 quat，`verified = true`）。后续要让 verify-imu 能在运动模式的 HOME 保持状态下运行，或者让调试模式可以选择保持上力。
- 原版 R17 的问题，跟本分支无关，但上机时会碰到：首次安装会把样机标定写成本机标定；舵机被别的程序上了力时，HOME 只核对 P/D、不重写，于是拒绝上力；`service.sh init` 在 robotd 运行时会抢锁失败；CPU 降频时网页手柄（webpad）超过 0.6 秒等不到 padd 确认，按键作废。

## 合进 V164、V167，以及新卡（2026-10-13 ～ 10-14）

- **10-13，合进 V164 / control.40**（分支 `hd1910-v164-trunk-imu-i2c`）。第一台的新卡用的就是它：kissqy 一键镜像 R1 换成瑞莎引导、开 `i2c4-m0`，换上这版 robotd、robotctl 和 4 个安装脚本，`[trunk_imu] source = "bno08x"`、`/dev/i2c-4`、0x4B。BNO085 正常出数，站 HOME，模型接管走了 13 秒后整机掉电（供电问题，不是这个补丁；见 microduck-replica 的 10-13 夜上机记录）。
- **10-14，合进 V167 / control.43**（本分支 `hd1910-v167-trunk-imu-i2c`）。唯一的冲突在 `duck-control/src/feetech.rs` 的测试模拟器 `Emulator`：我们加的 `requests`（记录每个非写请求）和 V167 加的 `reads`（记录每次读的地址和长度）两个字段都保留，两处记录也都保留。生产代码没有冲突。V167 上力时把 RAM 里的 D 写成 0、转矩限制写成 1000（`hd1910.rs` 的 `write_fixed_settings`），总线小板和模块两种来源都一样。
- **中控改动没带进这条分支**：上面「评审后的修正」里提到的两处中控改动（调试页按来源显示 IMU 名字、安装面板按当前来源读写）只留在 V164 那条分支里；这条分支的 `console/` 跟 kissqy 原版一字不差。所以中控里模块的 IMU 一行仍写 FT6，「保存安装确认」等 15 秒会超时，安装方向走 SSH 确认（见仓库根目录 `IMU-MODULE.md` 第 3 节）。
- **测试**（WSL，跟未改的 V167 在同一环境下对比；电脑上用的 ONNX Runtime 是 1.24.4，不是要求的 1.28.0，所以两边都有同样 17 个用例失败）：`verification/run-V167-tests.py` 跑 duck-control、duck-ipc-proto、robotd（每个用例一个进程）、padd，本分支 579 过 / 17 败，原版 498 过 / 17 败，失败名单完全相同；robotd-params、robotctl 全过；packaging 的 Python 测试两边每个文件结果相同；rustfmt、clippy 没有新增问题（原版本身有不合格项）。`cargo board -p robotd -p robotctl` 编译通过，glibc 最高 2.30，BUILD 是 `1fa8438-feetech-ft6-control.43`。
- **安装脚本** `deploy/imu-module-install.sh`：在模拟的安装目录里测过（跳过 systemd）：各种拒绝、安装、再装、撤回、中途失败自动撤回、没有 `tomllib` 时读标定，共 21 项全过。
- **还没上真机**：合进 V167 后的 robotd 和安装脚本都还没在板子上跑过。
