# 躯干 IMU 第二种接法：现成模块接主板 I²C（实施计划）

分支 `hd1910-trunk-imu-i2c`，基于 kissqy/Microduck-kissqy `94328f5`（R17 v1.0.146）。2026-10-11 由 microduck-replica 项目起草；同日把源 C（LSM6DSV16X 模块）纳入本分支范围。

## 为什么

R17 的 robotd 只支持一种躯干 IMU：挂在舵机总线上的 FT6 / imu_to_dxl 小板（ID 200，LSM6DSV16X SFLP，身份 `06 00 00 00 F2`），与 15 颗舵机同一次 Sync Read 读回。很多复刻者手上是现成的 IMU 模块（BNO085、LSM6DSV16X 转接板等），直接接 Radxa Zero 3W 的 I²C。本分支加两种 I²C 接法，配置三选一：

| 源 | 配置 `[trunk_imu] source` | 硬件 | 地址 | 状态 |
|---|---|---|---|---|
| A | `bus`（默认，行为一字不改） | 总线小板 ID 200 | — | R17 原生 |
| B | `bno08x` | BNO085/BNO086 模块，`/dev/i2c-4` | 0x4A / 0x4B（默认 0x4B） | 本分支新增 |
| C | `lsm6dsv16x` | LSM6DSV16X 模块（与小板同一颗芯片），`/dev/i2c-4` | 0x6A / 0x6B（必须显式填写） | 本分支新增 |

## 原则

- A 的数值路径不动，现有测试不改，先补一个 A 的黄金测试锁住 `SflpDecoder` 输出。
- B/C 的 IMU 不进总线：每拍 Sync Read 只读 15 颗舵机（严格模式，任何缺帧整帧失败，和 A 实际表现一致）；IMU 由后台线程读 I²C，放进「最新样本」槽，控制线程每拍取一次。
- B/C 的输出走和 A 同一条后处理（安装旋转、保持上一帧四元数、25 次就绪、三点中值），`ImuData` 语义一致。C 与 A 是同一颗芯片：陀螺换算和半精度四元数（含 `w = √(1−x²−y²−z²)`）复用 A 的同一份代码，同样的字节得到逐位相同的读数。
- 安装旋转按源分开存：标定文件新增 `[imu_mounts.bno08x]`、`[imu_mounts.lsm6dsv16x]`（`quat`/`verified`），A 原有顶层键不变。B 默认 mount `[0.5,0.5,0.5,0.5]`（wxyz），等价于 microduck-replica 已在实机验证的 Python 映射：躯干 gyro = 传感器 `(z, x, y)`。C 默认 mount 为单位四元数 `[1,0,0,0]`（转接板朝向未知）。未 verify 前拒绝上力。
- `i2c_address` 只有一个键、一个默认值（0x4B，属于 BNO08x）；校验按源进行，选 C 而不填 0x6A/0x6B 会被拒绝启动，`deploy/robotd.toml` 写明。

## 改动清单（按实施顺序）

1. `duck-control/src/imu.rs`：先提交 A 黄金测试；把 `SflpDecoder` 拆成 `TrunkFrame`（mount、保持四元数、就绪计数、中值）+ FT6 字节解析。
2. `robotd-params`：新增 `[trunk_imu]`（`source`、`i2c_device`、`i2c_address`），registry 三条普通 entry，`Params::validate()` 加 `ParamsError::TrunkImu`（非 bus 源要求 feetech；地址按源：bno08x 0x4A/0x4B，lsm6dsv16x 0x6A/0x6B；设备路径非空）；`deploy/robotd.toml` 加注释段，值等于默认。
3. `duck-control/src/bno08x.rs`（新，纯逻辑）：SHTP 包头/两阶段读取、软复位、Product ID、Set Feature（0x08 Game RV、0x02 校准陀螺，10 ms）、报告长度表、`Bno08xAssembler`（按报告 seq 去重、模长 0.9–1.1、xyzw→wxyz 且 w≥0、主机接收时刻、主机序号）。
3b. `duck-control/src/lsm6dsv16x.rs`（新，纯逻辑）：
   - 初始化序列做成数据（`BRING_UP`：整字节写 / 读改写 / 读回校验 / 等待 / 轮询），逐条移植 microduck-replica `hardware/imu_to_dxl/firmware/Core/Src/imu.c` 的 `configure_one_step`（BDU、IF_INC、FIFO bypass、XL ±4 g、GY ±500 dps、FIFO 水位 3、XL/GY 120 Hz 入 FIFO、SFLP game rotation 入 FIFO、SFLP 120 Hz、XL/GY ODR 120 Hz、嵌入功能页开 SFLP game rotation、FIFO stream），每个 ST 驱动调用落到具体寄存器/位并在注释里引用 `lsm6dsv16x_reg.c/.h` 行号；先查 WHO_AM_I（0x0F == 0x70）再 SW_POR（30 ms 后轮询，100 ms 超时）；末尾照固件做读回校验；不写 gbias，不写任何掉电保存的东西。
   - FIFO 解码：`FIFO_STATUS1/2` 的 9 位未读字数和 watermark/overrun/full 标志（溢出即丢数据，读取线程按新 epoch 重启）；7 字节字（TAG + 6 数据），tag = 第一字节 bit 7:3；陀螺字 → i16 LE ×3，17.5 mdps/LSB → rad/s；SFLP game rotation 字 → 半精度 x/y/z，w 由 A 的同一函数重建（w≥0）；加速度计及其他 tag 计数后跳过；跨读取切开的字可拼接。
4. `duck-control/src/trunk_imu.rs`（新）：`TrunkImuKind`（bus/bno08x/lsm6dsv16x）、`HostImuSample`、`HostImu` trait（B、C 共用；C 的读取线程按 FIFO 约 120 Hz 发布）、`HostImuGate`（最大年龄 50 ms、epoch 变化重置、序号连续 3 拍不变报错、epoch 起始后 1 s 稳定期、错误文本前缀沿用 `Feetech IMU not ready or stale`）。
5. `Calibration`：`imu_mounts` 表（`bno08x`、`lsm6dsv16x`）、`imu_mount(kind)`、`set_imu_mount(kind)`、`validate_motion_for(kind)`；`validate()` 改为任一源已确认即可（只有 A 的旧文件结果不变）。
6. `duck-control/src/feetech.rs`：`Bus::joint_telemetry(servo_ids)`；`FeetechIo` 内部 `enum TrunkImu { Bus{..}, Host{reader, gate, frame} }`；`open_with_imu` / `from_bus_with_imu`（原 `open`/`from_bus` 签名不变 = Bus）；B/C 跳过 ID 200 身份检查；诊断加 `imu_source`、`imu_reader`。
7. `robotd/src/trunk_imu_i2c.rs`（新，仅 Linux）：`std::thread` 读取器，B、C 各一个状态机，共用骨架：`libc` 打开 + `flock`（与 servo-web 的 `imu_i2c.py` 互斥）+ `ioctl(I2C_SLAVE)`；用 `read` 并检查长度，不用 `read_exact`；`arc-swap` 发布最新样本；出错退避重开（epoch+1）；不写芯片 flash。
   - B：清空→软复位→Product ID→使能报告→流式（自适应轮询）→芯片复位重使能（epoch+1）。
   - C：执行 `BRING_UP`→轮询 `FIFO_STATUS1/2`→按字读 `FIFO_DATA_OUT_TAG`（每字 7 字节，与固件一致）→FIFO 溢出或读回失败则重新 bring-up（epoch+1）。
8. `robotd/src/main.rs`：`open_bus` / `uart_setup` 路径按配置建读取器（重试循环外只建一次）；B/C 时 `CommunicationStats` 不含 200。
9. `robotd/src/feetech_commissioning.rs`、`feetech_read_only.rs`：B/C 分支；`verify-imu` 写当前源的槽位，B/C 额外检查倾角 ≤ 10° 并回显 gravity/gyro。
10. `packaging/runtime.py`：systemd drop-in 加 `DeviceAllow=char-i2c rw`（否则打开 `/dev/i2c-4` 返回 EPERM）；`calibration-config.py` 按源检查；`docs/design/robotd-design.md` 同步（含第 1068 行 "no IMU abstraction" 的决定）。

## 测试

`cargo test -p duck-control -p robotd-params -p robotd`（Linux / WSL）。纯逻辑：BNO 编解码（移植 microduck-replica `test_imu_i2c.py` 用例）、LSM bring-up（寄存器模型逐字段核对、银行切换、WHO_AM_I 先行）与 FIFO 解码（手工字节：跨读取切字、未知 tag、空 FIFO、与 A 同字节逐位相同）、映射黄金测试（移植 `test_policy_trial.py`）、gate、标定按源；仿真器：B/C 冒烟（每次 Sync Read 的 ID 表只有 15 颗、从不读 200、25 次后就绪、能上力）、缺舵机时严格失败。

## 已知风险

- BNO 的 Game RV 融合与 SFLP 不同，落脚冲击时重力瞬态可能不同——上机前做影子对比（robotd 跑 A 时用 `imu_i2c.py` 同时记 B）。C 与 A 同芯片同配置，差别只在安装位置和 I²C 读取时延。
- I²C 被舵机电流干扰时会重开，约 200 ms 空窗，控制环先 coast 再 hold。
- 新板要启用 i2c4 overlay；servo-web 占着 I²C 时 flock 失败，报错要点名。
- C 的 FIFO 是否支持跨字连读未验证，读取线程按固件做法每字一次 7 字节读取；解码器本身接受任意切分。
