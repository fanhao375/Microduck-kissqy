# IMU 模块直接接主板 I²C（不用总线 IMU 小板）

> **这是 [microduck-replica](https://github.com/fanhao375/microduck-replica) 维护的 fork，不是 kissqy 的官方版本，没经过他审阅，也不代表他认可。**
> 原版在 [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy)。用这一版出了问题，请到 [microduck-replica 的 Issues](https://github.com/fanhao375/microduck-replica/issues) 反馈，别去找 kissqy：这一版的 robotd 报的版本号跟他的一模一样，他分不出来。
> 本仓库首页 README 里的下载区、一键镜像、中控 ZIP 都是 kissqy 的原版，**不带**这里的 IMU 模块支持。

## 这是什么

kissqy 的 HD-1910 整套（R17 V167 / control.43）里，robotd 只认舵机总线上的 IMU 小板（ID 200，刷他的 FT6 固件）。手上是现成 IMU 模块的人用不了他的整套。

这条分支（`hd1910-v167-trunk-imu-i2c`）给 robotd 加了两种接法：**现成的 IMU 模块直接接主板的 I²C**。不用做总线小板，不用刷固件。`robotd.toml` 里一行选：

| `[trunk_imu] source` | 硬件 | I²C 地址 |
|---|---|---|
| `bus`（不写就是它） | kissqy 的总线 IMU 小板，ID 200 | — |
| `bno08x` | BNO085 / BNO086 模块 | 0x4A 或 0x4B（不写默认 0x4B） |
| `lsm6dsv16x` | LSM6DSV16X 转接板（跟总线小板同一颗芯片） | 0x6A 或 0x6B（必须写） |

不写 `[trunk_imu]` 时，行为跟 kissqy 原版一样。其余全部是 kissqy 的 V167：包括这一版的「修复鸭身抖动」（每次上力把舵机 RAM 里的 D 写成 0、转矩限制写成 1000）。

## 现在的状态（2026-10-14）

| | 状态 |
|---|---|
| BNO085 | **实测过**，不过用的是 V164（control.40）加这套补丁：我们的第一台鸭子，旧卡（R17）10-12 起、新卡（V164）10-13 起一直在跑；接 40 针 27 / 28 脚，`/dev/i2c-4`，0x4B |
| 合到 V167 以后 | 单元测试跟 kissqy 原版 V167 的结果一致（失败名单相同），arm64 编译通过；**还没上过真机** |
| LSM6DSV16X | 只对模拟总线测过，**没上过真机** |
| 安装脚本 `imu-module-install.sh` | 在模拟的安装目录里测过（21 项）；**还没在真板上跑过** |
| 已知问题 | ① IMU 偶尔有过期帧，约 1.7%（当时主板过热降频，没分清原因）；② `verify-imu` 只能在调试模式下做，而调试模式一启动就卸力（见下面「确认安装方向」） |

实施记录、每一处跟计划的出入、还没验证的项：[`microduck-hd1910/firmware/docs/project/trunk-imu-i2c.zh-CN.md`](microduck-hd1910/firmware/docs/project/trunk-imu-i2c.zh-CN.md)。机制本身：[`robotd-design.md` §2.6](microduck-hd1910/firmware/docs/design/robotd-design.md)。

## 1. 硬件

- **模块**：BNO085 / BNO086，或 LSM6DSV16X 转接板，都用 **3.3 V** 供电；
- **装在躯干上**，不是头上。用螺丝固定，线过关节的地方留余量。模块松了、碰歪了，IMU 读数会漂，要重新校；
- **接线**，两种：

| 接到哪 | 主板上的 I²C | 要做什么 | 我们测过没有 |
|---|---|---|---|
| 40 针 27 / 28 脚（SDA / SCL 按瑞莎的引脚图接），3.3 V、GND 就近取 | I2C4，`/dev/i2c-4` | 开 `i2c4-m0` overlay（见下） | **测过**（BNO085，0x4B） |
| 官方 HAT 的 J5（40 针 3 / 5 脚） | I2C3，`/dev/i2c-3` | kissqy 的镜像默认已经开了 `i2c3-pihat`，不用改 | 没测过 |

- **J5 那条线上已经有板载芯片**，地址别撞：C1 版 HAT 有音频 aic3104（0x18）、BMI088（0x19 / 0x68）；D1 版 HAT 有板载 LSM6DSV16X（0x6A）。BNO085（0x4A / 0x4B）两版都不撞；**D1 上的 LSM6DSV16X 模块必须用 0x6B**（模块的 SA0 接高）。接之前 `i2cdetect -y 3` 看一眼已占的地址；
- **开 `i2c4-m0`**（用 27 / 28 脚时）：
  - kissqy 原版镜像（Armbian 引导）：在 `/boot/armbianEnv.txt` 的 `overlays=` 那一行加上 `i2c4-m0`，重启（这条我们没在原版引导上试过）；
  - V1.12 主板换了瑞莎引导的（microduck-replica 的 [`tools/radxa/kissqy-r1-v112.sh`](https://github.com/fanhao375/microduck-replica/tree/master/tools/radxa)）：做卡时加 `--overlay i2c4-m0`；已经做好的卡，要改 `/boot/extlinux/extlinux.conf` 的 `fdtoverlays`，`armbianEnv.txt` 不生效；
  - 重启后能看到 `/dev/i2c-4` 就对了。

## 2. 安装

**前提**：鸭子已经按 kissqy 的说明，用他的中控 1.0.167「安装 / 升级固件」升到了 **R17 V167（control.43）**。这时 robotd 还是他的原版，还在找总线 IMU 小板，没有小板就会一直等，正常。

**第 1 步：编译**（Linux 或 WSL，要 Rust 1.99、`aarch64-unknown-linux-gnu` 目标、cargo-zigbuild 和 zig，详见 [`CONTRIBUTING.md`](microduck-hd1910/firmware/CONTRIBUTING.md)）：

```bash
cd microduck-hd1910/firmware
python3 prepare-V167.py                      # 恢复 kissqy 从 git 里去掉的随包资源（跑测试要用）
cargo board --locked -p robotd -p robotctl   # 产物在 target/aarch64-unknown-linux-gnu/release/
```

- **别跑 `build-V167.sh`**：它会核对源码跟 kissqy 的发布逐字节一致，我们改过，一定失败；
- **别拿错文件**：`packaging/robotd` 是 `prepare-V167.py` 恢复出来的 kissqy 原版，不认 IMU 模块。要用 `target/aarch64-unknown-linux-gnu/release/` 下面的 `robotd` 和 `robotctl`。

**第 2 步：拷到板上**，放进一个目录，比如 `~/imu-module/`：
- `robotd`、`robotctl`（第 1 步编出来的）；
- `microduck-hd1910/firmware/packaging/` 下的 `calibration-config.py`、`rebuild-config.py`、`runtime.py`、`voice-setup.py`；
- `microduck-hd1910/firmware/deploy/imu-module-install.sh`。

**第 3 步：扶住鸭子（会卸力），在板上运行：**

```bash
sudo bash ~/imu-module/imu-module-install.sh install ~/imu-module bno08x /dev/i2c-4 0x4B
```

脚本按顺序做这些，每一步写进 `/var/log/imu-module-install.log`：
1. 检查：板上是不是 control.43；拿来的 robotd 是不是 arm64、是不是 control.43、是不是带 IMU 模块支持（防止拿成 kissqy 原版）；地址对不对；`/dev/i2c-N` 在不在；
2. 让 robotd 卸力并确认卸力了，才停 robotd；
3. 备份：要换的 6 个文件、3 份清单、`robotd.toml`（两份）、标定文件（两份）、启动配置，放到 `/opt/robot/imu-module-backup-时间/`；
4. 换文件，重算 `SHA256SUMS`、`ELF-CHECK.json`、`BUILD.json` 并核对；
5. `robotd.toml` 和 `robotd.saved.toml` 写入 `[trunk_imu]`；
6. 重新生成 robotd 的启动配置，让它能打开 `/dev/i2c-*`；
7. 启动 robotd；最后提醒你这个模块的安装方向确认过没有。

**备份之后任何一步失败，脚本都会自己撤回，并重新启动 robotd。** 想手动撤回：`sudo bash imu-module-install.sh rollback <它打印的备份目录>`。看现在装的是什么：`sudo bash imu-module-install.sh check`。

## 3. 确认 IMU 的安装方向（必须做）

每种 IMU 在标定文件 `hd1910-calibration.toml` 里各有一份安装方向 `[imu_mounts.<source>]`。**没确认过的，robotd 在运动模式下拒绝上力**。安装脚本不会替你标「已确认」。两种办法：

**A. 用 robotd 自带的 `verify-imu`（走 SSH）**

kissqy 原版中控里点「保存安装确认」只等 15 秒，而模块这一步最多要 20 秒，所以在原版中控里点会超时、什么都不保存。走 SSH：

```bash
sudo bash /opt/robot/feetech-ft5-r5/service.sh disable                    # 切到调试模式，会卸力
sudo bash /opt/robot/feetech-ft5-r5/service.sh bus verify-imu --confirmed
sudo bash /opt/robot/feetech-ft5-r5/service.sh enable                     # 回到运动模式
```

- 第二条命令运行时：先让鸭子**直立**（离竖直不超过 5°），再在 20 秒内把整只鸭子**往前低头 15°～30° 并保持**，等命令返回；
- 卸了力的鸭子手扶着很难做到 5° 以内（我们手扶读到过 15.2°），用垫块或支架撑住。

**B. 我们用的办法：站 HOME 校零点**（适合模块装得不太正）
1. 先确认模块的装法跟 `bno08x` 的默认安装方向对得上（默认四元数 `[0.5, 0.5, 0.5, 0.5]`，对应关系见 `duck-control/src/trunk_imu.rs` 里 `the_bno08x_default_mount_is_the_mapping_the_robot_ran` 这个测试）；
2. 在标定文件里手动写上这一节，`verified = true`。四元数要归一化（平方和跟 1 的差不超过 1e-6），小数写够位数；
3. 用 microduck-replica 的 [`tools/radxa/onboard-analysis/`](https://github.com/fanhao375/microduck-replica/tree/master/tools/radxa/onboard-analysis) 先查四个方向都对，再站 HOME，用仿真算出机身应有的倾斜，跟 IMU 读数比，得出偏差，补进四元数。那几个脚本按我们的板子写的：用户名是 `duck`（kissqy 镜像是 `radxa`），仿真那一步要电脑上装 mujoco 和官方 microduck_rl。

## 4. 注意

- **别在 kissqy 的中控里点「安装 / 升级固件」**：它会装回他的 robotd（不认 IMU 模块），鸭子读不到 IMU、上不了力；他的安装脚本还会把总线小板那份安装方向标成已确认、关掉运动开关。点了也不要紧，重新跑一遍第 2 节的安装就行。kissqy 出新版时，等这里合进去再升；
- **换了模块、拆装过，要重新确认安装方向**；
- 别的程序在读同一个 I²C 节点时，robotd 打不开它，报错会说明（代码里说的 servo-web 是 microduck-replica 的网页调试台，它的 I²C 读 IMU 部分目前还没公开）；
- 看板上跑的是不是这一版：`imu-module-install.sh check`，或者看 `robot.busStatus` 里有没有 `imu_source` 这个字段（只有这一版有）。

## 5. 改了什么（修改说明）

相对 kissqy `78bef11`（R17 V167 / control.43）：

| 位置 | 改动 |
|---|---|
| `duck-control/src/` | 新增 `bno08x.rs`（BNO08x 的 SHTP / SH-2 编解码）、`lsm6dsv16x.rs`（LSM6DSV16X 的上电配置和 FIFO 解码）、`trunk_imu.rs`（主板读的 IMU 的样本、新鲜度门限、各型号安装方向）；`feetech.rs`、`feetech_snapshot.rs`、`imu.rs`、`io.rs`、`lib.rs` 接入这三种来源；`Cargo.toml` |
| `robotd/src/` | 新增 `trunk_imu_i2c.rs`（读 I²C 的后台线程，坏包丢弃、断线重开）；`main.rs`、`feetech_commissioning.rs`（`verify-imu` 的模块分支）、`feetech_read_only.rs`、`home_start.rs`、`raw_stream.rs` 和几处测试 |
| `robotd-params/src/` | 新增 `[trunk_imu]` 配置节和校验 |
| `robotctl/src/configure.rs` | 认得新配置节 |
| `packaging/` | `calibration-config.py`、`rebuild-config.py`（每种 IMU 一份安装方向，重装时保留）、`runtime.py`（启动配置允许 `char-i2c`，回读核对模块的安装方向）、`voice-setup.py`（没有 HAT 声卡时跳过音频设置），以及相应测试 |
| `deploy/` | `robotd.toml` 加 `[trunk_imu]` 的说明；新增 `imu-module-install.sh` |
| `docs/` | `design/robotd-design.md` §2.6；新增 `project/trunk-imu-i2c.zh-CN.md`；`README.md` 加一行索引 |
| `Cargo.lock` | 依赖锁定跟着变 |
| 仓库根目录 | `README.md`、`README.en.md` 顶部加了 fork 说明；新增本页 |

每个改过的源码和文档文件开头，都加了一行修改声明（`Cargo.lock` 没法加注释，见本表）。

**许可**：
- 我们对 Rust 工作区、`packaging/`、`deploy/`、`docs/` 的改动，跟 kissqy 的固件一样按 **Apache-2.0** 提供（版权 fanhao375 / microduck-replica）；
- 根目录 README 的那几行改动，跟随 kissqy 原 README 的个人非商业许可；
- LSM6DSV16X 的寄存器配置照搬自 microduck-replica 的 `imu_to_dxl` 固件（MIT，Copyright (c) 2026 imu_to_dxl contributors），引用的是 ST 的 LSM6DSV16X 驱动（BSD-3-Clause，Copyright (c) 2019 STMicroelectronics）的寄存器定义；
- kissqy 的 `LICENSE`、`LICENSES/`、第三方声明都原样保留；
- `SOURCE-SHA256SUMS.txt`、`firmware/SHA256SUMS`、`PUBLICATION-MANIFEST.json`、`console/BUILD_MANIFEST.json`、`verification/v167/` 描述的是 kissqy 的原版发布，对我们改过的文件不成立，**我们没有重算**：不拿他的发布身份给我们的代码作保。

**致谢**：kissqy 的 HD-1910 适配整套；Pollen Robotics 的官方 [microduck](https://github.com/pollen-robotics/microduck)（Apache-2.0）。

---

## English summary

This is a fork maintained by [microduck-replica](https://github.com/fanhao375/microduck-replica); it is **not** an official kissqy release and he has not reviewed it. It adds one thing to kissqy's R17 V167 (control.43): the trunk IMU can be a ready-made module on the Zero 3W's own I²C — `[trunk_imu] source = "bno08x"` (BNO085/086, 0x4A/0x4B) or `"lsm6dsv16x"` (0x6A/0x6B, address required) — instead of his bus IMU board (ID 200). Without `[trunk_imu]` it behaves like kissqy's V167.

Status: BNO085 runs on our robot with the same patch on V164 since 2026-10-12; the V167 merge passes the same unit tests as stock V167 and builds for aarch64, but has not run on a robot yet; LSM6DSV16X has only been tested against a fake bus.

Install on a duck already upgraded to V167 through kissqy's console: build with `python3 prepare-V167.py && cargo board --locked -p robotd -p robotctl` (do not run `build-V167.sh`, and do not use `packaging/robotd`, which is kissqy's), copy the two binaries, the four packaging scripts and `deploy/imu-module-install.sh` to the board, then `sudo bash imu-module-install.sh install <dir> bno08x /dev/i2c-4 0x4B`. It backs everything up, restores the backup on any failure, and never marks the IMU mount verified — confirm it with `service.sh bus verify-imu --confirmed` in commissioning mode (upright, then tip nose-down 15–30° within 20 s), not through kissqy's console (it gives up after 15 s). Do not press "install/upgrade firmware" in kissqy's console afterwards: it reinstalls his robotd, which cannot read the module.

Our changes to the Rust workspace, `packaging/`, `deploy/` and `docs/` are Apache-2.0; the LSM6DSV16X configuration follows the MIT-licensed `imu_to_dxl` firmware and ST's BSD-3-Clause driver. Report problems to microduck-replica, not to kissqy.
