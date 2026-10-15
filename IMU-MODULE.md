# IMU 模块直连主板 I²C · 舵机调参 · UDP 手柄

> **这是 [microduck-replica](https://github.com/fanhao375/microduck-replica) 维护的 fork，不是 kissqy 的官方版本，没经过他审阅，也不代表他认可。**
> 原版在 [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy)。用这一版出了问题，请到 [microduck-replica 的 Issues](https://github.com/fanhao375/microduck-replica/issues) 反馈，别去找 kissqy：这一版的 robotd 报的版本号跟他的一模一样，他分不出来。
> kissqy 的原版首页改名为 [README-kissqy.md](README-kissqy.md) 保留；那里的下载区、一键镜像、中控 ZIP 都是他的原版，**不带**这里的任何改动。

## 这是什么

在 kissqy 的 HD-1910 整套（R17 V167 / control.43）上加三样东西，都是可选的，不打开就跟他的 V167 一样：

1. **IMU 模块直连主板 I²C**：kissqy 的 robotd 只认舵机总线上的 IMU 小板（ID 200，刷他的 FT6 固件）。这里加了两种现成模块，`robotd.toml` 里一行选：

   | `[trunk_imu] source` | 硬件 | I²C 地址 |
   |---|---|---|
   | `bus`（不写就是它） | kissqy 的总线 IMU 小板，ID 200 | — |
   | `bno08x` | BNO085 / BNO086 模块 | 0x4A 或 0x4B（不写默认 0x4B） |
   | `lsm6dsv16x` | LSM6DSV16X 转接板（跟总线小板同一颗芯片） | 0x6A 或 0x6B（必须写） |

   BNO085 另有一个可选的「陀螺积分姿态」：芯片自己的姿态在走路时会突然跳几十度，robotd 用芯片的陀螺把它接住。默认不用（`chip`），只在诊断里多一路，见 4.4 节。

2. **舵机调参（可选）**：一个参数文件给全部 15 个舵机设 P、D、转矩限制，下次 HOME 上力时写入；另有命令改联合 Walk 模型的头、腿滤波。没有参数文件时，上力和卸力发到舵机的报文跟 V167 原版一个字节都不差（P = 模型合同的 5、D = 0、转矩限制 1000），诊断里只多一个值为 null 的 `servo_parameters`。见第 4 节。
3. **UDP 手柄与掉线坐下**：手柄（蓝牙的，或电脑上用官方 `duckctl udp-pad` 发过来的）正在开着鸭子时断了，1 秒后鸭子问一声、坐下；回来时打招呼。UDP 手柄默认关，一条命令打开。见第 5 节。

V167 本身的「修复鸭身抖动」（每次上力把舵机 RAM 里的 D 写成 0、转矩限制写成 1000）原样保留。

## 现在的状态（2026-10-15）

| | 状态 |
|---|---|
| BNO085 + V167 | 10-14 已在我们鸭子的新卡上实装（只换了 IMU 部分：robotd、robotctl 和 4 个安装脚本），BNO085 出数、安装方向确认过，站立、行走都跑过 |
| 舵机调参 | 单元测试过：不设参数文件时，上力 / 卸力发到串口的每一帧跟 kissqy 原版 V167 逐字节相同（同一个测试在原版代码上也通过）；诊断输出只多一个 `native.hd1910.servo_parameters: null`。**还没上真机** |
| UDP 手柄 / 掉线坐下 | padd 单元测试 79 项、电脑上的端到端联调 14 项（假 robotd + 模拟 duckctl）都过；这一版交叉编译的 padd **还没在板上跑过** |
| LSM6DSV16X | 只对模拟总线测过，**没上过真机** |
| BNO085 陀螺积分姿态 | 单元测试和 50 Hz 录像回放通过，**还没上真机**。默认 `chip`：发给策略的姿态跟以前逐字段相同，只多诊断（`busStatus` 和状态流里的 `fusion`）；切 `gyro` 才用积分姿态。原始报告落盘（`bno08x_raw_log`）同样没上过机 |
| 安装脚本 `imu-module-install.sh` | 在假板子上测过 73 项（模拟 systemd 和 robotd）；**这一版还没在真板上跑过**。上一版在真板上暴露的两个问题已改：robotd 卡在「等总线」时卸力确认必失败；kissqy 中控升级后清单里本来就有对不上的行，全量核对必失败 |
| 已知问题 | ① IMU 偶尔有过期帧，约 1.7%（当时主板过热降频，没分清原因）；② `verify-imu` 只能在调试模式下做，而调试模式一启动就卸力（见第 3 节）；③ V167 默认的 P5 / D0 在我们的鸭子上策略站立时，机身前后约 2.5 Hz 晃动（俯仰角速度 RMS 约 12°/s；旧卡 P3 / D0 / 转矩 700 时约 1～2°/s），要用调参在同一张卡上对比 |

实施记录、跟计划的出入、还没验证的项：[IMU 部分](microduck-hd1910/firmware/docs/project/trunk-imu-i2c.zh-CN.md)、[调参与 UDP 手柄部分](microduck-hd1910/firmware/docs/project/replica-v167.zh-CN.md)。机制本身：[`robotd-design.md` §2.6](microduck-hd1910/firmware/docs/design/robotd-design.md)（IMU）、同一文件「R17 HD1910 合同 P（control.33）」一节的 fork 注（调参）、[`docs/robot/udp-pad.md`](microduck-hd1910/firmware/docs/robot/udp-pad.md)（UDP 手柄）。

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
export DUCK_SYSROOT=~/.cache/duck-sysroot    # padd 要链接板上的 libudev：下面这步把 Debian trixie 的
sh scripts/cross-sysroot.sh                  # arm64 包解到这个目录，最后打印几行 export，照着执行
cargo board --locked -p robotd -p robotctl -p padd   # 产物在 target/aarch64-unknown-linux-gnu/release/
```

- 国内下载 Debian 包慢，可以先 `export DUCK_DEBIAN_MIRROR=https://mirrors.ustc.edu.cn/debian`；
- **别跑 `build-V167.sh`**：它会核对源码跟 kissqy 的发布逐字节一致，我们改过，一定失败；
- **别拿错文件**：`packaging/robotd`、`packaging/padd` 是 `prepare-V167.py` 恢复出来的 kissqy 原版。要用 `target/aarch64-unknown-linux-gnu/release/` 下面的 `robotd`、`robotctl`、`padd`。

**第 2 步：拷到板上**，放进一个目录，比如 `~/replica/`：
- `robotd`、`robotctl`、`padd`（第 1 步编出来的）；
- `microduck-hd1910/firmware/packaging/` 下的 `calibration-config.py`、`rebuild-config.py`、`runtime.py`、`voice-setup.py`、`service.sh`；
- `microduck-hd1910/firmware/deploy/imu-module-install.sh`。

**第 3 步：舵机电源开着，扶住鸭子（会卸力），在板上运行：**

```bash
sudo bash ~/replica/imu-module-install.sh install ~/replica bno08x /dev/i2c-4 0x4B
```

脚本按顺序做这些，每一步写进 `/var/log/imu-module-install.log`：
1. **检查**：同一时间只能有一个服务管理操作（跟 `service.sh` 用同一把锁）；板上是不是 control.43；拿来的三个程序是不是完整的 arm64 程序（用板上 kissqy 自己的检查）；robotd 是不是 control.43、带 IMU 模块和调参；padd 是不是认 `[netpad]`、依赖的库跟 kissqy 的 padd 相同、glibc 不超过 2.30，并在板上用 `ldd` 和 `padd --help` 确认能加载；`service.sh`、`runtime.py` 是不是这个分支的；地址对不对；`/dev/i2c-N` 在不在。板上已有旧版本留下的 `servo-test-parameters.json` 时打印出来：超出这个 robotd 收的范围（比如 kp = 0）就先拒绝，什么都不改；
2. **停 padd、卸力、停 robotd**：先停 padd（它的程序要换，也不让手柄在这中间再上力）；robotd 在运动模式控制中时，让它卸力并等到扭矩关闭、舵机寄存器恢复完成（第一次没回答会再问，最多 8 秒）；robotd 在等总线、在调试模式、在反复重启或 8 秒都不回答时，扭矩状态确认不了，只发一次卸力请求，提醒你扶住或放倒鸭子；套接字不存在就是没在运行，不用卸力。然后停 robotd（不管 systemd 说它在不在跑）。跟着 robotd 一起停的单元（比如录像服务）记下状态；
3. **备份**到 `/opt/robot/imu-module-backup-时间/`：要换的 3 个程序和 5 个脚本、3 份清单、`robotd.toml`（两份）、标定文件（两份）、已有的参数文件、robotd 的启动配置、padd 的 AF_INET 配置（或「原来没有」的记号），以及安装前 `SHA256SUMS` 就核对不过的行；
4. **换文件**（`service.sh` 可执行），重算 `SHA256SUMS`、`ELF-CHECK.json`、`BUILD.json`。换过的行必须全部核对通过，而且不能出现新的不一致；安装前就对不上的行（kissqy 中控升级会保留旧的模型目录、却写入新清单，`models/…/deployment-contract.json` 这类行刚升完就对不上）原样保留并在日志里列出；
5. `robotd.toml` 和 `robotd.saved.toml` 写入 `[trunk_imu]`；
6. 重新生成 robotd 的启动配置（允许 `/dev/i2c-*`），加 padd 的 `/etc/systemd/system/padd.service.d/zzzz-replica-netpad.conf`（允许 AF_INET，UDP 手柄要用；kissqy 自己的 padd 配置不动）；
7. **启动并核对**：重载 systemd → 起 robotd → 等它四步启动完成（最多 60 秒）→ 核对 `busStatus`（版本、IMU 来源、有没有调参字段；不对就再问，最多 8 秒）→ 重启 padd，看新进程开始驾驶 → GameSir 输入桥**只在它本来就是启用状态时**一并启动（不启用任何本来停用的服务，不跑 `gamepad.py prepare-all`）。最后报告输入服务的状态、IMU 安装方向确认过没有、有没有参数文件；跟着 robotd 停了、没有跟着起来的单元会提示，但不替你启动。

**备份之后任何一步失败，脚本都会自己撤回**：停 robotd 和 padd、把备份拷回去、逐个比对、重新启动。备份之前就失败（比如已经停了 padd、robotd，备份却写不进去），或者中途被 Ctrl-C、被 kill，也会把停掉的服务按原样起回来。

手动撤回：`sudo bash imu-module-install.sh rollback <它打印的备份目录>`，回到**那次安装之前**的样子：
- 恢复的是程序、脚本、清单、两份 `robotd.toml`、安装之前就有的参数文件、启动配置和 padd 的 AF_INET 配置。`robotd.toml` 被替换之前，现在的版本先另存到 `<备份目录>/etc-at-rollback/`；
- **两份标定文件不恢复**：安装之后重新标定或确认过的 IMU 方向会保留；跟备份时不一样就在日志里提示，备份时的版本在 `<备份目录>/etc/`；
- 安装之后才写的参数文件留在原处（不可调的 robotd 不读它）；`joint-filters` 改过的滤波也不跟着撤回（见 4.2）；
- 要用**这次安装新打的备份**。备份里的 robotd 是 kissqy 原版、而标定文件里有 `[imu_mounts]` 时，脚本拒绝撤回（原版 robotd 读不了它，会一直卡在等总线）；真要回到 kissqy 原版，先删掉两份标定文件里的 `[imu_mounts.*]` 再撤回；
- 第一版脚本打的旧备份（没有安装前的核对记录，也没有 padd、service.sh）默认拒绝，确实要用加 `--old-backup`，备份里没有的文件留着现在的；
- 撤回结束退出码为 2，表示文件已恢复、服务已启动，但出现了安装前没有的核对差异（日志写明是哪一行）。

看现在装的是什么：`sudo bash imu-module-install.sh check`（程序种类、robotd 状态、padd 的协议族、输入服务、IMU 方向、参数文件、两份 `robotd.toml` 的 `[netpad]`、Walk 的头腿滤波、清单里对不上的行）。

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

## 4. 调参（可选）

全部走 SSH，用板上的 `service.sh`（以下写成 `S=/opt/robot/feetech-ft5-r5/service.sh`）。kissqy 原版中控没有这些按钮。

### 4.1 舵机 P / D / 转矩限制

```bash
sudo bash $S servo-parameters 3 0 700     # P D 转矩限制，全部 15 个舵机（含嘴巴）用同一组
sudo bash $S servo-parameters-clear       # 删掉参数文件 = 回到 V167 原版
```

- **范围**：P 1～32、D 0～64、转矩限制 200～1000，都是整数。P 大于 10 或转矩限制小于 500 时打印警告（不拦，也不问）；
- **存在哪**：`/etc/robot/feetech-ft5/servo-test-parameters.json`，内容就是 `{"kp": 3, "kd": 0, "torque_limit": 700}`；
- **什么时候生效**：下次 HOME 上力时，一次写进全部舵机 RAM 的 48 号起 4 个字节（转矩限制低、高、P、D），再读回核对；不一致就拒绝上力。不写 EEPROM，策略的 gain 请求和换模型都不会改它；
- **卸力后是什么状态**：P 恢复成上力前的值，D = 0，转矩限制 1000，跟 V167 原版卸力后完全一样；
- **不设参数文件**：就是 V167 原版，上力写 P = 模型合同的 5、D = 0、转矩限制 1000，发到舵机的报文跟 kissqy 的 control.43 一个字节都不差（诊断里只多一个 `servo_parameters: null`）；
- **每条命令都会卸力并重启 robotd**，卸力按 robotd 当时的状态分开处理，**都要扶住或吊起鸭子**：
  - 运动模式控制中：让它卸力，确认扭矩关闭、寄存器恢复完成后才停它；确认不了就什么都不改，robotd 保持运行；
  - 在等总线、在调试模式：扭矩状态确认不了，发一次卸力请求，然后停；
  - 没在运行（套接字不存在或被拒）：直接停；
  - 8 秒内一直不回答：发一次卸力请求，提示「扭矩状态未确认，扶住或放倒鸭子」，然后停；
  之后写文件、重启 robotd、回读确认它读到的就是你给的值，最后拉起 padd（GameSir 输入桥只在启用时）；重启或回读失败时也会拉起 padd，并说明文件已经改了；
- **确认生效**：命令最后会打印回读结果；`sudo bash $S report` 里 `native.hd1910.servo_parameters` 是你的值，`gain_mode` 变成 `hd1910_servo_parameters_file_relax_p_restored_d0_limit1000`；HOME 时 `journalctl -u robotd` 有 `HD1910 servo parameters file (torque limit, P, D) confirmed on all joints`；
- **建议先跑对照**：`servo-parameters 5 0 1000` 跟不设文件时舵机寄存器的状态逐字节相同（有测试），先用它确认整条路径正常，再试目标值；新值先吊着试；
- **文件坏了**（手改错、范围外）：robotd 停在「等总线」并写明原因，不碰舵机；`sudo bash $S servo-parameters-clear` 恢复；
- **已经上力时 HOME**（比如 robotd 被重启过、舵机还带着扭矩）：只核对文件里的值，不写；不一致就拒绝上力，报错写「torque stays on; relax first」，先卸力再 HOME；
- **卸力后恢复一直确认不了**（某个舵机不应答）：命令会拒绝并保持 robotd 运行；先断舵机电，再 `sudo systemctl stop robotd`，然后重试；
- 一次写 48～51 四个相邻寄存器，是 kissqy 设计文档「P 写入和恢复不跨相邻寄存器」的例外；kissqy 自己的 control.29 用过同样的写法，P3 / D0 / 700 在我们的旧卡上跑过多次。

### 4.2 头、腿滤波（联合 Walk 模型）

```bash
sudo bash $S models                             # 看当前 Walk 的模型标识
sudo bash $S joint-filters <模型标识> 0.50 0.70   # 头部 腿部，0.10～1.00，步长 0.01
```

- 只改当前启用的联合 Walk 模型目录里 `deployment-execution.json` 的 `head_lowpass` / `legs_lowpass`，动作缩放不动；第一次改时把原文件存成 `deployment-execution.json.orig`，以后不再覆盖；
- 模型标识、是不是当前 Walk、有没有联合执行配置，在卸力**之前**就检查，打错不会白卸一次力；
- 恢复：`joint-filters <模型标识> 0.50 0.70`（随包默认），或者把 `.orig` 拷回去。安装脚本撤回时**不会**恢复滤波，会在日志里提醒。

### 4.3 要知道的

- **robotd 停止、重启、崩溃都不卸力，也不恢复寄存器。** 调参一律用上面的 `service.sh` 子命令，别直接 `systemctl restart robotd`；
- **掉电、换电池、舵机重启之后，先卸力再 HOME**：robotd 认为自己还上着力时，HOME 直接返回，舵机会带着上电默认的 P32 / D40 上力（V167 原版也一样）；
- **这些会删参数文件**：`service.sh install / start / upgrade / reset-models`，以及 kissqy 中控的「安装 / 升级固件」（它们走同一个安装流程）。`reset-models` 还会把随包模型的滤波文件覆盖回默认（`.orig` 留着）；
- robotd 上力中被重启过、又删了参数文件时，卸力后 P 可能停在文件里的旧值，给舵机断一次电就回到出厂值。

### 4.4 BNO085：用芯片的姿态，还是陀螺积分的（试验中）

**为什么**：我们的录像里，走路时 BNO085 自己融合出来的姿态隔一阵就在 20 ms 里跳 20–60°，同一时刻芯片的陀螺说机身几乎没动（推摇杆走路的帧里 2 %～23 %，看脚底材料；静站 0.05 %）。策略看到的重力跟着跳，等于突然被告知「你在倒」。这一版的 robotd 把芯片的陀螺积分成一份自己的姿态，每来一个芯片姿态就比一下倾斜：差得少（< 6°）就慢慢跟芯片，差得多就不信芯片、靠陀螺撑着，撑久了再按规则慢慢或整步对回芯片。机制见 [`robotd-design.md` §2.6](microduck-hd1910/firmware/docs/design/robotd-design.md)，依据和测试见 [`trunk-imu-i2c.zh-CN.md`](microduck-hd1910/firmware/docs/project/trunk-imu-i2c.zh-CN.md) 最后一节。

两个键，都在 `[trunk_imu]` 里，只对 `bno08x` 起作用：

| 键 | 取值 | 作用 |
|---|---|---|
| `bno08x_fusion` | `"chip"`（不写就是它）/ `"gyro"` | 发给策略和倒地判断的是芯片的姿态还是积分姿态。两种模式下融合都在跑 |
| `bno08x_raw_log` | `"off"`（不写就是它）/ 绝对路径 | 把芯片发来的每一条报告追加写进这个 CSV 文件（陀螺、姿态的原始值、序号、延迟），用来在真机 100 Hz 数据上重新定门限。约 1 MB/分钟，64 MB 封顶；写盘跟不上就丢行并计数，不会拖慢 IMU |

改的办法：两份 `robotd.toml` 都要改（`robotd.saved.toml` 是升级、重建配置时的来源），改完用 `reconnect` 重启：它先让鸭子卸力，再重启 robotd、拉起 padd，**扶住鸭子**。别直接 `systemctl restart robotd`（不卸力，见 4.3）：

```bash
C=/etc/robot/feetech-ft5; S=/opt/robot/feetech-ft5-r5/service.sh
sudo grep -A6 'trunk_imu' $C/robotd.toml          # 看现在的 [trunk_imu]
sudo nano $C/robotd.toml $C/robotd.saved.toml      # 在 [trunk_imu] 那一节里加或改：bno08x_fusion = "gyro"
sudo bash $S reconnect                             # 先卸力，再重启 robotd、拉起 padd
```

**上机顺序**（每一步出问题就停，退回上一步）：

1. **影子模式（默认，零风险）**：装上这一版什么都不用改，发给策略的跟以前一样。顺手把原始日志打开（两份文件加 `bno08x_raw_log = "/var/lib/robot/feetech-ft5/r5/bno08x-raw.csv"`，`reconnect`）。`sudo bash $S report` 里 `native.imu_reader.fusion` 的计数在涨、`mode` 是 `chip`、`disabled` 是 `false`；`raw_log` 的 `lines` 在涨、`error` 为空。录几段：静站 60 秒、各方向倾斜、推几次摇杆走路（录像里每帧多一个 `imu.fusion`，`fused` 就是积分那一路），连同原始日志拿回来分析。用完把 `bno08x_raw_log` 改回 `"off"`。
2. **gyro 静态**：改成 `bno08x_fusion = "gyro"`、`reconnect`。静站 5 分钟，积分姿态跟芯片的倾斜差中位应 < 0.5°，`realign_long` 不涨；再做一遍倾斜。
3. **gyro 走路 A/B**：同参数、同地面，先跑一段 `chip` 当基线，再 `gyro`、`chip`、`gyro` 轮着来，记下起止时间。主指标是推摇杆走路每分钟摔几次（或走多久才摔）；另看晃动不要比 `chip` 大一成以上。
4. **故意摔、抱起来放平**：静止后 2 秒内，积分姿态应回到跟芯片差 1° 以内，HOME 起始检查正常。

**立刻退回 `chip` 的情况**：静站时倾斜差 > 1°；走路中出现一拍 > 5° 的台阶；`realign_long` 在没倒的时候涨；IMU 过期次数比 `chip` 时多；`disabled` 变成 `true`（融合内部出错已自己关掉，发的是芯片姿态，把日志发给我们）。退回就是把两份文件里的 `bno08x_fusion` 改回 `"chip"`（或删掉这行），`reconnect`。

**要知道的**：
- 安装脚本重写 `[trunk_imu]` 时会保留这两个键；撤回时两份 `robotd.toml` 回到安装前的样子；
- 装回不认识这两个键的 robotd（这条分支之前的 fork 版本）时，它们**不会**让 robotd 起不来：robotd 读到不认识的键会在日志里警告一句「this build has no such keys; they are ignored」，照常启动，行为就是 `chip`。kissqy 原版连 `[trunk_imu]` 都不认，见第 6 节；
- 这次没升 `API_VERSION`：只在已有的消息里加了可选字段（`busStatus` 的 `imu_reader.fusion` / `raw_log`、状态流的 `imu.fusion`），只有 BNO08x 才出现，旧客户端读不到也不出错；没有新方法、没有改旧字段，所以不需要中控、btd、configd 跟着升级；
- `verify-imu` 和标定永远拿芯片自己的姿态去比，不会把安装方向拟合到积分姿态上；
- 门限 6°、5°/s、0.5 s 这些数是从 50 Hz 录像定的初值，等影子模式的原始日志回来再定。

## 5. UDP 手柄与掉线坐下

这一版换了 padd（读手柄、把按键变成指令的那个程序）。机制和完整说明：[`docs/robot/udp-pad.md`](microduck-hd1910/firmware/docs/robot/udp-pad.md)。

### 5.1 掉线坐下（蓝牙手柄和 UDP 手柄都有）

**下面是按代码推断的预期，还没上机验证**（单元测试和电脑上的联调过了）。正在开着鸭子的手柄不见了：padd 先停止发指令，robotd 的 0.5 秒 deadman 让鸭子停步；满 1 秒还没回来，鸭子发出一声疑问，然后坐下（`sit_toggle`，跟方向键下默认的动作一样）。已经坐着、或者策略没在驱动（只 HOME 了）时不动。手柄回来时打招呼。短于 1 秒的闪断不问、不坐（超过 0.5 秒会先停步）。坐下用的是当前上力的增益（有参数文件时就是文件里的值，转矩限制调得很低时坐下可能变成塌下）。

**不开 UDP 手柄时，padd 也已经不是 kissqy 原版**：多了掉线坐下、提示音，以及上游的 Select 修复。

什么时候算「不见了」（同样是预期）：
- **G50 Pro（Switch Pro 模式）直连**：不经过输入桥，手柄断开时设备直接消失，会触发；
- **经 GameSir 输入桥（`gamesir-xboxd`）**：桥在手柄断开时销毁虚拟手柄，正常关手柄会触发；但信号突然丢失时，在蓝牙链路超时（缺省约 20 秒）之前，虚拟手柄会保留最后的摇杆值，padd 会照着继续发（kissqy 原有的行为），到链路判定断开才触发；
- **UDP 手柄**：超过 `timeout_ms`（默认 250 ms）没收到数据。大约 0.25 秒判掉线、0.75 秒停步、1.25 秒坐下。

### 5.2 用电脑上的手柄（UDP）

默认关。在板上开（会先卸力，扶住或吊起鸭子）：

```bash
sudo bash $S netpad on             # 端口 4210，超时 250 ms；也可以写 netpad on 4210 250
sudo bash $S netpad off            # 回到蓝牙手柄和网页手柄
```

电脑上（Windows / macOS / Linux，先装 [uv](https://docs.astral.sh/uv/)），插上手柄：

```bash
uvx duckctl@0.17.1 --help                  # 第一次先跑一下，把依赖下好
uvx duckctl@0.17.1 udp-pad 192.168.x.x     # 一定写鸭子的 IP；推杆前进鸭子却后退，加 --invert-y
```

- `netpad on` 会检查两份 `robotd.toml` 的数值（端口 1～65535，超时大于 0 且小于 `[safety] deadman_ms`，没写就是 500）、板上的 padd 认不认 `[netpad]`、padd 允不允许 AF_INET、robotd 有没有完成四步启动（padd 要等它）；写进两份配置（这次写的文件各留一份 `.netpad-previous`），只重启 padd，看新进程的日志里有没有 `netpad on —`，没有就只把这次写过的文件恢复回去，并说明是 robotd 没就绪还是 padd 没按新设置启动；
- `netpad off` 不带参数，保留原来的端口和超时，**从不回滚**（它是回到蓝牙手柄的路）：padd 没按时开始驾驶只报错并说明 robotd 是否就绪；robotd 还没就绪时写好就返回，padd 会等它起来后按 off 启动；
- **开着时蓝牙手柄和网页手柄都不读**；
- **没有认证**：同一个局域网里谁都能开你的鸭子、让它卸力、关机。只在信得过的网络里、用的时候才开；
- Windows 第一次运行会弹防火墙，要放行；电脑开着 Clash 之类的 TUN 模式时，局域网的包可能被截走，关掉 TUN 或者把鸭子的网段排除；
- 逐个试按键时**跳过这三个**：Select 长按 2 秒（坐下并关机）、方向键上按住 3 秒（切到滚轮模式）、方向键右（重启舵机）；
- 摇杆按下（L3 滚轮测试）不在 UDP 协议里，只能用蓝牙手柄；
- kissqy 中控的「修复手柄连接」靠 padd 有没有打开虚拟 Xbox 手柄来判断，UDP 手柄开着时会误判、重启 padd；重启输入桥也会触发一次掉线坐下；
- **`[netpad]` 写错了**（手改）：这一版的 robotd 启动第 1 步就拒绝，日志点名是哪个键（跟上游一致）；把两份 `robotd.toml` 的 `[netpad]` 改好或删掉，再 `sudo bash $S reconnect`（重启 robotd，再拉起 padd；GameSir 输入桥只在启用时一起启动，跟 kissqy 原来在启用桥的板子上的做法一样）。

## 6. 注意

- **别在 kissqy 的中控里点「安装 / 升级固件」**：它会装回他的 robotd（不认 IMU 模块，鸭子读不到 IMU、上不了力）和他的 padd，删掉参数文件；他的安装脚本还会把总线小板那份安装方向标成已确认、关掉运动开关，升级完多半报「启动超时」。点了也不要紧，重新跑一遍第 2 节的安装就行。kissqy 出新版时，等这里合进去再升；
- **换了模块、拆装过，要重新确认安装方向**；
- 别的程序在读同一个 I²C 节点时，robotd 打不开它，报错会说明（代码里说的 servo-web 是 microduck-replica 的网页调试台，它的 I²C 读 IMU 部分目前还没公开）；
- 看板上跑的是不是这一版：`imu-module-install.sh check`，或者看 `robot.busStatus` 里有没有 `imu_source`、`native.hd1910.servo_parameters` 这两个字段（只有这一版有）。

## 7. 改了什么（修改说明）

相对 kissqy `78bef11`（R17 V167 / control.43）：

| 位置 | 改动 |
|---|---|
| `duck-control/src/` | 陀螺积分姿态：新增 `gyro_fusion.rs`；`bno08x.rs`（每条报告按包内顺序交给融合，发布逻辑不变）、`trunk_imu.rs`、`imu.rs`、`feetech.rs`、`feetech_snapshot.rs`、`lsm6dsv16x.rs`、`lib.rs`。IMU：新增 `bno08x.rs`（BNO08x 的 SHTP / SH-2 编解码）、`lsm6dsv16x.rs`（LSM6DSV16X 的上电配置和 FIFO 解码）、`trunk_imu.rs`（主板读的 IMU 的样本、新鲜度门限、各型号安装方向）；`feetech.rs`、`feetech_snapshot.rs`、`imu.rs`、`io.rs`、`lib.rs` 接入这三种来源；`Cargo.toml`。调参：`hd1910.rs`（有参数文件时的上力 / 卸力分支）、`hd1910_test.rs`（读参数文件、范围；kissqy 原有这个文件但没接入，这里接入）、`feetech.rs`（`gain_mode`、测试，包括跟原版逐字节比对的金样） |
| `robotd/src/` | 陀螺积分姿态：新增 `trunk_imu_raw_log.rs`（原始报告落盘）；`trunk_imu_i2c.rs`、`main.rs`、`feetech_commissioning.rs`（标定用芯片姿态）、`raw_stream.rs` 和几处测试。IMU：新增 `trunk_imu_i2c.rs`（读 I²C 的后台线程，坏包丢弃、断线重开）；`main.rs`、`feetech_commissioning.rs`（`verify-imu` 的模块分支）、`feetech_read_only.rs`、`home_start.rs`、`raw_stream.rs` 和几处测试。调参：`main.rs` 开串口前读参数文件 |
| `robotd-params/src/` | 新增 `[trunk_imu]`（含 `bno08x_fusion`、`bno08x_raw_log`）、`[netpad]` 配置节和校验 |
| `duck-ipc-proto/src/lib.rs`（kissqy 原文件） | 状态流的 `ImuState` 加可选的 `fusion`（只有 BNO08x 才有，不升 `API_VERSION`） |
| `robotctl/src/configure.rs` | 认得 `[trunk_imu]`；`[netpad]` 改了之后提示重启 padd |
| `padd/` | 新增 `src/loss.rs`（掉线坐下）、`src/udp.rs`、`src/udp/{health,receiver,wire}.rs`（UDP 手柄），移植自 Pollen Robotics 官方 microduck 0.17.1 的 `pad-map`、`netpadd`；`src/main.rs` 接入；`systemd/padd.service` 允许 AF_INET |
| `packaging/` | IMU：`calibration-config.py`、`rebuild-config.py`（每种 IMU 一份安装方向，重装时保留）、`runtime.py`（启动配置允许 `char-i2c`，回读核对模块的安装方向）、`voice-setup.py`（没有 HAT 声卡时跳过音频设置）。调参与 UDP：`service.sh`（`servo-parameters`、`servo-parameters-clear`、`joint-filters`、`netpad`）、`runtime.py`（同上）、`padd.service`（AF_INET）、`fixtures/robotd-board-v167.toml`（板上真实的配置格式，测试用）。以及相应测试，新增 `test_tunable_parameters.py`、`test_netpad.py` |
| `deploy/` | `robotd.toml` 加 `[trunk_imu]` 的说明（含两个融合键）；新增 `imu-module-install.sh`（装整套，重写 `[trunk_imu]` 时保留两个融合键）和它的假板子测试 `test-imu-module-install.sh` |
| `test-data/bno08x-fusion-replay/` | 新增：4 段 BNO085 录像（CSV，约 300 KB），融合的回放测试用 |
| `scripts/pad-link-test.sh` | 统计掉线时只数 padd 的 `pad gone —` |
| `docs/` | `design/robotd-design.md`（§2.6，含陀螺积分姿态一段；padd 一段加掉线坐下和 UDP；control.33 一节加 fork 注）；新增 `robot/udp-pad.md`、`project/trunk-imu-i2c.zh-CN.md`、`project/replica-v167.zh-CN.md`；`README.md` 加索引 |
| `Cargo.lock` | 依赖锁定跟着变 |
| `console/`（kissqy 的中控） | **没改**，原样保留。所以用模块时，中控调试页的 IMU 那一行还写着 FT6；「保存安装确认」只等 15 秒，模块要 20 秒，会超时，安装方向用第 3 节的 SSH 办法确认；中控里也没有调参和 UDP 手柄的按钮 |
| 仓库根目录 | `README.md`、`README.en.md` 换成我们写的首页；kissqy 的原首页改名为 `README-kissqy.md`、`README-kissqy.en.md` 原样保留（只改了顶部一段和语言切换链接）；`DUCK-SHOPPING-LIST*.md`、`docs/README*.md`、`docs/AUTHOR-JOURNEY*.md` 里指向首页的链接改指原版首页；新增本页 |

每个改过的源码和文档文件开头，都加了一行修改声明（`Cargo.lock` 没法加注释，见本表）；从 Pollen 移植的文件，声明里写了来源文件。

**许可**：
- 我们对 Rust 工作区、`packaging/`、`deploy/`、`scripts/`、`docs/` 的改动，跟 kissqy 的固件一样按 **Apache-2.0** 提供（版权 fanhao375 / microduck-replica）；
- padd 的 UDP 手柄和掉线坐下移植自 Pollen Robotics 的官方 [microduck](https://github.com/pollen-robotics/microduck) 0.17.1（Apache-2.0）；
- 原首页和那几份文档里的改动，跟随 kissqy 原文的个人非商业许可；新首页 `README.md`、`README.en.md` 是我们写的；
- LSM6DSV16X 的寄存器配置照搬自 microduck-replica 的 `imu_to_dxl` 固件（MIT，Copyright (c) 2026 imu_to_dxl contributors），引用的是 ST 的 LSM6DSV16X 驱动（BSD-3-Clause，Copyright (c) 2019 STMicroelectronics）的寄存器定义；
- kissqy 的 `LICENSE`、`LICENSES/`、第三方声明都原样保留；
- `SOURCE-SHA256SUMS.txt`、`firmware/SHA256SUMS`、`packaging/SHA256SUMS`、`PUBLICATION-MANIFEST.json`、`console/BUILD_MANIFEST.json`、`verification/v167/` 描述的是 kissqy 的原版发布，对我们改过的文件不成立，**我们没有重算**：不拿他的发布身份给我们的代码作保。板上的清单由安装脚本按实际换上的文件重算。

**致谢**：kissqy 的 HD-1910 适配整套；Pollen Robotics 的官方 [microduck](https://github.com/pollen-robotics/microduck)（Apache-2.0）。

---

## English summary

This is a fork maintained by [microduck-replica](https://github.com/fanhao375/microduck-replica); it is **not** an official kissqy release and he has not reviewed it. On top of kissqy's R17 V167 (control.43) it adds four optional things; with none of them switched on it behaves like his V167:

1. **IMU module on the board's I²C**: `[trunk_imu] source = "bno08x"` (BNO085/086, 0x4A/0x4B) or `"lsm6dsv16x"` (0x6A/0x6B, address required) instead of his bus IMU board (ID 200).
2. **Servo tuning**: `service.sh servo-parameters P D limit` writes `servo-test-parameters.json` (P 1–32, D 0–64, torque limit 200–1000); the next HOME writes it to all 15 servos in one 4-byte write at register 48 and reads it back; relax leaves P restored, D=0 and limit 1000, exactly as V167 does. Without the file, the frames robotd sends the servos are V167's, byte for byte (a golden test passes on both); the diagnostics only add `native.hd1910.servo_parameters: null`. `servo-parameters-clear` removes it; `joint-filters <id> head legs` edits the joint Walk's head/legs lowpass (the first original kept as `.orig`). Each command relaxes the robot and restarts robotd — hold the duck. robotd never relaxes on stop/restart/crash; after a power cycle, relax before HOME. `service.sh install/start/upgrade/reset-models` and kissqy's console upgrade delete the file.
3. **UDP pad and pad-loss sit** (padd ported from Pollen microduck 0.17.1): a driving pad (Bluetooth, or `uvx duckctl@0.17.1 udp-pad <robot IP>` over the LAN) that stays gone a second is expected to make the robot ask and sit down (from the code and a host run; not yet seen on a robot). The UDP pad is off by default; `service.sh netpad on [port] [timeout_ms]` / `netpad off` switch it, checking both `robotd.toml` copies against their own deadman. It is unauthenticated: LAN you trust only.

4. **BNO085 gyro-integrated attitude** (optional, experimental): walking, the BNO085's own orientation sometimes jumps 20–60° within one tick while its gyro says the trunk barely turned. robotd integrates that gyro into an attitude of its own and checks every chip orientation against it (tilt under 6° taken slowly, over it refused; long refusals realigned only on a calm, still trunk, otherwise slewed at 5°/s). `[trunk_imu] bno08x_fusion = "chip"` (default) publishes the chip's orientation exactly as before, the fused one only in diagnostics (`busStatus` `native.imu_reader.fusion`, the state stream's `imu.fusion`); `"gyro"` publishes the fused one. `bno08x_raw_log = "<absolute path>"` appends every report to a CSV file, for settling the numbers on 100 Hz data. The installer keeps both keys. An older robotd ignores them with a warning and behaves as `chip`; no `API_VERSION` step, since only optional fields were added. Unit-tested and replayed on 50 Hz recordings only — not yet run on a robot. Order on a robot: shadow (chip) with the raw log, gyro standing, gyro walking A/B against chip, deliberate falls; back to chip at once on a standing tilt difference over 1°, a step over 5° while walking, `realign_long` rising while upright, more stale IMU samples, or `disabled`.

Status: the IMU part (BNO085) runs on our robot on V167 (stood and walked, 2026-10-14). The gyro-integrated attitude has not run on a robot. LSM6DSV16X has only been tested against a fake bus. Tuning and the UDP pad pass their unit tests (and a host end-to-end run for the UDP pad) but have not run on a robot; the cross-built padd has never run on a board. The installer has only been tested on fake boards.

Install on a duck already upgraded to V167 through kissqy's console: `python3 prepare-V167.py`, `sh scripts/cross-sysroot.sh` (export what it prints), `cargo board --locked -p robotd -p robotctl -p padd` (do not run `build-V167.sh`, and do not use `packaging/robotd` / `packaging/padd`, which are kissqy's); copy the three binaries, `calibration-config.py`, `rebuild-config.py`, `runtime.py`, `voice-setup.py`, `service.sh` and `deploy/imu-module-install.sh` to the board; with servo power on, `sudo bash imu-module-install.sh install <dir> bno08x /dev/i2c-4 0x4B`. It relaxes the robot according to its state, backs everything up, waits for robotd's four startup steps, checks busStatus and padd, and restores the backup by itself on any failure; `rollback <backup>` returns to the state before that install — use the backup this install printed: an older one may bring back kissqy's robotd, which cannot read the calibration's `[imu_mounts]` and waits for the bus forever (the script refuses that while the calibration has them). The calibration files themselves are never rolled back. It never marks the IMU mount verified — confirm it with `service.sh bus verify-imu --confirmed` in commissioning mode (upright, then tip nose-down 15–30° within 20 s), not through kissqy's console (it gives up after 15 s).

Do not press "install/upgrade firmware" in kissqy's console afterwards: it reinstalls his robotd, which cannot read the module, and his padd, and deletes the servo parameter file; if you did, run the installer again.

Our changes to the Rust workspace, `packaging/`, `deploy/`, `scripts/` and `docs/` are Apache-2.0; the padd UDP pad follows Pollen Robotics' microduck 0.17.1 (Apache-2.0); the LSM6DSV16X configuration follows the MIT-licensed `imu_to_dxl` firmware and ST's BSD-3-Clause driver. Report problems to microduck-replica, not to kissqy.
