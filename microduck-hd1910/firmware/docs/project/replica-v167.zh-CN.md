<!-- Added 2026-10 by fanhao375 (microduck-replica): optional servo P/D/torque-limit file; UDP pad ([netpad]) and pad-loss sit; the installer; see IMU-MODULE.md at the repository root. -->
# V167 上的 microduck-replica 整套：调参、UDP 手柄、安装脚本 —— 实施记录

2026-10-14，分支 `hd1910-v167-replica`，基于 `hd1910-v167-trunk-imu-i2c`（`19555da`，即 kissqy/Microduck-kissqy `78bef11`（R17 V167 / control.43）加躯干 IMU 模块）。由 microduck-replica 项目起草方案、经两轮审核后实施。

**机制以设计文档为准**：调参见 [`design/robotd-design.md`](../design/robotd-design.md)「R17 HD1910 合同 P（control.33）」一节的 fork 注；UDP 手柄见 [`robot/udp-pad.md`](../robot/udp-pad.md)；怎么用见仓库根目录 `IMU-MODULE.md` 第 2、4、5 节。本页是这一天的实施记录：做了什么、和方案哪里不同、还有什么没在真机上验证。它会过时，这是有意的。

## 为什么

- **调参**：10-14 晚在新卡上装好 V167 + IMU 模块后，HOME（无参数文件，即 V167 原版：P5 / D0 / 转矩 1000）再策略站立（零指令）72 秒，机身前后约 2.47 Hz 持续晃动，俯仰角速度 RMS 11.8°/s；旧卡上 P3 / D0 / 转矩 700 站 216 秒是 1.1°/s。要在同一张卡上比，就得能改 P / D / 转矩限制。V164 上本地有过一版可调（`hd1910-v164-tunable`），这里按 V167 的语义重新合。
- **UDP 手柄与掉线坐下**：V164 上的分支 `hd1910-v164-udp-pad`（移植自 Pollen microduck 0.17.1），一起合进来。
- **安装脚本**：上一版在真板上暴露了两个必然失败的点（见文末），装整套之前要先改。

## 实施步骤（按提交）

1. **可调版 Rust**（`86c475a`）：robotd 在打开串口前读 `servo-test-parameters.json`（`hd1910_test::load`，范围 P 1–32、D 0–64、转矩 200–1000），坏文件停在 `waiting_for_bus`。`hd1910.rs` 在 inspect 之后进入独立的 `arm_with_file`：全部卸力时先存卸力用的数据，再一次写 48 号 4 字节 [转矩低、高、P、D]，读回 48×4；已上力时只核对。卸力写 [1000、上力前 P、D=0] 并读回（方案的「选 C」：卸力后与无文件时 RAM 相同）。无文件时 V167 的 `arm` / `disarm` / `write_fixed_settings` 原文不动。诊断末尾多 `servo_parameters`；`gain_mode` 无文件时是 V167 原串，有文件时是 `hd1910_servo_parameters_file_relax_p_restored_d0_limit1000`。
2. **可调版 packaging**（`154ce16`）：`service.sh servo-parameters / servo-parameters-clear / joint-filters`，沿用 V164 可调版的流程（先 `relax-and-restore` 确认扭矩关闭且恢复完成才停 robotd），文案改成 V167 语义；P>10 或转矩<500 只警告。kissqy 中控相关的测试不带。
3. **UDP 手柄整组**（`e709eab`）：`git diff 7ba9dbe f101794` 的 15 个文件（不含 `packaging/SHA256SUMS`）一次 apply，结果与 `f101794` 只差修改声明和 `udp-pad.md` 的版本号、drop-in 名。
4. **`service.sh netpad`**（`77b49e3`）：两份 `robotd.toml` 用 tomllib 读、`configure.dumps` 写（板上的表名带引号、技能是内联数组，正则改不对），各按自己的 `[safety] deadman_ms` 校验；`off` 不收参数、保留原数值；写前各留 `.previous`；只重启 padd，按新进程的日志判定，失败从 `.previous` 恢复。servo-parameters 等三条命令结尾也拉起 padd，GameSir 输入桥只在启用时一并启动。
5. **安装脚本**（`40871ae`）：装整套（3 个程序 + 5 个脚本），见 `IMU-MODULE.md` 第 2 节。
6. **第一轮代码审核的修正**（`c85fb81`）：卸力前第一次 busStatus 超时不再当成「robotd 不在运行」；恢复确认不了时给出路；joint-filters 的只读检查提到卸力之前；已上力核对不一致的报错写明「torque stays on; relax first」；重启 robotd 失败时说明参数已写入。
7. **robotctl**（`ff5ea66`）：合进 UDP 手柄后 `robotctl` 的 `every_registry_key_says_how_it_applies` 测试失败（三个 `[netpad]` 键没说改了以后要重启谁）；补上「重启 padd」。V164 那条 UDP 分支没跑过 robotctl 的测试，所以当时没发现。
8. **第二轮审核的修正**（`2b14b11`、`aea117d`，文档同本提交）：
   - `netpad`：备份改用专用的 `.netpad-previous`，只给这次写的文件留、只恢复这次写的文件、恢复或确认后都删掉。原来用 `.previous`，而 kissqy 的 `atomic_write` 每次写都会改它，恢复可能把别人的改动撤掉、两次恢复还会来回翻。`netpad on` 先要求 robotd 已完成四步启动，失败时说明是 robotd 没就绪还是 padd 没起来；`netpad off` 从不回滚（它是回到蓝牙手柄的路）。`reconnect` 改成只在启用时起 GameSir 输入桥；三条调参命令回读失败时也拉起 padd。
   - 安装脚本：加 `set -E` 和 ERR / INT / TERM 陷阱，忽略 HUP——停了 padd、robotd 还没换文件就失败（或被打断）时把原服务起回来，换了文件之后照旧自动撤回；跟 `service.sh` 用同一把锁；顺序改成先停 padd、再卸力、再停 robotd；判断 robotd 状态照搬 `runtime.py`（只有套接字不存在或被拒才算没在跑，慢的再问，8 秒都不回答按「状态未知」处理）；装后 busStatus 再问 8 秒；`ldd` 先写文件再查；报告随 robotd 停止的单元。撤回：拒绝第一版脚本的旧备份（除非 `--old-backup`）和「备份里是原版 robotd 而标定有 `[imu_mounts]`」；**不恢复两份标定**（不一样时提示备份那份在哪）；`robotd.toml` 恢复前另存到 `etc-at-rollback/`。
   - 文档：「逐字节相同」只说串口报文，诊断多一个 `servo_parameters: null`；掉线坐下写成「按代码推断的预期」；英文摘要补齐。

## 与方案的出入、实施时的取舍

- 文件分支抽成独立方法 `arm_with_file`，`arm()` 里只多一个条件和一个分支入口，V167 原文其余不动。
- `servo_parameters` 放在诊断结构的最后一个字段：没有参数文件时诊断只多这一个 null，去掉它后与 V167 相同（金样按 JSON 值比较）。
- 删掉没人调用的 `configure_servo_test`；`raw_stream` 的 BusKey 不加参数（参数只在进程启动时读一次，审核意见）。
- 金样测试：两份副本（本分支、kissqy `78bef11` 的临时副本）的测试模拟器都加一行记录原始请求帧；金样（从启动卸力确认到两轮上力卸力的每一帧，加 hd1910 诊断 JSON）是在 `78bef11` 副本上实际跑出来的，测试函数两边逐字节相同，两边都通过。`78bef11` 副本只在本机临时目录，没提交到任何分支。
- `robotd-params` 多一个测试（共 107）：板上真实的 `robotd.toml` 加上 `service.sh netpad on` 写出的那一节能加载；`test_netpad.py` 断言写出的那一节与这个测试用的文本逐字节相同。另在 `78bef11` 的临时副本里确认：kissqy 原版 robotd-params 加载同一文件成功、忽略 `[netpad]`（撤回或被 kissqy 升级后不会因此起不来）。
- `padd/src/udp.rs` 是按上游 `netpadd` 的做法写成 padd 的第二个输入源，不是逐行移植，声明写作「after Pollen Robotics microduck 0.17.1 netpadd」；其余四个新文件写明来源文件。
- `netpad off` 遇到从没有 `[netpad]` 的文件时不写（不新增一节 `enabled = false`）。
- 安装脚本：
  - `NEEDED`、glibc 版本从 ELF 自身的表里读（板上不一定有 readelf），`ELF-CHECK.json` / `BUILD.json` 里 padd 的条目随之更新；
  - 已有的 `servo-test-parameters.json` 超出这个 robotd 的范围时，启动前就拒绝（kissqy 旧版本允许 kp = 0，装上后 robotd 会一直等总线）；
  - 自动撤回前先停 robotd 和 padd（这次可能已经起来，还是卸力状态），再拷回旧文件；
  - 测试发现原版就有的隐患：`rollback ""` 会把 `/opt` 当备份目录，往安装目录里拷 `/opt/*`。现在要求备份目录里有 `opt/robotd` 和 `etc/`；
  - 测试用的假 systemd 目录下跳过 `runtime.py configure-startup`（它写真实的 `/etc` 路径）。
- 交叉编译的 padd：`cargo board` 前先跑 `scripts/cross-sysroot.sh`（板上 Debian trixie 的 libudev），产物只要 `LIBUDEV_183`、GLIBC 不超过 2.30、`BIND_NOW`、无 RPATH，`NEEDED` 与 kissqy 的 padd 相同。

## 测试（WSL，同一环境下与原版对比）

电脑上的 ONNX Runtime 是 1.24.4（要求 1.28.0），所以 robotd 有同样 17 个用例在原版和本分支上都失败，名单逐条相同。

| 项目 | kissqy 原版 V167 | IMU 版（19555da） | 本分支 |
|---|---|---|---|
| `run-V167-tests.py`（duck-control、duck-ipc-proto、robotd 逐用例、padd） | 498 过 / 17 败 | 579 过 / 17 败 | 626 过 / 17 败（新增：调参 9、padd 38） |
| robotd-params / robotctl | — | 103 / 199 | 107 / 200 |
| packaging Python（逐用例） | 293 个用例：233 通过 / 16 失败 / 44 出错 | 301：240 / 16 / 45 | 338：277 / 16 / 45；相对 IMU 版新增失败 0，新增的 37 个全过 |
| rustfmt（有差异的文件） | 9 个 | 8 个 | 8 个，同 IMU 版；改过的文件都干净 |
| clippy 警告（不加 `-D`；加 `-D` 时三边都在 duck-ipc-proto 停下） | 15 条 | 14 条 | 14 条，同 IMU 版（只有 `hardware.rs` 那条枚举大小从 2512 变成 2536 字节） |

另外：安装脚本的假板子测试 `deploy/test-imu-module-install.sh` 68 项全过（第一版的 21 项、不接 systemd 的新增项，以及用假 systemctl 和假 robotd 套接字模拟的卸力状态、慢回答、不回答、备份失败、ERR 陷阱、中途 TERM、随 robotd 停的单元、撤回前重新标定、旧备份、原版 robotd + imu_mounts、锁被占用）；UDP 手柄的电脑端到端联调（假 robotd + 模拟 `duckctl udp-pad` 的发送端，跑本分支 x86 的 padd）14 项全过。clippy 加上 padd 时，padd 的测试目标在三边都因 `roller_test.rs` 里一处 `approx_constant`（默认拒绝）停下，这是原版就有的；上表是加 `-A clippy::approx_constant` 后的结果，padd 没有警告。

## 上一版安装脚本在真板上的经历（2026-10-14 晚）

- 板子先用 kissqy 中控 1.0.167 升到 V167：原版 robotd 卡在启动第 2 步（`unknown field imu_mounts`，标定文件里我们的那一节），中控报「固件安装 / 等待机器人启动 失败 · 退出码 78」。升级只删了 `robotd.toml` 的第一行注释，`[trunk_imu]` 原样保留；robotd 的启动配置里 `DeviceAllow=char-i2c rw` 被删；padd 和我们的录像服务被停且没起来。
- 上一版 `install` 第一次：`runtime.py relax-confirmed` 要求运动模式下扭矩确认关闭，而 robotd 卡在等总线、根本没有这个字段 → 拒绝，什么都没改。→ 现在按 robotd 的状态分开处理。
- 手动停 robotd 后再装：换文件后全量 `SHA256SUMS` 核对失败，失败的是 `models/1cbf8730…/deployment-contract.json`：kissqy 升级保留板上已有的模型目录，却写入了 V167 包的新清单，两份合同只差标定段。也就是**原版 V167 刚装完、我们什么都没做时，整表就有这一行对不上**。自动撤回把 14 个文件逐字节恢复了，但末尾同样全量核对，于是报「撤回没完成」、也没起 robotd。→ 现在只核对换过的行，其余不一致限定在安装前就有的那些；撤回分开「复制失败」和「核对有差异」。
- 当晚最后用一次性脚本（同样的步骤，只核对换过的行）装上了 IMU 版：robotd 四步启动成功，BNO085 出数，安装方向已确认；之后站立、行走都跑过。

## 尚未在真机验证

- 调参整条路径：参数文件 → HOME 一次写 48 号 4 字节并回读 → 卸力写回。旧卡上 kissqy control.29 用过同样的 4 字节写法，但新卡和 V167 上没跑过。回读不一致时的备选是分开写。
- 交叉编译的 padd 从没在板上运行过（安装脚本会先在板上 `ldd`、`padd --help`）；AF_INET drop-in 是否与 kissqy 的 `RestrictAddressFamilies` 合并生效（systemd 的规则是取并集）；startup_gate 包着的 padd 重启时序。
- 掉线坐下：G50 Pro 直连时设备是否随断开消失；经 GameSir 桥时信号丢失要等蓝牙链路超时；坐下用的是当前上力的增益。
- `service.sh netpad`：按 padd 新进程日志里的 `netpad on —` / `driving —` 判定，依赖日志文字。
- 安装脚本这一版只在假板子上跑过；真板上要确认四步启动等待、busStatus 核对、padd 重启判定、输入桥只在启用时启动。
