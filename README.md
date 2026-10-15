**简体中文** | [English](README.en.md)

# 🦆 Microduck HD-1910 · IMU 模块版

**现成的 IMU 模块，4 根线接到主板，直接跑 HD-1910 鸭子的整套。** 不用做总线 IMU 小板，也不用刷固件。另带可选的舵机调参和 UDP 手柄 / 掉线坐下。

由 [microduck-replica](https://github.com/fanhao375/microduck-replica)（Microduck 复刻总站：硬件、采购清单、上机记录、踩坑记录）维护；哪些部分在我们自己的鸭子上实测过、哪些还没有，下面的状态表照实写。

## 能做什么

- **IMU 模块直连主板 I²C**：BNO085 / BNO086、LSM6DSV16X，`robotd.toml` 里一行切换。不写这一行，就还是读总线 IMU 小板，跟原版一样；
- **舵机调参（可选）**：一条命令给全部舵机设 P、D、转矩限制，下次 HOME 生效；另一条改联合 Walk 模型的头、腿滤波。不设就是上游原版（发到舵机的报文一个字节都不差）；
- **掉线坐下、UDP 手柄**：开着鸭子的手柄断了，1 秒后鸭子问一声、坐下；也可以用电脑上的手柄通过局域网开（上游官方 `duckctl udp-pad`，默认关）；
- **一键安装脚本**：先检查版本和文件，按 robotd 的状态卸力，再备份，等它四步启动完成并核对，中途任何一步失败自动撤回；`check` 随时看装的是什么，`rollback` 回到安装之前；
- **跟着上游最新版走**：现在基于 R17 V167（control.43），已经带着上游的「治抖」（上力时舵机 D 写成 0、转矩限制 1000）；
- **中文说明**：接线、overlay、编译、安装、确认 IMU 方向，一步一步写清楚。

## 现在的状态（2026-10-15）

| | 状态 |
|---|---|
| BNO085 + V167 | 10-14 在我们鸭子的新卡上装好；10-15 用这一版整套重装，站立、行走、转向都跑过 |
| BNO085 陀螺积分姿态（`gyro`） | **10-15 上过真机**：芯片自己的姿态落脚时乱跳，以前「走两三步就摔」；切 `gyro` 后推摇杆 60 s 一次没摔，在 walk 模式里待了 10 分多钟（推摇杆的 60 s）只摔 1 次。没做完整的轮流 A/B。默认仍是 `chip`，要自己切（[IMU-MODULE.md 4.4](IMU-MODULE.md)） |
| 舵机调参 | 单元测试过（不设参数文件时，发到舵机的报文跟上游原版逐字节相同，诊断只多一个空字段）；这一版 10-15 在板上跑，但只走了「不设文件」那条路，**设了文件的还没上真机** |
| 掉线坐下、UDP 手柄 | 单元测试和电脑上的端到端联调过；这一版的 padd 10-15 在板上跑过（蓝牙手柄开着走了十几分钟），**UDP 手柄和掉线坐下还没在真机上试过** |
| 安装脚本（整套） | 假板子测试过；10-15 在真板上装这一版一次通过；`rollback` 还没在真板上跑过 |
| LSM6DSV16X | 写好了，只对模拟总线测过，还没上过真机 |

## 快速上手

1. **接线**：模块的 3.3 V、GND、SDA、SCL 接主板 40 针的 27 / 28 脚（I2C4），开 `i2c4-m0` overlay；装了官方 HAT 的，也可以接 J5（I2C3）；
2. **先升到 V167**：用上游中控 1.0.167 的「安装 / 升级固件」（见[上游原版首页](README-kissqy.md)）；
3. **编译、安装**：先跑 `sh scripts/cross-sysroot.sh`，**把它最后打印的几行 `export` 照着执行**，再 `cargo board --locked -p robotd -p robotctl -p padd` 编出 arm64 程序；把这三个程序、`packaging/` 下的 5 个脚本（`calibration-config.py`、`rebuild-config.py`、`runtime.py`、`voice-setup.py`、`service.sh`）和 `deploy/imu-module-install.sh` 拷到板上，舵机电源开着，运行：
   ```bash
   sudo bash imu-module-install.sh install ~/replica bno08x /dev/i2c-4 0x4B
   ```
4. **确认 IMU 的安装方向**，然后站 HOME、走路；
5. 需要时再调参（`service.sh servo-parameters`）、开 UDP 手柄（`service.sh netpad on`）。

每一步的细节和注意事项：**[IMU-MODULE.md](IMU-MODULE.md)**。

> 装好以后，**别再点上游中控里的「安装 / 升级固件」**：它会装回不认 IMU 模块的原版 robotd 和原版 padd，还会删掉调参的参数文件。点了也不要紧，重新跑一遍安装脚本、再设一次参数就行。

## 相关

- **[microduck-replica](https://github.com/fanhao375/microduck-replica)**：复刻总站，硬件、BOM、上机记录、[踩坑记录](https://github.com/fanhao375/microduck-replica/blob/master/踩坑记录.md)
- **[IMU-MODULE.md](IMU-MODULE.md)**：完整使用说明（接线、安装、IMU 方向、调参、UDP 手柄）、改了哪些文件、许可
- [上游原版首页](README-kissqy.md)：系统镜像、中控 ZIP、视频、采购清单（原样保留）
- 问题反馈：[microduck-replica Issues](https://github.com/fanhao375/microduck-replica/issues)

## 来源与许可

- 基于 [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy) 的 R17 V167。**这不是 kissqy 的官方版本**，没经过他审阅；他的原版首页改名为 [README-kissqy.md](README-kissqy.md)，原样保留。
- 仓库是分项许可：`microduck-hd1910/LICENSE`、`LICENSES/` 和各处第三方声明都原样保留。我们对 robotd 等 Rust 工作区、`packaging/`、`deploy/` 的改动按 Apache-2.0 提供；本页是我们写的。
- 运行时的源头是 Pollen Robotics 的官方 [microduck](https://github.com/pollen-robotics/microduck)（Apache-2.0）；padd 的 UDP 手柄和掉线坐下移植自它的 0.17.1。
