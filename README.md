**简体中文** | [English](README.en.md)

# 🦆 Microduck HD-1910 · IMU 模块版

**现成的 IMU 模块，4 根线接到主板，直接跑 HD-1910 鸭子的整套。** 不用做总线 IMU 小板，也不用刷固件。

由 [microduck-replica](https://github.com/fanhao375/microduck-replica)（Microduck 复刻总站：硬件、采购清单、上机记录、踩坑记录）维护，每一版都先在我们自己的鸭子上实测。

## 能做什么

- **IMU 模块直连主板 I²C**：BNO085 / BNO086、LSM6DSV16X，`robotd.toml` 里一行切换。不写这一行，就还是读总线 IMU 小板，跟原版一样；
- **一键安装脚本**：先检查版本和文件，再备份，中途任何一步失败自动撤回；`check` 随时看装的是什么，`rollback` 一键回到原样；
- **跟着上游最新版走**：现在基于 R17 V167（control.43），已经带着上游的「治抖」（上力时舵机 D 写成 0、转矩限制 1000）；
- **中文说明**：接线、overlay、编译、安装、确认 IMU 方向，一步一步写清楚。

## 现在的状态（2026-10-14）

| | 状态 |
|---|---|
| BNO085 | 在我们第一台鸭子上跑过（V164 版本，10-12 起） |
| 当前版本（V167） | 单元测试结果跟上游原版一致，arm64 编译通过；**今晚上真机实测**，结果更新在这里 |
| LSM6DSV16X | 写好了，只对模拟总线测过，还没上过真机 |

## 快速上手

1. **接线**：模块的 3.3 V、GND、SDA、SCL 接主板 40 针的 27 / 28 脚（I2C4），开 `i2c4-m0` overlay；装了官方 HAT 的，也可以接 J5（I2C3）；
2. **先升到 V167**：用上游中控 1.0.167 的「安装 / 升级固件」（见[上游原版首页](README-kissqy.md)）；
3. **编译、安装**：`cargo board -p robotd -p robotctl` 编出 arm64 程序，拷到板上运行：
   ```bash
   sudo bash imu-module-install.sh install ~/imu-module bno08x /dev/i2c-4 0x4B
   ```
4. **确认 IMU 的安装方向**，然后站 HOME、走路。

每一步的细节和注意事项：**[IMU-MODULE.md](IMU-MODULE.md)**。

> 装好以后，**别再点上游中控里的「安装 / 升级固件」**：它会装回不认 IMU 模块的原版 robotd。点了也不要紧，重新跑一遍安装脚本就行。

## 相关

- **[microduck-replica](https://github.com/fanhao375/microduck-replica)**：复刻总站，硬件、BOM、上机记录、[踩坑记录](https://github.com/fanhao375/microduck-replica/blob/master/踩坑记录.md)
- **[IMU-MODULE.md](IMU-MODULE.md)**：完整使用说明、改了哪些文件、许可
- [上游原版首页](README-kissqy.md)：系统镜像、中控 ZIP、视频、采购清单（原样保留）
- 问题反馈：[microduck-replica Issues](https://github.com/fanhao375/microduck-replica/issues)

## 来源与许可

- 基于 [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy) 的 R17 V167。**这不是 kissqy 的官方版本**，没经过他审阅；他的原版首页改名为 [README-kissqy.md](README-kissqy.md)，原样保留。
- 仓库是分项许可：`microduck-hd1910/LICENSE`、`LICENSES/` 和各处第三方声明都原样保留。我们对 robotd 等 Rust 工作区、`packaging/`、`deploy/` 的改动按 Apache-2.0 提供；本页是我们写的。
- 运行时的源头是 Pollen Robotics 的官方 [microduck](https://github.com/pollen-robotics/microduck)（Apache-2.0）。
