**简体中文** | [English](README.en.md)

> **这是 [fanhao375](https://github.com/fanhao375)（[microduck-replica](https://github.com/fanhao375/microduck-replica) 项目）维护的 fork，不是 kissqy 的官方版本，没经过他审阅**。分支 `hd1910-v167-trunk-imu-i2c` 在 kissqy 的 R17 V167 上只加了一样东西：**IMU 模块（BNO085 / LSM6DSV16X）直接接主板 I²C，不用总线 IMU 小板**，说明见 [IMU-MODULE.md](IMU-MODULE.md)。下面的下载区、一键镜像、中控 ZIP 都是 kissqy 原版，不带这项支持。用这个 fork 出了问题，请到 [microduck-replica](https://github.com/fanhao375/microduck-replica/issues) 反馈。原版：[kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy)。


<div align="center">

## 🎬 看看鸭鸭的真实表现（5 段视频）

<table>
<tr>
  <td width="20%"><video src="https://github.com/user-attachments/assets/d09a6bfe-7a28-4266-ba90-de2b331472ad" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/a604c0e0-af01-413f-ab00-99ccd26db8d5" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/4508d91c-e91c-40f4-8c3e-5ca5a3f959c3" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/fd0dfafd-dd02-4852-940c-33706c287e9d" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/693a3272-d814-42f6-86b3-be5cb3b35b0f" controls></video></td>
</tr>
</table>

<sub>以上为实机视频。在线演示已同步机器人中控 R17 V167 与训练中控 R1.5.27 的界面，使用模拟数据，不连接实机，也不会真正训练。</sub>

# 🦆 Microduck · 让桌面上的小鸭动起来

### 刷好镜像，连上 Wi-Fi，打开中控。简单一点，好玩很多。

**基于官方 Microduck 0.15.1 · 飞特 HD1910 适配 · 两套中文中控**

**软件版本：机器人中控 R17 V167 · 固件 control.43 · 训练中控 R1.5.27 · 首次配网 R5**

[🦆 **鸭子采购总清单**](DUCK-SHOPPING-LIST.md) · [📥 前往下载区](#下载与版本) · [📚 使用说明](docs/README.md) · [🛠️ 源码](microduck-hd1910/README.md) · [🌱 作者如何从新手进阶](docs/AUTHOR-JOURNEY.md)

[🎮 机器人中控在线演示](https://kissqy.github.io/Microduck-kissqy/) · [🧠 训练中控在线演示](https://kissqy.github.io/Microduck-kissqy/training/)

</div>

> ✅ **Public Image R1 已正式发布！** 整合镜像已通过重新刷卡、首次配网、SSH、机器人通信和 R17 中控识别测试，**无需自行编译机器人固件。**
>
> 📦 **机器人中控 R17 V167 和训练中控 R1.5.27 已上传，可直接下载。** 配套的已编译 control.43 固件及两个中控源码也已公开，下载统一见下方表格。

---

## ✨ 不用从零配置一堆软件

这是一只会**走路、坐下、捡东西、踢球、前滚翻**的小鸭。装上滑轮，还能探索**对称滑行、蹲下、原地转和倒地起身**等动作。实体手柄就能控制，不必为了每个动作打开电脑点按钮；想训练自己的动作，还有独立训练中控。

**Public Image R1 已正式发布并通过刷卡实测**：把官方系统和本项目机器人服务预装在一起。刷入 Radxa ZERO 3W、连接 Wi-Fi，再打开中文中控，就能使用现成的软件环境，无须从源码编译固件。

## 🚀 三步开始

1. **刷镜像** — 将下载区的系统镜像 `.img.xz` 用 [Armbian Imager](https://imager.armbian.com/) 选择「自定义镜像」，选好 TF 卡后刷写。**无需手动解压**，刷机工具会自动解压写入。
2. **连网络** — Zero 首次启动后连接 `Microduck-kissqy` 热点（密码 `12345678`），按照提示设置家庭 Wi-Fi 和新的系统登录密码；若手机没有自动弹出网页，就访问 `http://10.42.0.1:8088`。
3. **开中控** — 完整解压机器人中控 ZIP，运行 `start.cmd` 连接鸭鸭；核对标定并完成 HOME 后再启用运动。需要自行训练时，解压训练中控并运行 `start-training.cmd`。

> 🛠️ **软件预装不等于机械免标定。** 第一次装配或更换舵机，务必核对全部关节的零位、方向、限位和 IMU，再启用运动；不要直接把样机标定用在另一只鸭上。

<div align="center">
  <img src="docs/assets/wifi-setup-desktop.webp" width="760" alt="Microduck 首次开机热点配网页面，带真实鸭鸭与开源入口" />
  <p><sub>首次热点配网页面展示；实际以 Public Image R1 开机界面为准。</sub></p>
</div>

## 下载与版本

| 文件 | 用途 | 下载 |
| --- | --- | --- |
| **系统镜像 R1** · 官方 0.15.1 / control.40 | 新装 ZERO 3W，预装机器人服务和运动模型 | [⬇️ .img.xz](https://github.com/kissqy/Microduck-kissqy/releases/download/public-image-r1/microduck-zero3w-0.15.1-R1.img.xz) |
| **机器人中控 R17 V167** | 电脑连接与操作鸭鸭，内含同版固件升级包 | [⬇️ ZIP](https://github.com/kissqy/Microduck-kissqy/raw/refs/heads/main/release-assets/Microduck-Console-R17-V167.zip) |
| **训练中控 R1.5.27** | 自行训练、仿真评估和导出模型时使用 | [⬇️ ZIP](https://github.com/kissqy/Microduck-kissqy/raw/refs/heads/main/release-assets/Microduck-Training-Studio-R1.5.27.zip) |
| **飞特 HD1910 已编译固件 control.43** | 已有系统单独升级 ZERO 端服务；ARM64 安装包 | [⬇️ .tar.gz](https://github.com/kissqy/Microduck-kissqy/raw/refs/heads/main/microduck-hd1910/firmware/console/updates/Microduck-Robotd-FT6-R17.tar.gz) |

**普通使用下载「系统镜像＋机器人中控」即可。** 训练中控按需下载；V167 中控内含 control.43，独立固件与它一致。镜像 R1 仍预装 control.40，刷卡后在 V167 中控安装随包固件即可更新。

[机器人中控说明](microduck-hd1910/firmware/console/README.md) · [训练中控说明](microduck-hd1910/training-console/README.md) · [独立固件安装与校验](release-assets/README.md) · [镜像发布说明](docs/releases/public-image-r1.md)

源码入口：[固件](microduck-hd1910/firmware/) · [机器人中控](microduck-hd1910/firmware/console/) · [训练中控](microduck-hd1910/training-console/) · [编译说明](microduck-hd1910/docs/BUILD.md)。当前源码与 V167 / control.43 对齐；源码目录已去除重复编译产物。

**V167 更新：修复鸭身抖动的问题。**

## 🎮 两个中控，各做各的事

| 🐣 机器人中控 · R17 V167 | 🧠 训练中控 · R1.5.27 |
| --- | --- |
| 实时 3D 姿态、摄像头、关节/IMU 数据、舵机标定 | 独立训练任务、配方选择、训练队列、进度与奖励曲线 |
| 实体手柄直接操控、固件维护、模型状态 | 可视化仿真、模型评估、ONNX 导出 |
| 不用自己编译 Zero 服务 | 需要具备训练环境的电脑；不要求 Zero 才能运行界面 |

**V167 版本准备了 10 份运动模型**（6 个常规 + 4 个滑轮）。不用手动编写舵机动作，模型由控制服务执行。滑轮动作、倒地起身的效果受装配、标定和模型训练影响，仍需实机谨慎验证。


## 🔧 HAT 与 IMU 自制记录

🎬 **[IMU 手工制作全过程 · Bilibili](https://www.bilibili.com/video/BV1qapz6PERs/)**

自制 HAT 与独立躯干 IMU 均已在实机上经过长时间验证，运行正常，没有发现问题。

[HAT 制作时间线](hardware/HAT-TIMELINE.md) · [IMU 制作时间线](hardware/IMU-TIMELINE.md) · [硬件工程与说明](hardware/) · [🔧 PCB采购表（HAT／IMU）](hardware/README.md#采购清单)

## ❤️ 分享给大家，也尊重原创

这是一个基于 [Pollen Robotics 官方 Microduck](https://github.com/pollen-robotics/microduck) `0.15.1` 的**个人学习与开发项目**，并非官方发布。**除明确标明的官方及第三方代码、模型、素材外，其余自主新增的固件适配、中控、工具、文档及作者自训成果，均为本项目作者原创开发（使用 AI 工具辅助编程）。**

**作者有权授权的原创部分，仅供个人学习、研究及非商业使用。商业使用请先获得本人书面授权。** 包括将原创部分用于商业产品、收费服务或随售卖整机分发。官方 Apache-2.0、第三方 MIT、官方几何模型等各自遵守原许可证，绝不以个人条款覆盖第三方权利。

完整范围：[分项许可](microduck-hd1910/LICENSE) · [个人非商业许可](microduck-hd1910/LICENSES/Personal-NonCommercial.txt) · [第三方说明](microduck-hd1910/docs/THIRD-PARTY.md)

这个项目从一块板、一只鸭、一次次调试做起来；我不是专业团队，代码也有 AI 协作的部分。希望你能少走弯路，更快见到自己的小鸭动起来。欢迎 Star ⭐、交流和反馈。

<p align="center"><img width="180" alt="支持作者的二维码" src="https://github.com/user-attachments/assets/49054608-65ba-41d6-b3f1-a97941976c4b" /></p>
