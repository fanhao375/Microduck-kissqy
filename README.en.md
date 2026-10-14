[简体中文](README.md) | **English**

> **This is a fork maintained by [fanhao375](https://github.com/fanhao375) ([microduck-replica](https://github.com/fanhao375/microduck-replica)), not an official kissqy release, and not reviewed by him.** Branch `hd1910-v167-trunk-imu-i2c` adds one thing to kissqy's R17 V167: **an IMU module (BNO085 / LSM6DSV16X) on the board's own I²C instead of the bus IMU board** — see [IMU-MODULE.md](IMU-MODULE.md). The downloads, one-click image and console ZIP below are kissqy's originals without it. Report problems to [microduck-replica](https://github.com/fanhao375/microduck-replica/issues). Upstream: [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy).


<div align="center">

## 🎬 See the duck in action — 5 real-robot videos

<table>
<tr>
  <td width="20%"><video src="https://github.com/user-attachments/assets/d09a6bfe-7a28-4266-ba90-de2b331472ad" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/a604c0e0-af01-413f-ab00-99ccd26db8d5" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/4508d91c-e91c-40f4-8c3e-5ca5a3f959c3" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/fd0dfafd-dd02-4852-940c-33706c287e9d" controls></video></td>
  <td width="20%"><video src="https://github.com/user-attachments/assets/693a3272-d814-42f6-86b3-be5cb3b35b0f" controls></video></td>
</tr>
</table>

<sub>These clips show the real robot. The online demos now match the Robot Console R17 V167 and Training Studio R1.5.27 interfaces. They use simulated data and cannot connect to your robot or run actual training.</sub>

# 🦆 Microduck · Bring a little desktop duck to life

### Flash the image. Connect to Wi-Fi. Open the console. Get your duck moving.

**An integrated ZERO 3W firmware image · Official Microduck 0.15.1 · Feetech HD1910 support · Two Chinese-language consoles**

**Versions: Robot Console R17 V167 · Firmware control.43 · Training Studio R1.5.27 · First-boot Wi-Fi setup R5**

[🦆 **Duck shopping guide**](DUCK-SHOPPING-LIST.en.md) · [📥 Downloads](#downloads-and-versions) · [📚 Guides](docs/README.en.md) · [🛠️ Source](microduck-hd1910/README.en.md) · [🌱 The author's learning journey](docs/AUTHOR-JOURNEY.en.md)

[🎮 Robot Console online demo](https://kissqy.github.io/Microduck-kissqy/) · [🧠 Training Studio online demo](https://kissqy.github.io/Microduck-kissqy/training/)

</div>

> ✅ **Public Image R1 is available!** The integrated image has passed a fresh-card installation test covering boot, Wi-Fi setup, SSH, robot communication, and R17 recognition. **No need to compile the robot firmware yourself.**
>
> 📦 **Robot Console R17 V167 and Training Studio R1.5.27 are ready to download.** Compiled control.43 firmware and both console sources are also published. All downloads are grouped in the table below.

---

## ✨ Skip the software setup from scratch

This little duck can **walk, sit, pick things up, kick a ball, and roll forward**. Add rollers to explore **swizzling, crouching, spinning, and standing up after a fall**. Use a physical gamepad to control it; an independent Training Studio is available when you want to train your own movements.

**Public Image R1 is released and has been tested by reflashing a real board.** It combines the official system with this project's preinstalled robot services and movement models. Flash it onto a Radxa ZERO 3W, connect to Wi-Fi, and open the console to use the prepared software environment without compiling firmware from source.

## 🚀 Start in three steps

1. **Flash the image** — Use the `.img.xz` from the Downloads table. In [Armbian Imager](https://imager.armbian.com/), select a custom image and your microSD card, then flash it. **Manual decompression is unnecessary:** the imager decompresses it while writing.
2. **Connect to Wi-Fi** — On first boot, join the `Microduck-kissqy` hotspot with password `12345678`. Follow the setup page to enter your home Wi-Fi and set a new system login password. If the page does not open automatically, visit `http://10.42.0.1:8088`.
3. **Open the console** — Extract the complete Robot Console ZIP and run `start.cmd`. Connect to the duck, check its calibration, and complete HOME before enabling movement. For training, extract Training Studio and run `start-training.cmd`.

> 🛠️ **Preinstalled software still requires calibration for your assembly.** After assembly or servo replacement, verify every joint's zero position, direction, limits, and the IMU before enabling movement. Do not copy the reference duck's calibration blindly.

<div align="center">
  <img src="docs/assets/wifi-setup-desktop.webp" width="760" alt="Microduck first-boot hotspot and Wi-Fi setup page" />
  <p><sub>First-boot Wi-Fi setup preview. The actual screen is the one shipped with Public Image R1.</sub></p>
</div>

## Downloads and versions

| Package | Purpose | Download |
| --- | --- | --- |
| **System Image R1** · Official 0.15.1 / control.40 | Fresh ZERO 3W installation; robot services and movement models preinstalled | [⬇️ .img.xz](https://github.com/kissqy/Microduck-kissqy/releases/download/public-image-r1/microduck-zero3w-0.15.1-R1.img.xz) |
| **Robot Console R17 V167** | Connect to and operate the duck from your computer; includes the matching firmware payload | [⬇️ ZIP](https://github.com/kissqy/Microduck-kissqy/raw/refs/heads/main/release-assets/Microduck-Console-R17-V167.zip) |
| **Training Studio R1.5.27** | Train, evaluate, and export your own models | [⬇️ ZIP](https://github.com/kissqy/Microduck-kissqy/raw/refs/heads/main/release-assets/Microduck-Training-Studio-R1.5.27.zip) |
| **Compiled Feetech HD1910 firmware control.43** | Upgrade ZERO-side services on an existing system; ARM64 installation package | [⬇️ .tar.gz](https://github.com/kissqy/Microduck-kissqy/raw/refs/heads/main/microduck-hd1910/firmware/console/updates/Microduck-Robotd-FT6-R17.tar.gz) |

**For ordinary use, download the system image and Robot Console.** Training Studio is optional. The standalone firmware matches V167 bundled control.43. Image R1 still includes control.40; install bundled firmware from V167 to upgrade after flashing. No compilation is required.

[Robot Console guide](microduck-hd1910/firmware/console/README.en.md) · [Training Studio guide](microduck-hd1910/training-console/README.en.md) · [Standalone firmware installation and checksums](release-assets/README.en.md) · [Image release guide](docs/releases/public-image-r1.en.md)

Source: [firmware](microduck-hd1910/firmware/README.en.md) · [Robot Console](microduck-hd1910/firmware/console/README.en.md) · [Training Studio](microduck-hd1910/training-console/README.en.md) · [build guide](microduck-hd1910/docs/BUILD.en.md). Current source matches V167 / control.43; duplicate compiled resources are restored when needed.

**V167 update: Fixed body jitter.**

## 🎮 Two consoles, two jobs

| 🐣 Robot Console · R17 V167 | 🧠 Training Studio · R1.5.27 |
| --- | --- |
| Live 3D pose, camera, joint / IMU data, servo calibration | Training tasks, recipes, queue, progress and reward curves |
| Physical gamepad control, firmware maintenance, model status | Visual simulation, policy evaluation, ONNX export |
| No need to compile the ZERO services yourself | Requires a computer with a training environment; the interface itself does not require a ZERO |

**V167 includes 10 movement models: 6 regular models and 4 roller models.** The control service executes the models; you do not have to hand-script servo movements. Roller movements and recovery depend on assembly, calibration, and training and still need careful real-robot evaluation.

**English instructions:** [Robot Console](microduck-hd1910/firmware/console/README.en.md) · [Training Studio](microduck-hd1910/training-console/README.en.md) · [Image setup](docs/releases/public-image-r1.en.md) · [Source / build guides](microduck-hd1910/README.en.md). The application interfaces in the current downloads remain Chinese; the English guides include the original button labels where needed.

## 🔧 DIY HAT and IMU

🎬 **[Watch the complete IMU build · Bilibili](https://www.bilibili.com/video/BV1qapz6PERs/)**

Both the custom HAT and the independent torso IMU have passed extended real-robot use, operate normally, and have no identified issues.

[HAT build timeline](hardware/HAT-TIMELINE.en.md) · [IMU build timeline](hardware/IMU-TIMELINE.en.md) · [Hardware projects and notes](hardware/README.en.md) · [🔧 PCB shopping list (HAT / IMU)](hardware/README.en.md#shopping-list)

## ❤️ Sharing the project and crediting its creators

This is a **personal learning and development project** based on [Pollen Robotics' official Microduck](https://github.com/pollen-robotics/microduck) `0.15.1`, rather than an official release. **Except for explicitly identified upstream and third-party code, models, and assets, the additional firmware adaptations, consoles, tools, documentation, and the author's trained results were developed by this project's author, with AI-assisted programming.**

**Original work that the author owns and can license is provided for personal learning, research, and noncommercial use only. Obtain the author's written permission before commercial use**, including commercial products, paid services, or distribution with robots sold commercially. Upstream Apache-2.0 software, third-party MIT components, official geometry, and other assets retain their own licenses; personal terms do not override third-party rights.

Full scope: [Component licenses](microduck-hd1910/LICENSE) · [Personal noncommercial terms](microduck-hd1910/LICENSES/Personal-NonCommercial.txt) · [Third-party credits](microduck-hd1910/docs/THIRD-PARTY.en.md)

This project grew from one board, one duck, and many debugging sessions. I am an individual maker, with AI helping on parts of the code. I hope these files help you get your own duck moving sooner. Stars ⭐, discussion, and feedback are welcome.

<p align="center"><img width="180" alt="QR code to support the author" src="https://github.com/user-attachments/assets/49054608-65ba-41d6-b3f1-a97941976c4b" /></p>
