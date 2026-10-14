[简体中文](README.md) | **English**

# 🦆 Microduck HD-1910 · IMU module edition

**Wire an off-the-shelf IMU module to the board with four wires and run the full HD-1910 duck stack.** No bus IMU board to build, no firmware to flash. Optional servo tuning and a UDP pad / pad-loss sit come with it.

Maintained by [microduck-replica](https://github.com/fanhao375/microduck-replica) (the Microduck replica hub: hardware, BOM, bring-up records, pitfalls); every release is tested on our own duck first.

## What it does

- **IMU module straight on the board's I²C**: BNO085 / BNO086 or LSM6DSV16X, one line in `robotd.toml`. Leave it out and it reads the bus IMU board, exactly like upstream;
- **Servo tuning (optional)**: one command sets P, D and torque limit for all servos from the next HOME; another sets the joint Walk model's head/legs filters. Unset, it is upstream's V167 byte for byte;
- **Pad-loss sit and UDP pad**: when the pad that is driving goes away, the duck asks and sits down after a second; a pad on a laptop can drive it over the LAN (upstream's official `duckctl udp-pad`, off by default);
- **One-command installer**: checks the version and the files, relaxes the robot according to its state, backs up, waits for robotd's four startup steps and checks them, and rolls back by itself if any step fails; `check` shows what is installed, `rollback` returns to the state before the install;
- **Tracks the latest upstream release**: currently R17 V167 (control.43), including upstream's jitter fix (servo D=0 and torque limit 1000 on every arm);
- **Step-by-step instructions** (Chinese, with an English summary): wiring, overlays, build, install, confirming the IMU mount.

## Status (2026-10-14)

| | Status |
|---|---|
| BNO085 + V167 | Installed on our duck's new card on 10-14; it has stood and walked |
| Servo tuning | Unit-tested (without the file, byte for byte upstream V167); **not yet on a robot** |
| Pad-loss sit, UDP pad | Unit-tested and run end to end on a host; **not yet on a robot** |
| Installer (whole set) | Tested on fake boards; **this version has not run on a real board** |
| LSM6DSV16X | Written and tested against a fake bus only; not yet on a robot |

## Quick start

1. **Wire it**: the module's 3.3 V, GND, SDA and SCL to header pins 27 / 28 (I2C4) with the `i2c4-m0` overlay; with the official HAT, J5 (I2C3) works too;
2. **Upgrade to V167 first**, with "install/upgrade firmware" in upstream console 1.0.167 (see the [upstream homepage](README-kissqy.en.md));
3. **Build and install**: `sh scripts/cross-sysroot.sh`, then `cargo board --locked -p robotd -p robotctl -p padd`; copy them and the five packaging scripts to the board and, with servo power on, run
   ```bash
   sudo bash imu-module-install.sh install ~/replica bno08x /dev/i2c-4 0x4B
   ```
4. **Confirm the IMU mount**, then HOME and walk;
5. Tune (`service.sh servo-parameters`) or switch on the UDP pad (`service.sh netpad on`) when you need them.

Details and caveats: **[IMU-MODULE.md](IMU-MODULE.md)**.

> Once installed, **do not press "install/upgrade firmware" in the upstream console**: it reinstalls the stock robotd, which cannot read the module, and the stock padd, and deletes the tuning file. If you do, run the installer again and set the parameters again.

## Related

- **[microduck-replica](https://github.com/fanhao375/microduck-replica)**: the replica hub — hardware, BOM, bring-up records, pitfalls
- **[IMU-MODULE.md](IMU-MODULE.md)**: full instructions (wiring, install, IMU mount, tuning, UDP pad), what changed, licensing
- [Upstream homepage](README-kissqy.en.md): system image, console ZIPs, videos, shopping list (kept unchanged)
- Report problems: [microduck-replica Issues](https://github.com/fanhao375/microduck-replica/issues)

## Origin and licensing

- Based on R17 V167 of [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy). **This is not an official kissqy release** and he has not reviewed it; his homepage is kept unchanged as [README-kissqy.en.md](README-kissqy.en.md).
- The repository is mixed-license: `microduck-hd1910/LICENSE`, `LICENSES/` and every third-party notice are kept as they are. Our changes to the Rust workspace (robotd and friends), `packaging/` and `deploy/` are Apache-2.0; this page is ours.
- The runtime descends from Pollen Robotics' official [microduck](https://github.com/pollen-robotics/microduck) (Apache-2.0); padd's UDP pad and pad-loss sit are ported from its 0.17.1.
