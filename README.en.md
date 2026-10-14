[简体中文](README.md) | **English**

# 🦆 Microduck HD-1910 · IMU module edition

**Wire an off-the-shelf IMU module to the board with four wires and run the full HD-1910 duck stack.** No bus IMU board to build, no firmware to flash.

Maintained by [microduck-replica](https://github.com/fanhao375/microduck-replica) (the Microduck replica hub: hardware, BOM, bring-up records, pitfalls); every release is tested on our own duck first.

## What it does

- **IMU module straight on the board's I²C**: BNO085 / BNO086 or LSM6DSV16X, one line in `robotd.toml`. Leave it out and it reads the bus IMU board, exactly like upstream;
- **One-command installer**: checks the version and the files, backs up, and rolls back by itself if any step fails; `check` shows what is installed, `rollback` restores it;
- **Tracks the latest upstream release**: currently R17 V167 (control.43), including upstream's jitter fix (servo D=0 and torque limit 1000 on every arm);
- **Step-by-step instructions** (Chinese, with an English summary): wiring, overlays, build, install, confirming the IMU mount.

## Status (2026-10-14)

| | Status |
|---|---|
| BNO085 | Has run on our first duck (V164 build, since 10-12) |
| Current build (V167) | Unit tests match stock upstream V167; builds for arm64; **goes on the robot tonight**, results posted here |
| LSM6DSV16X | Written and tested against a fake bus only; not yet on a robot |

## Quick start

1. **Wire it**: the module's 3.3 V, GND, SDA and SCL to header pins 27 / 28 (I2C4) with the `i2c4-m0` overlay; with the official HAT, J5 (I2C3) works too;
2. **Upgrade to V167 first**, with "install/upgrade firmware" in upstream console 1.0.167 (see the [upstream homepage](README-kissqy.en.md));
3. **Build and install**: `cargo board -p robotd -p robotctl`, copy to the board and run
   ```bash
   sudo bash imu-module-install.sh install ~/imu-module bno08x /dev/i2c-4 0x4B
   ```
4. **Confirm the IMU mount**, then HOME and walk.

Details and caveats: **[IMU-MODULE.md](IMU-MODULE.md)**.

> Once installed, **do not press "install/upgrade firmware" in the upstream console**: it reinstalls the stock robotd, which cannot read the module. If you do, just run the installer again.

## Related

- **[microduck-replica](https://github.com/fanhao375/microduck-replica)**: the replica hub — hardware, BOM, bring-up records, pitfalls
- **[IMU-MODULE.md](IMU-MODULE.md)**: full instructions, what changed, licensing
- [Upstream homepage](README-kissqy.en.md): system image, console ZIPs, videos, shopping list (kept unchanged)
- Report problems: [microduck-replica Issues](https://github.com/fanhao375/microduck-replica/issues)

## Origin and licensing

- Based on R17 V167 of [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy). **This is not an official kissqy release** and he has not reviewed it; his homepage is kept unchanged as [README-kissqy.en.md](README-kissqy.en.md).
- The repository is mixed-license: `microduck-hd1910/LICENSE`, `LICENSES/` and every third-party notice are kept as they are. Our changes to the Rust workspace (robotd and friends), `packaging/` and `deploy/` are Apache-2.0; this page is ours.
- The runtime descends from Pollen Robotics' official [microduck](https://github.com/pollen-robotics/microduck) (Apache-2.0).
