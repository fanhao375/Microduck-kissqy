# Qinmo IMU selection and verification

Install the matching runtime and Studio together. This runtime reports build
`0791fcd-feetech-qinmo-control.45`; its IPC API_VERSION is 40. An older binary cannot implement
the new data profile merely because a configuration key was saved.

On the IMU webpage, select **Qinmo** once and save. The matching Studio operation stops the
main program through its commissioning/torque-off gates, saves the fixed profile choice and
reconnects. Startup then reuses that saved choice automatically. Existing FT6/imu_to_dxl
installations keep **Legacy**, with their existing automatic wire-protocol discovery.
The [runtime mechanism](../design/robotd-design.md#26-where-the-trunk-imu-is-wired) owns the
supported layouts, units, discovery packets and freshness limits.

The saved native-bus settings are:

```toml
[bus]
protocol = "feetech"
imu_protocol = "auto"
imu_profile = "qinmo"

[trunk_imu]
source = "bus"
```

Keep the installed port and calibration path. `imu_protocol="feetech"` is also supported;
`"dxl2"` is refused for Qinmo. The webpage performs this saved selection; routine setup does
not require SSH, manual file editing or selecting a protocol at each boot. The runtime has
no live hardware-switching RPC.

Changing Legacy ↔ Qinmo invalidates the effective bus mount, even though both can use
Feetech packets. After reconnecting in commissioning, physically check the sensor axes and
mount, then run the webpage's IMU mounting verification. This is the existing commissioning
`bus verify-imu --confirmed [--mount-wxyz w,x,y,z]` operation. Success persists both
`imu_bus_protocol="feetech"` and `imu_bus_profile="qinmo"`, with motion still disabled.
Use the existing enable-motion flow only after that verification. Returning to Legacy also
requires verification; separate I²C mount slots are preserved.

Before discovery succeeds, `configured_imu_profile` still shows the saved choice so it can
be corrected from the webpage. After discovery, `imu_bus` must show Qinmo, Feetech,
`identification="configured_profile"`, empty identity and a gyro scale near 0.06097561 °/s
per count. This is configured compatibility, not authenticated vendor identification.
The top-level effective `imu_mount.verified` must be false on a binding mismatch, even if
the raw saved calibration contains an earlier `imu_mount_verified=true`.

Offline checks use Rust 1.99+, without a robot:

```sh
cargo test -p duck-control -p robotd-params -- --test-threads=1
cargo test -p robotd qinmo_ -- --test-threads=1
cargo test -p robotd configured_qinmo_profile -- --test-threads=1
cargo test -p robotd --test single_instance --test updater_gate
```

PTY peers in these tests are specification models of the published protocol, not manufacturer
firmware. They exercise discovery, units, ready identity, rejected malformed/status frames,
counter wrap/freeze, the fifteen-servo ordering/deadline and calibration persistence.
Physical Qinmo/FT6/DXL2/I²C acceptance and robot motion remain separate, unperformed checks.
