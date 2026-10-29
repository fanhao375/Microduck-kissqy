# Independent startup bus-IMU diagnostics — native verification

Native API 42, build `0791fcd-feetech-qinmo-control.47`.
Mechanism and install consequences: [robotd design](../docs/design/robotd-design.md).

## TDD evidence

- Before production changes, the real PTY zero-servo/FT6 motion-startup test failed:
  `native.imu_observation.attempted` was null rather than true. The report retained
  the original torque-readback refusal. After implementation the same test passed.
- Added sample-read-timeout coverage failed because recognized FT6 identity was
  dropped; preserving diagnostic identity independently made it pass.
- Added uncommissioned-motion coverage failed because validation returned before
  UART acquisition; the read-only path now preserves the original refusal and passes.
- Added replacement-report coverage failed because a prior startup capture leaked
  into a report without IMU evidence; the cache-presence guard now passes.

## Commands and results

Run with Ubuntu-22.04 WSL as root from this worktree's `microduck-hd1910/firmware`:

```sh
export PATH=/root/.cargo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export CARGO_TARGET_DIR=/root/target-v167-autodetect-dxl2-board
cargo test -p duck-control independent_imu -- --nocapture
cargo test -p robotd independent_imu -- --nocapture
cargo test -p duck-control feetech -- --test-threads=1
cargo test -p robotd feetech_r9_tests -- --test-threads=1
cargo test -p duck-control startup_ -- --test-threads=1
cargo test -p robotd startup_servo_observations_age -- --nocapture
cargo test -p duck-ipc-proto --lib
```

Results respectively: 6/6, 2/2, 92/92, 7/7, 7/7, 1/1, 54/54 passed.
The 92-test run used the ignored `packaging/models/1cbf8730eadd75a5704332c9f3c10f60/policy.onnx`
from `/root/microduck-startup-status-20261029.w93jfP`; without it the existing alternate-P
model test fails to load its fixture. No model binary is part of this commit.

The PTY fixture covers motion and factory-default commissioning with zero/partial
servos, no devices, all servos with missing/unknown IMU, invalid data/status/reserved
bytes, frozen RPC capture age, selected host source, configured Qinmo and automatic
DXL2 transport. Requested packets are checked for read-only IMU acquisition; original
startup torque/readiness/goal invariants remain covered by the existing tests.
[The wire fixture](../duck-control/tests/fixtures/startup-imu-observation.json) is
asserted against the normalized real PTY busStatus observation.

`git diff --check` passed. `cargo fmt --all -- --check` still reports repository-wide
pre-existing formatting differences (first in `duck-control/src/deployment.rs:103`);
new diagnostic module and new test blocks were formatted with rustfmt. The full
workspace suite was not rerun; the parent task retains the known inherited baseline.
ARM build, signed delivery and hardware acceptance are separate from these host tests.
