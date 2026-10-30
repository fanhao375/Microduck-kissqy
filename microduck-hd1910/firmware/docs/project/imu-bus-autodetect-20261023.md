# Bus IMU autodetection verification, 2026-10-23

This fork builds on `4734cbb0aac075335ef206d6557d0b6a3eb9e0a9`. The mechanism and
supported layouts are owned by [robotd-design §2.6](../design/robotd-design.md#26-where-the-trunk-imu-is-wired).
The build identity is `4734cbb-feetech-imu-auto-control.44`.

Verification used Rust 1.99.0 on Ubuntu 22.04, without a board. Real ignored ONNX fixtures
were restored from the previously verified image payload; model contracts and hashes were
not disabled or replaced by dummy files.

| Check | Result |
|---|---|
| `cargo test -p duck-control -p robotd-params -- --test-threads=1` | 251 control tests and 110 parameter tests passed; eight pre-existing recurrent-policy integration tests ignored |
| DXL2 codec / mixed UART tests, included above | 19 passed, covering byte stuffing, CRC/noise, explicit selection, read-only restrictions, shared deadline, missing/error replies, freeze detection, protocol binding and diagnostic accounting |
| `robotd` unit tests, each in a fresh process | New tree: 296 passed, two ignored, 17 failed; pinned baseline under identical conditions: 295 passed, two ignored, the same 17 failed. No new failing test names |
| New DXL2 commissioning PTY flow | Passed: autodetect, samples, mount verification/persistence and fail-stop on servo error; original Feetech flow also passed |
| `cargo test -p robotd --test single_instance --test updater_gate` | 18 passed |
| `cargo clippy -p duck-control -p robotd-params -p robotd --all-targets` | Exit 0, with existing style/large-enum/test warnings; this is not a warnings-as-errors result |
| `git diff --check` | Passed |

The baseline's remaining failures include model fixture/control expectations, stale startup
report assertions and startup/control timeout assertions. Process isolation removes some shared
deployment-state interference but does not make the baseline suite fully pass. The unchanged
failures remain visible; no production assertions or tests were weakened to obtain this result.

Independent specification and quality reviews found no remaining P1/P2 issues after the
noise/partial-read, unsupported layout and acquisition-accounting fixes. Physical FT6, DXL2,
I²C, standing and walking acceptance of this build remains outstanding.
