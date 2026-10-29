# Native startup servo observations verification

Scope: native diagnostic capture/publication only, based on `bdf494a`. No live hardware
was accessed. API 41 / `0791fcd-feetech-qinmo-control.46`. The mechanism is owned by
[robotd-design.md](../docs/design/robotd-design.md).

The integration fixture is
[`startup-servo-observations.json`](../duck-control/tests/fixtures/startup-servo-observations.json).
The missing-host-IMU serial-peer regression compares its complete observation object
against that fixture after normalizing measured ages. Rows include `check_age_ms`.

## Actual red/green evidence

Before implementing observation capture:

`cargo +1.99.0 test -p duck-control startup_servo_observations -- --test-threads=1`

Three tests failed with `left: Null, right: true/false`: missing host sample,
partial/checksum-invalid replies and torque-only presence. After capture was added,
the same three passed. The final filter has five passing tests, adding stale bus IMU,
latest-request miss/history and unrelated-read aging checks.

Before aging cached RPC reports:

`cargo +1.99.0 test -p robotd startup_servo_observations -- --test-threads=1`

Failed with `frozen cached evidence must age: 0 -> 0`. After the fix it passes for
both a frozen raw snapshot and a startup-only typed sidecar, with at least 30 ms
of age growth across a 40 ms wait. Row ages stay attached to their capture.

Before publishing pending-readiness diagnostics:

`cargo +1.99.0 test -p robotd startup_invalid_imu -- --test-threads=1`

Failed because `bus_status` was `None`. It now passes while retaining the exact
26-read bounded warmup and refusing the invalid quaternion.

## Passing focused checks

- `cargo +1.99.0 test -p duck-control feetech -- --test-threads=1`: 86 passed.
  Includes atomic samples, stale torque-on refusal, checksum failures, the original
  V167 golden UART frames, torque deadlines, model P/gain restoration and IMU contracts.
- `cargo +1.99.0 test -p robotd startup_ -- --test-threads=1`: 11 unit tests and
  one single-instance integration test passed. The real PTY open_bus failure test
  confirms fifteen responding torque replies survive missing IMU discovery; its
  only write is the existing torque-OFF broadcast, with no goals or control ticks.
- `cargo +1.99.0 test -p robotd home_start:: -- --test-threads=1`: seven passed.
- `git diff --check`: clean.

Rust 1.99.0 ran under WSL Linux, using `/root/target-v167-autodetect-dxl2-board`.
Ignored policy ONNX fixtures were copied from the previously verified frozen build
`/root/qinmo-runtime-build-20261027.aexgEQ`; their original pinned payload is
`/root/microduck-build-20261027/inputs/payload.tgz`, SHA256
`370f64af5c690eeebab429f4c59d9c5d00883b5cf1e7f0d0e2c18d4afe45998d`.
Without those fixtures, the alternate-model-P test failed; it passes with them.

## Full robotd baseline comparison

`cargo +1.99.0 test -p robotd --bin robotd -- --test-threads=1` was run on the patched
tree and on a clean `git archive bdf494a` exported to
`/root/startup-native-baseline-20261029`, using identical verified model fixtures.
The clean tree had 296 passed / 20 failed / 2 ignored; the patched tree at the first
full run had 297 passed / the same 20 failed / 2 ignored. Later added focused tests
are covered above. The full suite is not green. Baseline log:
`/root/startup-native-baseline-20261029/robotd-tests.log`.

The identical existing failure names are:

```text
control::joint_uniform_tests::joint_uniform_keeps_manual_lb_teacher_and_returns_to_new_walk
control::selftrained_checks::lb_recovery_test_runs_official_skill_and_returns_to_original_walk
feetech_recovery_tests::explicit_home_after_a_policy_pause_restores_absolute_home_without_replay
feetech_recovery_tests::fallen_lb_preempts_home_ramp_and_drops_same_tick_mode_or_policy_changes
feetech_recovery_tests::fallen_limp_lb_starts_teacher_directly_finishes_home_and_retries_after_select
feetech_recovery_tests::feetech_official_coast_holds_and_resumes_without_reenable
feetech_recovery_tests::home_lb_pending_request_is_cancelled_by_start_home_disable_and_relax
feetech_recovery_tests::home_lb_starts_fallen_finishes_home_and_start_cancels_while_blind
feetech_recovery_tests::limp_lb_rechecks_current_feedback_and_never_replays_a_refused_edge
feetech_recovery_tests::limp_lb_revalidates_after_torque_readback_and_honors_late_init_or_select
feetech_recovery_tests::limp_lb_start_via_padd_init_cancels_even_while_sensors_are_blind
feetech_recovery_tests::policy_lb_returns_walk_and_select_then_limp_lb_can_restart
feetech_recovery_tests::start_home_ends_the_seat_even_when_reads_are_paused
raw_stream::tests::lazy_bus_rpc_preserves_snapshot_fields_without_streaming
raw_stream::tests::offline_control_keeps_acquisition_counts_without_full_reports_or_geometry
stability_tests::feetech_acquisition_ticks_before_first_valid_pose_without_target_writes
tests::a_mode_switch_on_a_limp_robot_swaps_the_policies_and_leaves_it_limp
tests::a_mode_switch_that_arrives_on_a_blind_tick_is_not_lost
tests::r17_empty_snapshot_and_warming_imu_do_not_accept_home
tests::setting_the_mode_accepts_refuses_and_says_which
```

Existing startup/control gates were not weakened to satisfy those tests. Final ARM64
build, ABI inspection, independent reviews and board deployment belong to the coordinator.

## Review follow-up: commissioning initialization failure

The initial commissioning `connect` closure also dropped its local UART owner after
torque readback succeeded and IMU identification failed. The new real-PTY regression
`commissioning_startup_missing_imu_preserves_servo_presence_and_frozen_rpc_age` was
run before the fix and failed with `native.imu_source: Null` versus `"bus"`.

After the fix, the same test passes with an absent calibration file (factory defaults).
It checks fifteen ordered responding rows, null telemetry, configured source/profile,
no ready commissioning snapshot, no control ticks and exactly one torque-OFF broadcast.
After the commissioning task stops publishing, actual busStatus RPC age grows by at
least 30 ms across a 40 ms wait, while captured row ages remain unchanged.

This follow-up only retains diagnostics on the initialization-error path. The
torque-OFF/verification/IMU-identification order, retry interval and existing typed
commissioning `ready`/`read_error` publications are unchanged. `Bus::servo_observations`
copies existing evidence; it performs no UART traffic and creates no control sample.

Focused revalidation after this follow-up:

- `cargo +1.99.0 test -p robotd commissioning_startup_missing_imu -- --test-threads=1`: one passed.
- `cargo +1.99.0 test -p robotd startup_ -- --test-threads=1`: 12 unit + one integration passed.
- `cargo +1.99.0 test -p robotd feetech_commissioning:: -- --test-threads=1`: 18 passed.
- `cargo +1.99.0 test -p robotd feetech_r9_tests:: -- --test-threads=1`: five passed.
- `cargo +1.99.0 test -p duck-control feetech -- --test-threads=1`: 86 passed.
