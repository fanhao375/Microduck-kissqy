# Native physical-controller mapping verification — 2026-10-30

> Historical development evidence, superseded by the native release preparation audit.
> Release auditing found that this revision's robot proof fixture nested `BusDynamic` fields,
> whereas real robotd flattens them. The original `63c0f7...` binary must not be shipped.
> The release preparation branch fixes the consumer, shares an actual robotd-serializer fixture
> with Studio, and rebuilds a separately identified candidate; see
> [`native-release-prep-20261030`](../native-release-prep-20261030/).

Branch: `feat/pad-remap-native-20261030`, based on `931ce97`.
The mechanism and contract are documented in
[`pad-mapping-design.md`](../../docs/design/pad-mapping-design.md).
All commands ran in isolated WSL Ubuntu-22.04 sources at
`/root/microduck-remap-native-20261030`. No robot connection, deployment, push or release occurred.

## Final results

| Check | Result / evidence |
|---|---|
| `cargo +1.99.0 test -p padd --offline` | **112 passed, 0 failed**, [`padd-tests.log`](padd-tests.log). Includes an unprivileged uid65534 child proving actual Linux EACCES preservation. |
| `cargo +1.99.0 fmt --package padd` | Formatted; source synchronized back to the dedicated worktree. |
| `cargo +1.99.0 clippy -p padd --bin padd --offline --no-deps -- -D warnings` | Passed without warnings. |
| `cargo +1.99.0 clippy -p padd --all-targets --offline --no-deps -- -D warnings -A clippy::approx_constant` | Passed. The allowance is for pre-existing numeric quaternion constants in `roller_test.rs`; new code requires no lint allowance. |
| Actual host `padd` process | [`process-smoke.json`](process-smoke.json): real main loop, temporary sockets/config/state directory, zero input devices, zero movement/skill requests. Only startup `robot.mode` was observed; `begin` with no device was rejected before robot proof. |
| ARM64 release build | Passed, [`arm64-build.log`](arm64-build.log), [`arm64-elf.txt`](arm64-elf.txt). One Zig linker diagnostic says its optimization setting `1` is deprecated; no build error. |
| Target GLIBC symbol ceiling | `GLIBC_2.30`, [`arm64-glibc.txt`](arm64-glibc.txt), below the selected `2.31` floor. |
| QEMU ARM64 CLI | [`qemu-version.txt`](qemu-version.txt) and [`qemu-help.txt`](qemu-help.txt) pass, including both new CLI arguments. |
| `git diff --check` | Passed. |
| Independent review | Separate reviewer ran all 112 tests, actual final host process smoke and diff checks; no unresolved Critical, Important or Minor findings. |

The target executable is retained locally at
`/root/microduck-remap-native-20261030/verification/pad-remap-20261030/padd-arm64`; it is intentionally
not committed as a release artifact. SHA256:

```text
63c0f7b486604552ad4c95309462f3f9653396fbbf779e69023c074c40d558ab
```

## Reproduction

Host tests use `CARGO_TARGET_DIR=/root/target-v167-autodetect-dxl2-board` and Rust `1.99.0`.
The production-process smoke is separate from synthetic mapping tests:

```sh
cargo +1.99.0 build -p padd --bin padd --offline
python3 padd/tests/process_smoke.py --binary /root/target-v167-autodetect-dxl2-board/debug/padd
```

The script refuses a host containing `/dev/input/event*`. Its robot fixture accepts read-only
methods only, and records any unexpected motion/skill method as a failure.

ARM64 build uses pinned Zig `0.15.1` in `/root/padd-remap-buildtools`, cargo-zigbuild `0.23.4`, and
the existing ARM64 libudev sysroot:

```sh
export PATH=/root/padd-remap-buildtools/bin:/root/.cargo/bin:/usr/local/bin:/usr/bin:/bin
export PKG_CONFIG_ALLOW_CROSS=1
export PKG_CONFIG_SYSROOT_DIR=/root/.cache/duck-sysroot
export PKG_CONFIG_PATH=/root/.cache/duck-sysroot/usr/lib/aarch64-linux-gnu/pkgconfig
export CARGO_TARGET_DIR=/root/target-v167-autodetect-dxl2-board
export CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1
cargo +1.99.0 zigbuild --locked --offline --release --jobs 2 \
  --target aarch64-unknown-linux-gnu.2.31 -p padd --bin padd
```

QEMU used `/tmp/microduck-updater-spike-root` plus the cached ARM64 libudev and libcap search paths:

```sh
qemu-aarch64 -L /tmp/microduck-updater-spike-root \
  -E LD_LIBRARY_PATH=/root/.cache/duck-sysroot/usr/lib/aarch64-linux-gnu:/root/duck/ir/x/usr/lib/aarch64-linux-gnu \
  /root/target-v167-autodetect-dxl2-board/aarch64-unknown-linux-gnu/release/padd --version
```

## Red/green evidence and exercised boundaries

The development session observed each new behavior fail before its implementation. Initial
mapping tests failed for missing AB normalization, trigger/hat normalization, validation and
bus/capability identity; session tests failed against unavailable stubs before lease, persistence
and raw history were implemented. Subsequent regression cycles included:

| Regression | Observed red failure | Final behavior |
|---|---|---|
| Default disconnect | Entered calibration gate and bypassed ordinary pad-loss behavior | Default source retains its established loss path. |
| Press/release or axis pulse inside a tick | Gate resumed too early | Every non-neutral raw pulse restarts stable-neutral timing. |
| Repeated reader errors | Generation changed repeatedly | Output clearing occurs only on a transition. |
| Select state crossing learning | `shutdown_sent` remained set | All old Select state is cleared. |
| Default raw reader unavailable | Default gilrs driving became blocked | Extra observer failure does not disable an unmapped device; potentially applicable saved profiles fail closed. |
| `held` while actual policy is enabled | Stopped proof incorrectly accepted | Explicit `robot.busStatus.dynamic.policy_enabled=false` is required. |
| Corrupt existing store | Save overwrote original bytes | Save/reset reject with `persistence`; original bytes remain untouched. |
| Slow byte-by-byte mapping/robot replies | Per-read timeout allowed total deadline extension | Absolute request/proof deadlines are enforced. |
| Full robot Unix listen backlog | Blocking connect exceeded proof budget | Nonblocking connect immediately fails closed; no background worker leaks. |
| Voice-state publication failure | No output clearing occurred | Five safe latch-clearing notifications precede publication and acknowledgment. |
| Pose reset | Default `PoseParams` retained `active:true` | Wire parameters explicitly contain `active:false`. |
| Custom axis calibration | Global deadzone removed small calibrated input | Calibrated values enter actual control without a second deadzone. |

Real Unix-socket tests check permissions, token omission, pending command responses, unchanged
read-only raw stream behavior and two fresh read-only robot proof ticks. Synthetic controls
exercise optional bindings, code/range/role duplication refusal, preview, save, restart, reset,
lease expiry, identity isolation, history limits, SYN_DROPPED and device replacement.

Studio also ran its Python `PadMapping`/`MappingClient` contract probe against the opt-in
`mapping::native::tests::cross_language_fixture`, using the production mapping socket handler.
The companion Studio worktree owns that probe and its evidence; it covers learning, preview,
normalization, save/reload, disconnect/reconnect, cancel/reset and the five-second lease.

These checks do not substitute for physical pad/robot acceptance. Actual evdev USB/BT radio
reconnects, multiple controller models, the GameSir bridge and complete voice/UI behavior with
the paired Studio implementation remain bench acceptance work. The bridge exposes a fixed USB
identity, so upstream USB/BT transport cannot be inferred or promised as distinct profiles.
