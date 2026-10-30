# Native controller inputs for the 20261027 software-update path

This is an installation contract and release-preparation audit, not a signed package or an
installer. No board was contacted, and no installed file or prior artifact was replaced.

## Frozen source and scope

`release/native-inputs-20261030` combines `b5b91c4` and `a10fcf7` at merge
`df2877fc751d62d0170bb4d0b38b0143810c3929`, then fixes the real robotd wire boundary at
`665613ac628fdc9b1b71f324809f0c16ce2795f2`.

The latter is the frozen **build source**. A separate audit-only commit may add reports/scripts;
the embedded binary revision remains the frozen build source. Its firmware Git archive is
94,177,280 bytes, SHA256
`19d477da39233009377f6653dff317ba16457534e9e2436545adf40385b02f38`.
The archive and `native-source.json` are under the separate
`artifacts/native-release-prep-20261030` directory. `build_frozen_padd.sh` prepares two independent
source/target slots on F:, fixed Rust 1.99.0 / cargo-zigbuild 0.23.4 / Zig 0.15.1,
`aarch64-unknown-linux-gnu.2.31`, release codegen-units 1. It embeds the source revision and the
fixed commit timestamp (`2026-10-30T23:17:10+09:00`) rather than wall-clock build time.

The production changes from .47 source are padd mapping/control plus `StateDirectory=padd`, and
the mediad address-family allowance only. The sole robotd source difference is a function inside
its existing `#[cfg(test)]` module that exports the actual serializer. No robotd, duck-control or
mediad algorithm is changed or rebuilt for this bundle.

The new ARM64 input is **3,522,648 bytes**, SHA256
`dc1c7043a108964583fc09a11e3d6f87a25e259c346f7ce9154e77500a0881ec`.
Two independently extracted source trees and empty Rust targets produced byte-identical
`padd-arm64-a` and `padd-arm64-b`; each passed QEMU version/help and actual process status checks.
`native-inputs.json` records these verified pins. Both ELF files are AArch64 PIE, use
`/lib/ld-linux-aarch64.so.1`, require GLIBC at most 2.30, have no RPATH/RUNPATH, and need only
libudev.so.1, libm.so.6, libpthread.so.0, libc.so.6 and libdl.so.2. The embedded source revision and
fixed build timestamp were checked. This is an offline input freeze, not hardware acceptance.

The first a smoke attempt exposed an overlong AF_UNIX path because the F: TMPDIR prefix was
longer than the Linux socket limit. Only smoke sockets were moved to short Linux `/tmp` paths;
the completed a ARM binary was preserved, and its verification resumed successfully. Rust/Zig
objects remain on F:. Host checks clear the cross-architecture pkg-config sysroot. The earlier
development binary
`63c0f7b486604552ad4c95309462f3f9653396fbbf779e69023c074c40d558ab` must **not** be published:
its proof fixture used a nested bus report that did not match real robotd.

## Actual robotd compatibility

`robot.policies.enabled` means policies are configured/available. It does not mean the policy
is executing. The real `robot.busStatus` serializer flattens `BusDynamic`: actual execution is
`result.policy_enabled`, and HOME state is `result.homed`; there is no `result.dynamic` wrapper.
The release fix preserves the strict requirement that actual policy execution is explicitly
false, with two advancing held/zero `robot.state` ticks and completed HOME or limp state.

| Existing robotd | IPC API | Source | Exact published binary SHA256 |
|---|---:|---|---|
| .44 | 40 | `0791fcd44f71c705dc8bb0fbcde405593b52d19b` | `edf182edb7bf11aeeeedfdab98170ab2ec48a9ec18417f9b68c744b0a60c6d16` |
| .45 | 40 | `bdf494ab3fd74898b5159419fbffe87113b4097c` | `febf44e2e5d4e573f4513f25a1ae6480b976c3988dd3d5e7cdfcf84d2a2d5063` |
| .46 | 41 | `0583a2a27e1f4cc719e1ac9dafbc1f0876616848` | `3dbbf1723c0315afc15fd9f28787c0cfc194622a67802adcd1f5aa397ea47422` |
| .47 | 42 | `931ce978f2efb27c157b7a583ee402793131a0d2` | `725b74cd93660a312970a44879abe5b75e8becf108499a1b8798961e48ca5f00` |

These sources expose the needed flat `policy_enabled`/`homed`, `robot.subscribe` and advancing
`state.t_ns`, and the normal action calls used by padd. New padd compiles against IPC42 but does
not reject old IPC numbers or invent routes; the new mapping socket has its own private version1.
These are source/wire compatibility findings, not physical acceptance of every old executable.
The updater must still validate the exact installed robotd process/layout and real fresh stopped
state. Startup/commissioning-only states without an ordinary held state stream cannot acknowledge
learning. Unknown or partial facts refuse safely; the API number alone never establishes safety.

`padd/tests/fixtures/robotd-stopped-wire.json` is shared, unmodified serializer output from
`robotd::raw_stream::Bus::report`, `PoliciesResult` and `RobotState`. SHA256:
`aa9036954c808aa251fb3f560a302340538235abf52dc9faf66e93eb6b95e5b3`.
It deliberately retains `policies.enabled=true`, `bus.policy_enabled=false`, `bus.homed=true`.
Studio must consume that distinction without changing the fixture to make policy configuration
false. The export is a frozen capture; age fields need not be byte-identical when regenerated.

## Fixed predecessor padd and layout

The verified 20261027 payload has SHA256
`370f64af5c690eeebab429f4c59d9c5d00883b5cf1e7f0d0e2c18d4afe45998d` and contains:

| Input | Size | SHA256 |
|---|---:|---|
| `./opt/robot/feetech-ft5-r5/padd` | 3,737,592 | `85fc8051df2ce496abf32bbb5eba3c94c44d875365a8f33a2f53f90facba90db` |
| Original `padd.service` payload | 5,543 | `be69d564451cedafeb97f9303703448a6d9517c32c63322ef5dc37d162c0a0ae` |

The existing offline camera audit independently recorded that same padd hash at the installed
ft5-r5 path alongside robotd .46. This audit made no new SSH connection. The new controller
updater may admit this **one** reviewed padd predecessor; unknown hashes are refused. Historical
repository `BUILD.json` entries `356ed712...` and `6f8d78d2...` do not match this payload and are
not additional approved inputs. The .28/.29/.30 robotd-only software releases do not establish a
different padd predecessor; do not infer one from a software version string.

The active program target is fixed at `/opt/robot/feetech-ft5-r5/padd`, root-owned executable.
The existing generated drop-in is
`/etc/systemd/system/padd.service.d/zzzz-r17-0151.conf`. `packaging/gamepad.py` generates:

```text
/usr/bin/python3 /opt/robot/feetech-ft5-r5/startup_gate.py --timeout 60 --exec
  /opt/robot/feetech-ft5-r5/padd --socket /run/robotd.sock
  --config /etc/robot/feetech-ft5/robotd.toml --max-head <saved head_rad>
  --input-settings /etc/robot/feetech-ft5/pad-settings.json
```

`startup_gate.py` replaces itself with padd, retaining the PID. Verify both the fixed systemd
wrapper and the final `/proc/PID/exe` inode/hash/cmdline. Preserve the selected head/mouth settings,
configuration, calibration, startup gate, GameSir bridge and existing ExecStart/drop-ins. Do not
replace that wrapper with the generic upstream `/opt/robot/daemon/current/bin/padd` unit command.

## Minimal service changes and permissions

For the known installed unit, add a fixed, root-owned 0644 updater-owned drop-in such as
`/etc/systemd/system/padd.service.d/zzzzzz-microduck-pad-mapping.conf`:

```ini
[Service]
StateDirectory=padd
```

Keep `RuntimeDirectory=padd`, `User=padd`, `Group=padd`, supplementary `input robot`, and existing
hardening. systemd creates `/var/lib/padd` for the service and makes it writable despite
`ProtectSystem=strict`; manual directory creation is not a replacement for that namespace rule.
The full source units already contain `StateDirectory=padd`, but applying the small drop-in
preserves the established 27 image's startup command. See the official
[StateDirectory semantics](https://www.freedesktop.org/software/systemd/man/latest/systemd.exec.html).

Require, and health-check:

| Path | Owner/group | Mode / behavior |
|---|---|---|
| `/run/padd/mapping.sock` | padd:robot | 0660, one-line v1 capability |
| `/run/padd/pad.sock` | padd:robot | existing0660 read-only `pad.input` stream |
| `/run/padd/mapping-state.json` | padd:robot | 0640; version1/current PID/device/profile/gate, no token |
| `/var/lib/padd` | service padd identity | persistent directory; never deleted on update/rollback |
| `/var/lib/padd/mappings.json` | padd | 0600, schema1, bounded262144B, atomic replacement |

The profile directory and file are padd-owned, unlike root-owned executable/unit targets.
The existing native transaction's root-only `read()` is therefore not an appropriate unchanged
validator for these user profiles. If backing them up, validate fixed paths, no symlinks,
expected padd UID, safe mode, size/schema and parent ownership explicitly. Keep byte hashes and
absence markers; never replace a later user's profile with an older snapshot. Existing
duck-echo uses supplementary `robot`, which is necessary to read the state file.

For mediad, preserve the executable (known hash
`0417c8c2dc5a63dd2e1c2a65d652f31609c232abab95d50e4acbd1faab0f742b`), source selection,
camera/ISP settings and installed ExecStart. The reviewed unit adds only AF_NETLINK. For a
verified predecessor allow-list `AF_UNIX AF_INET AF_INET6`, an owned drop-in can reset that
property and set `AF_UNIX AF_INET AF_INET6 AF_NETLINK`; refuse unknown custom deny/allow policies
instead of silently overriding them. Do not choose `media.source="camera"` on every board:
the separate camera fix required actual sensor discovery and preserved boards without cameras.

## Required transaction extensions

The existing `native_update.py` and publisher option are fixed to **robotd only**: reviewed
robotd hash/size, one executable target, four exact parameter/calibration files, fixed journal
schema, guarded controller restoration. They cannot safely install padd merely by appending an
unsigned file to a software package. Its old NATIVE-UPDATE.md also contains stale .45 values;
the live reviewed module pins .47. The controller bundle must extend this verified boundary:

1. Publisher binds the reviewed padd bytes, source/reproducibility metadata, exact size/hash and
   exact owned drop-in contents into signed suite hashes. No client path, URL, service, force or
   arbitrary command argument may select inputs. No signing or publication was performed here.
2. Install paired Studio/helper/voice support first. It must handle native-unavailable honestly,
   preserve `policies.enabled=true` semantics, honor mapping/native-update exclusion, and follow
   saved R3/Select bindings. New native plus old voice is not an accepted learning combination.
3. Reuse the updater operation/service-management locks, exclusive motion lease, durable busy
   guard and controller start exclusions. Reject active mapping, web motion, recording, servo/
   calibration sessions, voice/music and unfinished earlier transactions. Require explicit user
   support/unload confirmation and fresh genuine stopped/unloaded feedback; never issue HOME,
   enable, skill or movement commands to make an update pass.
4. Preflight exact installed padd hash/path/arguments and process generation, known robotd
   compatibility, unit/drop-in hashes and ownership, battery/disk, and presence/absence/hash of
   persisted profiles and existing settings. Unknown/custom layouts refuse.
5. Journal and fsync backups **before** mutation: old executable/mode, owned drop-ins with
   absence markers, relevant existing service states/generations, profile snapshot/absence,
   immutable hashes of user configuration/settings/calibration, and original mediad/robotd
   identities. Do not reuse the strict old robotd journal shape without a reviewed schema/
   compatibility/recovery update to all helper and voice busy readers.
6. Exclude starts, stop and confirm padd/voice/GameSir controller states, and recheck unload,
   generations and unchanged user data. Atomically replace the fixed padd target; atomically
   write only owned service fragments; fsync files/directories; `daemon-reload`. For padd-only
   scope robotd must not restart or be replaced. Apply mediad's unit change without rebuilding
   its algorithm or changing media config.
7. Start only recorded previously active services under transaction exclusions/restore permit.
   The startup gate can wait60s or exit78; a mere listening pathname is not readiness. Restore
   service states exactly, and preserve inactive/absent states. If a known startup-only system
   cannot run padd yet, distinguish staged bytes from verified running capability; never claim
   complete runtime health or bypass the startup gate.
8. Verify the running padd PID/invocation/start-time, `/proc/PID/exe` inode and target SHA, fixed
   cmdline and embedded source identity; require a bounded successful mapping `status` reply
   version1 and consistent current-PID voice state/permissions. No controller or hand is required
   for `device:null` status. Check old raw tap remains read-only. Health checks must not call
   begin/preview/save/reset, create a profile, press a button or request robot movement.
9. Verify robotd identity/config/calibration/settings unchanged. For mediad, verify expected
   address families and unchanged executable/config with a new service generation if restarted.
   Signalling/active alone does not certify camera frames; physical/video acceptance is separate.
10. Commit updated protected-service baseline only after all checks succeed, then remove owned
    guards durably. The stage1 software engine protects robotd/padd identities, so an authorized
    padd generation change must update its recorded baseline, not disable that check.

On any failure or power interruption, keep the busy journal and controller exclusion. Restore
and verify the exact old padd bytes and owned fragment states before restoring controllers;
remove a new fragment only if its current hash is the updater's expected hash. Unknown later
changes stay blocked for recovery. Preserve `/var/lib/padd` and profiles on rollback, including
newer user mappings; old padd ignores them, and UI must report native mapping unavailable.
Do not restore user config/calibration/settings bytes over later edits. Recovery must survive
reboot and be idempotent across file replacement, reload, partial service restore and cleanup.

## Verification to retain with the release

After host maintenance, 113 fresh padd tests, host actual process smoke and both ARM64 QEMU
actual process smokes passed. Independent release builds took 53.12s and 49.47s, with the same
non-fatal Zig linker warning about a deprecated optimization setting. The tests and smoke
sockets used Linux `/tmp`; compiler objects used separate F: targets. Before the WSL failure,
113 padd tests passed; actual flat serializer export/consumer passed;
production padd clippy with warnings denied passed. The red/green case first changed the fixture
to real top-level fields: old b5 proof rejected a genuinely stopped report; the two-line consumer
fix made it pass. Keep the shared fixture unmodified in the Studio cross-language/browser checks.

An additional raw_stream module run had6 passed/2 failed; clean merged baseline df2877f had
5 passed/the same2 failed. These are inherited
`lazy_bus_rpc_preserves_snapshot_fields_without_streaming` (old configuring_uart expectation)
and `offline_control_keeps_acquisition_counts_without_full_reports_or_geometry` (8 vs7 reads).
No production robotd change was made to hide those failures, and there is no full-workspace-green
claim. Keep the frozen independent ARM64 build/hash equality, ELF/GLIBC/needed libraries,
QEMU CLI and actual process status smoke with source identity, plus paired Studio contract tests.
Temporary-root transaction acceptance must cover failed health, profile/group
permissions, wrong hashes, unit rollback, crash/reboot recovery and unchanged robotd identity.

## Disposable build directories

The one-off build uses independent Rust targets rather than reusing one target to claim
reproducibility. All shipping binaries, source archive, native input manifest and logs are
outside those targets. Nonshipping `padd-host-proof` and `padd-contract-fixture` executables
were also copied outside them for independent review. Their paths/hashes and each disposable
directory's realpath/allocated/apparent bytes are in `build-cache-inventory.json`.

Future invocations require at least 20 GiB free on both E: and F: before creating build
directories and set `CARGO_INCREMENTAL=0` for this one-off verification. This is a maintenance
admission policy, not a hard budget provided by Cargo, and adds no cron or automatic cleanup,
disk growth or cache relocation. The already frozen `dc1c7043...` a/b binaries used the previous
1 GiB E: / 10 GiB F: admission guard; this wrapper-only change does not rebuild or alter them.

The nine paths below share the fixed prefix
`/mnt/f/chengshenzhilu/Robot/Microduck/artifacts/native-release-prep-20261030/`.

| Disposable directory | Allocated bytes |
|---|---:|
| `frozen-a` | 95,440,896 |
| `frozen-b` | 95,440,896 |
| `linux-target-a` | 1,115,439,104 |
| `linux-target-b` | 209,162,240 |
| `zig-global-cache` | 40,210,432 |
| `zig-local-cache-a` | 0 |
| `zig-local-cache-b` | 0 |
| `tmp-a` | 0 |
| `tmp-b` | 0 |

Total allocated: 1,555,693,568 bytes. These directories are reconstructible from the frozen
source archive and build script; they are not permanent release inputs. The release owner
will remove only this explicit list after Studio/independent checks stop using it. No cleanup
was executed in this input-freeze commit. Preserve the source archive, both ARM binaries,
proof executables, manifests, scripts and evidence logs; do not clear either reserved E: target.
