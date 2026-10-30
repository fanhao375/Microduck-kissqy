# Public source export and historical binary provenance

This branch is a clean export with the already public fork baseline
`e376346723e1670a649b2a00e4dcec0f3d2a46cd` as its sole parent. The original local
release/audit tip was `6c6056ecb320a3f88efb822bc6c8a039b0703004`. The export copies its
tree, then restores the entire hook-protected `microduck-hd1910/firmware/console`
directory and these two protected paths to that public baseline:

- `microduck-hd1910/firmware/packaging/test_tunable_ui.mjs`
- `microduck-hd1910/firmware/packaging/test_tunable_console.py`

Both packaging test paths are absent in the public baseline and remain absent.
The only protected tree difference removed from the local release is the 15-line
mapping explanation in `console/README.zh-CN.md`. No padd implementation or test
depends on it. No local release commits or protected modification history are
parents of this export; no hook is changed or bypassed.

All other existing file blobs and modes match `6c6056e`. All program/build inputs
outside protected paths and the verification directory also match the original
build source `665613ac628fdc9b1b71f324809f0c16ce2795f2`. Only this export note and the
public-source reproduction entry point are added to the copied release tree.

## Distinguish source publication from embedded build identity

The frozen ARM64 binary is unchanged: 3,522,648 bytes, SHA256
`dc1c7043a108964583fc09a11e3d6f87a25e259c346f7ce9154e77500a0881ec`.
Its historical embedded revision is `665613ac628fdc9b1b71f324809f0c16ce2795f2`,
with timestamp `2026-10-30T23:17:10+09:00`. The two actual frozen builds and tests
used that original source archive. This export does **not** claim that commit665
is publicly reachable, and the public export commit has a different Git identity.

The historical `build_frozen_padd.sh` and `verify_frozen_inputs.py` retain the
original complete archive hash. A public export archive has different documentation
and audit files and must not be renamed or represented as that original archive.

`build_public_padd.sh <full-public-export-commit> a` is a separate entry point:
it archives the supplied fetched public commit, builds its program bytes with the
original explicit `DUCK_REVISION`, `DUCK_BUILD_TIME` and `SOURCE_DATE_EPOCH`, and
refuses success unless the result matches the frozen SHA and size. The resulting
JSON records the actual public source commit separately from the original embedded
identity. Slotb uses an independent source/target directory; it must not reuse
slota's Rust target to claim reproducibility. For example, after fetching this
branch, run from its Git checkout on the pinned WSL build environment:

```bash
public_ref=$(git rev-parse HEAD)
bash microduck-hd1910/firmware/verification/native-release-prep-20261030/build_public_padd.sh "$public_ref" a
bash microduck-hd1910/firmware/verification/native-release-prep-20261030/build_public_padd.sh "$public_ref" b
```

The entry point requires the existing pinned Rust1.99.0, cargo-zigbuild0.23.4,
Zig0.15.1, ARM sysroot and cached dependencies described by the installation
contract. It refuses admission below20 GiB free on either E: or F:, disables
incremental compilation, and does not clean caches, relocate them or grow disks.
After successful byte verification, its source copies, target, Zig and temporary
directories are disposable; retain the public archive, binary, JSON and log.

This public-source recipe has been syntax checked and its source tree equivalence
audited. It has **not** been run: WSL was under host maintenance during export.
The existing a/b binary evidence remains the original frozen build evidence.
