#!/bin/bash
# Reproduce the frozen padd bytes from an explicitly fetched public export commit.
set -euo pipefail
ref=${1:?supply the full public export commit SHA}
slot=${2:?use a or b for independent reproduction slots}
[[ "$ref" =~ ^[0-9a-f]{40}$ ]]
case "$slot" in a|b) ;; *) exit 2;; esac
git cat-file -e "$ref^{commit}"
art=/mnt/f/chengshenzhilu/Robot/Microduck/artifacts/native-public-repro-20261031
python=/root/padd-remap-buildtools/bin/python
export PATH=/root/padd-remap-buildtools/bin:/root/.cargo/bin:/usr/local/bin:/usr/bin:/bin
"$python" - <<'PY'
import shutil
for path,label in [('/mnt/e','WSL host E:'),('/mnt/f','F: reproduction staging')]:
    if shutil.disk_usage(path).free < 20*1024**3:
        raise SystemExit(f'{label} needs at least 20 GiB free')
PY
work="$art/public-$slot"
target="$art/target-$slot"
archive="$art/source-$slot.tar"
binary="$art/padd-public-$slot"
record="$art/public-reproduction-$slot.json"
for path in "$work" "$target" "$archive" "$binary" "$record"; do test ! -e "$path"; done
mkdir -p "$art"
mkdir "$work" "$target"
git archive --format=tar "$ref" microduck-hd1910/firmware > "$archive"
tar -xf "$archive" -C "$work"
export DUCK_REVISION=665613ac628fdc9b1b71f324809f0c16ce2795f2
export DUCK_BUILD_TIME=2026-10-30T23:17:10+09:00
export SOURCE_DATE_EPOCH=1793369830
export CARGO_TARGET_DIR="$target"
export CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1
export CARGO_INCREMENTAL=0
export ZIG_GLOBAL_CACHE_DIR="$art/zig-global-cache"
export ZIG_LOCAL_CACHE_DIR="$art/zig-local-cache-$slot"
export TMPDIR="$art/tmp-$slot"
mkdir "$TMPDIR"
export PKG_CONFIG_ALLOW_CROSS=1
export PKG_CONFIG_SYSROOT_DIR=/root/.cache/duck-sysroot
export PKG_CONFIG_PATH=/root/.cache/duck-sysroot/usr/lib/aarch64-linux-gnu/pkgconfig
cd "$work/microduck-hd1910/firmware"
cargo +1.99.0 zigbuild --locked --offline --release --jobs 2 \
  --target aarch64-unknown-linux-gnu.2.31 -p padd --bin padd 2>&1 | tee "$art/build-$slot.log"
cp "$target/aarch64-unknown-linux-gnu/release/padd" "$binary"
"$python" - "$ref" "$archive" "$binary" "$record" <<'PY'
import hashlib,json,sys
from pathlib import Path
ref,archive,binary,output=sys.argv[1:]
archive=Path(archive);binary=Path(binary)
digest=hashlib.file_digest(binary.open('rb'),'sha256').hexdigest()
if digest!='dc1c7043a108964583fc09a11e3d6f87a25e259c346f7ce9154e77500a0881ec':
    raise SystemExit('public source did not reproduce frozen bytes')
if binary.stat().st_size!=3522648:
    raise SystemExit('public source did not reproduce frozen size')
record={'schema':'microduck-controller-public-reproduction/1','public_source_commit':ref,
        'public_archive_sha256':hashlib.file_digest(archive.open('rb'),'sha256').hexdigest(),
        'original_embedded_revision':'665613ac628fdc9b1b71f324809f0c16ce2795f2',
        'original_embedded_build_time':'2026-10-30T23:17:10+09:00',
        'original_revision_publication_claimed':False,'sha256':digest,'size':binary.stat().st_size}
with Path(output).open('x',encoding='utf-8') as stream:
    json.dump(record,stream,indent=2);stream.write('\n')
print(json.dumps(record))
PY
