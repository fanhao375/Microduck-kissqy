#!/bin/bash
# Prepare one isolated candidate; never replace an earlier input or installed service.
set -euo pipefail
slot=${1:?use a or b for independent build slots}
case "$slot" in a|b) ;; *) exit 2;; esac
art=/mnt/f/chengshenzhilu/Robot/Microduck/artifacts/native-release-prep-20261030
python=/root/padd-remap-buildtools/bin/python
export PATH=/root/padd-remap-buildtools/bin:/root/.cargo/bin:/usr/local/bin:/usr/bin:/bin
"$python" - "$art" <<'PY'
import shutil,sys
# Maintenance admission policy, not a Cargo quota or an automatic cache cleaner.
for path,label in [('/mnt/e','WSL host E:'),(sys.argv[1],'F: build staging')]:
    if shutil.disk_usage(path).free < 20*1024**3:
        raise SystemExit(f'{label} needs at least 20 GiB free')
PY
mapfile -t metadata < <("$python" - "$art" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1]);record=json.loads((root/'native-source.json').read_text())
archive=root/record['archive']
with archive.open('rb') as source:
    assert hashlib.file_digest(source,'sha256').hexdigest()==record['archive_sha256']
assert record['source_commit']=='665613ac628fdc9b1b71f324809f0c16ce2795f2'
print(record['source_commit']);print(record['duck_build_time']);print(record['source_date_epoch'])
PY
)
export DUCK_REVISION=${metadata[0]}
export DUCK_BUILD_TIME=${metadata[1]}
export SOURCE_DATE_EPOCH=${metadata[2]}
work="$art/frozen-$slot"
target="$art/linux-target-$slot"
binary="$art/padd-arm64-$slot"
test ! -e "$work"
test ! -e "$target"
test ! -e "$binary"
mkdir "$work" "$target"
tar -xf "$art/native-reviewed-source.tar" -C "$work"
cd "$work/microduck-hd1910/firmware"
# Build outputs and Zig caches stay on F:, avoiding further expansion of the E: WSL VHD.
export CARGO_TARGET_DIR="$target"
export CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1
export CARGO_INCREMENTAL=0
export ZIG_GLOBAL_CACHE_DIR="$art/zig-global-cache"
export ZIG_LOCAL_CACHE_DIR="$art/zig-local-cache-$slot"
export TMPDIR="$art/tmp-$slot"
mkdir -p "$TMPDIR"
export PKG_CONFIG_ALLOW_CROSS=1
export PKG_CONFIG_SYSROOT_DIR=/root/.cache/duck-sysroot
export PKG_CONFIG_PATH=/root/.cache/duck-sysroot/usr/lib/aarch64-linux-gnu/pkgconfig
cargo +1.99.0 zigbuild --locked --offline --release --jobs 2 \
  --target aarch64-unknown-linux-gnu.2.31 -p padd --bin padd 2>&1 | tee "$art/arm64-build-$slot.log"
cp "$target/aarch64-unknown-linux-gnu/release/padd" "$binary"
chmod 755 "$binary"
file "$binary" | tee "$art/arm64-elf-$slot.txt"
sha256sum "$binary" | tee "$art/arm64-$slot.sha256"
readelf --version-info "$binary" | sed -n 's/.*Name: \(GLIBC_[^ ]*\).*/\1/p' | sort -Vu > "$art/arm64-glibc-$slot.txt"
readelf --wide --file-header --program-headers --dynamic --version-info "$binary" > "$art/arm64-metadata-$slot.txt"
qemu-aarch64 -L /tmp/microduck-updater-spike-root \
  -E LD_LIBRARY_PATH=/root/.cache/duck-sysroot/usr/lib/aarch64-linux-gnu:/root/duck/ir/x/usr/lib/aarch64-linux-gnu \
  "$binary" --version > "$art/qemu-version-$slot.txt"
qemu-aarch64 -L /tmp/microduck-updater-spike-root \
  -E LD_LIBRARY_PATH=/root/.cache/duck-sysroot/usr/lib/aarch64-linux-gnu:/root/duck/ir/x/usr/lib/aarch64-linux-gnu \
  "$binary" --help > "$art/qemu-help-$slot.txt"
launcher="$art/qemu-launcher-$slot"
cat > "$launcher" <<EOF
#!/bin/sh
exec qemu-aarch64 -L /tmp/microduck-updater-spike-root -E LD_LIBRARY_PATH=/root/.cache/duck-sysroot/usr/lib/aarch64-linux-gnu:/root/duck/ir/x/usr/lib/aarch64-linux-gnu "$binary" "\$@"
EOF
chmod 755 "$launcher"
# Unix socket paths are capped at 108 bytes; keep smoke sockets off the long F: path.
TMPDIR=/tmp "$python" padd/tests/process_smoke.py --binary "$launcher" \
  --expect-revision "$DUCK_REVISION" > "$art/qemu-process-smoke-$slot.json"
if [ "$slot" = a ]; then
  # Actual POSIX permission tests need Linux /tmp rather than DrvFS permissions.
  # Host libudev must not inherit the ARM sysroot used above.
  unset PKG_CONFIG_ALLOW_CROSS PKG_CONFIG_SYSROOT_DIR PKG_CONFIG_PATH
  TMPDIR=/tmp cargo +1.99.0 test -p padd --locked --offline --jobs 2 2>&1 | tee "$art/frozen-padd-tests.log"
  TMPDIR=/tmp cargo +1.99.0 build -p padd --bin padd --locked --offline --jobs 2
  TMPDIR=/tmp "$python" padd/tests/process_smoke.py --binary "$target/debug/padd" \
    --expect-revision "$DUCK_REVISION" > "$art/host-process-smoke.json"
fi
