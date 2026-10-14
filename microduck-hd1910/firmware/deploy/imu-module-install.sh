#!/usr/bin/env bash
# Microduck HD1910 · trunk IMU module over I2C — board-side installer.
# Added 2026-10 by fanhao375 (microduck-replica); Apache-2.0 like the rest of this workspace.
#
# On a duck already upgraded to kissqy's R17 V167 (control.43) through his console, swap in the
# robotd / robotctl built from this branch and the four packaging scripts it changes, add
# [trunk_imu] to robotd.toml, and let robotd open /dev/i2c-*. Everything replaced is backed up
# first; a failure after that restores the backup by itself, and `rollback` does it on request.
# Run on the board as root.
#
#   sudo bash imu-module-install.sh check
#   sudo bash imu-module-install.sh install <dir> <bno08x|lsm6dsv16x> <i2c device> <address>
#        <dir> holds robotd, robotctl (aarch64, built from this branch) and calibration-config.py,
#        rebuild-config.py, runtime.py, voice-setup.py (this branch's packaging/).
#        e.g. install ~/imu-module bno08x /dev/i2c-4 0x4B
#   sudo bash imu-module-install.sh rollback <backup dir printed by install>
#
# Every step is logged, with category and level, to $LOG. It never writes the servos' EEPROM and
# never marks an IMU mount as verified: that stays a deliberate step (IMU-MODULE.md).
set -euo pipefail

INSTALL_DIR=${INSTALL_DIR:-/opt/robot/feetech-ft5-r5}
CONFIG_DIR=${CONFIG_DIR:-/etc/robot/feetech-ft5}
BACKUP_ROOT=${BACKUP_ROOT:-/opt/robot}
LOG=${LOG:-/var/log/imu-module-install.log}
NO_SYSTEMD=${NO_SYSTEMD:-0}          # tests only: skip relax, systemctl and the drop-in rewrite
WANT_BUILD=1fa8438-feetech-ft6-control.43
BINS=(robotd robotctl)
SCRIPTS=(calibration-config.py rebuild-config.py runtime.py voice-setup.py)
MANIFESTS=(SHA256SUMS ELF-CHECK.json BUILD.json)
CONFIGS=(robotd.toml robotd.saved.toml hd1910-calibration.toml hd1910-calibration.saved.toml)
DROPIN=/etc/systemd/system/robotd.service.d/zzzz-r17-0151.conf
BACKUP=''          # set once install has made its backup; a later failure restores it
IN_RESTORE=0

log() { local level=$1 cat=$2; shift 2; printf '%s [%s][%s] %s\n' "$(date '+%F %T')" "$level" "$cat" "$*" | tee -a "$LOG" >&2; }

die() {
    log ERROR "$1" "$2"
    if [ -n "$BACKUP" ] && [ "$IN_RESTORE" = 0 ]; then
        IN_RESTORE=1
        log WARN rollback "自动撤回到 $BACKUP"
        if restore_from "$BACKUP" && start_robotd; then
            log WARN rollback '已撤回，板子回到安装前的样子'
        else
            log ERROR rollback "自动撤回没完成，请手动：sudo bash $0 rollback $BACKUP"
        fi
    fi
    echo "失败：$2（日志：$LOG）" >&2
    exit 1
}

need_root() { [ "$(id -u)" = 0 ] || [ "$NO_SYSTEMD" = 1 ] || die precheck '要用 root 运行（sudo bash …）'; }

installed_build() { grep -o "[0-9a-f]\{7\}-feetech-ft6-control\.[0-9]*" "$INSTALL_DIR/release_identity.py" 2>/dev/null | head -1; }

robotd_active() { [ "$NO_SYSTEMD" != 1 ] && systemctl is-active --quiet robotd.service; }

relax_and_stop() {
    if robotd_active; then
        log INFO relax '让 robotd 卸力并确认（请扶住鸭子）'
        python3 "$INSTALL_DIR/runtime.py" relax-confirmed >>"$LOG" 2>&1 \
            || die relax '没能确认卸力，robotd 保持运行，什么都没改'
        systemctl stop robotd.service || die stop 'systemctl stop robotd 失败'
        log INFO stop 'robotd 已停止'
    else
        log INFO stop 'robotd 没在运行'
    fi
}

start_robotd() {   # returns non-zero instead of dying, so a failed restore can still report
    if [ "$NO_SYSTEMD" = 1 ]; then log INFO start '（测试模式）跳过 daemon-reload / start'; return 0; fi
    systemctl daemon-reload || return 1
    systemctl start robotd.service || return 1
    log INFO start 'robotd 已启动'
}

rewrite_dropin() {
    if [ "$NO_SYSTEMD" = 1 ]; then log INFO dropin '（测试模式）跳过启动配置'; return 0; fi
    local mode=motion
    if grep -q -- '--commissioning' "$DROPIN" 2>/dev/null; then mode=commissioning; fi
    python3 "$INSTALL_DIR/runtime.py" configure-startup "$mode" >>"$LOG" 2>&1 || die dropin '生成启动配置失败'
    grep -q 'DeviceAllow=char-i2c rw' "$DROPIN" || die dropin "启动配置里没有 DeviceAllow=char-i2c（$DROPIN）"
    log INFO dropin "启动配置已重写（$mode 模式），允许 /dev/i2c-*"
}

update_manifests() {
    python3 - "$INSTALL_DIR" "${BINS[@]}" -- "${SCRIPTS[@]}" <<'PY'
import hashlib, json, sys
from pathlib import Path
base = Path(sys.argv[1]); split = sys.argv.index('--'); bins = sys.argv[2:split]
names = bins + sys.argv[split + 1:] + ['ELF-CHECK.json', 'BUILD.json']   # SHA256SUMS lists both manifests too
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
for manifest in ('ELF-CHECK.json', 'BUILD.json'):
    path = base / manifest
    data = json.loads(path.read_text())
    for table in (data, data.get('built_binaries', {})):
        for name in bins:
            if isinstance(table.get(name), dict):
                table[name]['sha256'] = digest(base / name)
                table[name]['bytes'] = (base / name).stat().st_size
    if manifest == 'BUILD.json':
        data['microduck_replica_patch'] = {
            'what': 'robotd/robotctl and four packaging scripts from the trunk IMU over I2C branch (fanhao375/Microduck-kissqy)',
            'note': 'not a kissqy release; BUILD string kept so his V167 console accepts it'}
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')
sums = base / 'SHA256SUMS'
rows = []
for line in sums.read_text().splitlines():
    sha, name = line.split('  ', 1)
    rows.append(f'{digest(base / name)}  {name}' if name in names else line)
sums.write_text('\n'.join(rows) + '\n')
print('manifests updated')
PY
}

set_trunk_imu() {   # file source device address(decimal)
    python3 - "$@" <<'PY'
import re, sys
from pathlib import Path
path, source, device, address = Path(sys.argv[1]), sys.argv[2], sys.argv[3], int(sys.argv[4])
text = path.read_text()
block = f'["trunk_imu"]\n"source" = "{source}"\n"i2c_device" = "{device}"\n"i2c_address" = {address}\n'
header = re.compile(r'^\[\s*"?trunk_imu"?\s*\]\s*$', re.M)
m = header.search(text)
if m:
    nxt = re.compile(r'^\[', re.M).search(text, m.end())
    text = text[:m.start()] + block + ('\n' + text[nxt.start():] if nxt else '')
else:
    text = text.rstrip('\n') + '\n\n' + block
path.write_text(text)
print(f'{path.name}: [trunk_imu] source={source} {device} {address:#04x}')
PY
}

mount_state() {   # prints the mount status of one source from the calibration file
    python3 - "$CONFIG_DIR/hd1910-calibration.toml" "$1" <<'PY'
import re, sys
text = open(sys.argv[1], encoding='utf-8').read()
try:
    import tomllib
    mount = (tomllib.loads(text).get('imu_mounts') or {}).get(sys.argv[2])
except ImportError:   # Python older than 3.11: read just this one table
    pattern = r'^\[\s*"?imu_mounts"?\s*\.\s*"?' + re.escape(sys.argv[2]) + r'"?\s*\]\s*$(.*?)(?=^\[|\Z)'
    found = re.search(pattern, text, re.M | re.S)
    mount = None
    if found:
        body = found.group(1)
        quat = re.search(r'quat\s*=\s*\[([^\]]*)\]', body)
        mount = {'verified': re.search(r'verified\s*=\s*true', body) is not None,
                 'quat': [float(v) for v in quat.group(1).split(',')] if quat else []}
if not mount:
    print('missing')
elif mount.get('verified') is True:
    print('verified ' + ' '.join(f'{v:.9f}' for v in mount.get('quat', [])))
else:
    print('unverified')
PY
}

restore_from() {   # backup dir; robotd must already be stopped
    local backup=$1 f
    for f in "$backup"/opt/*; do cp -a "$f" "$INSTALL_DIR/" || return 1; done
    for f in "${CONFIGS[@]}"; do
        if [ -f "$backup/etc/$f" ]; then cp -a "$backup/etc/$f" "$CONFIG_DIR/$f" || return 1; fi
    done
    if [ -f "$backup/etc/dropin.conf" ] && [ "$NO_SYSTEMD" != 1 ]; then cp -a "$backup/etc/dropin.conf" "$DROPIN" || return 1; fi
    ( cd "$INSTALL_DIR" && sha256sum -c --quiet SHA256SUMS ) >>"$LOG" 2>&1
}

cmd_check() {
    echo "安装目录：$INSTALL_DIR"
    echo "已装版本：$(installed_build || true)（这个脚本要求 $WANT_BUILD）"
    if [ -f "$INSTALL_DIR/robotd" ]; then
        if grep -a -q 'lsm6dsv16x' "$INSTALL_DIR/robotd"; then kind='带 IMU 模块支持'; else kind='kissqy 原版（不认 IMU 模块）'; fi
        echo "robotd：$(sha256sum "$INSTALL_DIR/robotd" | cut -c1-16)…  $kind"
    fi
    echo "I2C 设备：$(ls /dev/i2c-* 2>/dev/null | tr '\n' ' ')"
    grep -A3 -E '^\[\s*"?trunk_imu"?\s*\]' "$CONFIG_DIR/robotd.toml" 2>/dev/null || echo 'robotd.toml 里没有 [trunk_imu]（默认读总线 IMU 小板）'
    for s in bno08x lsm6dsv16x; do echo "安装方向 $s：$(mount_state "$s" 2>/dev/null || echo '读不了标定文件')"; done
    if grep -q 'DeviceAllow=char-i2c rw' "$DROPIN" 2>/dev/null; then echo '启动配置：允许 /dev/i2c-*'; else echo '启动配置：不允许 /dev/i2c-*'; fi
}

cmd_install() {
    [ $# -eq 4 ] || die usage '用法：install <文件目录> <bno08x|lsm6dsv16x> <i2c 设备> <地址>'
    local src=$1 source=$2 device=$3 address=$(( $4 )) f
    need_root
    log INFO precheck "开始安装：$src $source $device $(printf '%#04x' "$address")"
    [ "$(installed_build)" = "$WANT_BUILD" ] \
        || die precheck "板上是 $(installed_build || echo 未知版本)，不是 $WANT_BUILD：先用 kissqy 中控 1.0.167 升级到 V167"
    case $source in
        bno08x) [[ $address == 74 || $address == 75 ]] || die precheck 'BNO08x 地址只能是 0x4A 或 0x4B' ;;
        lsm6dsv16x) [[ $address == 106 || $address == 107 ]] || die precheck 'LSM6DSV16X 地址只能是 0x6A 或 0x6B' ;;
        *) die precheck "source 只能是 bno08x 或 lsm6dsv16x，不是 $source" ;;
    esac
    [ -c "$device" ] || [ "$NO_SYSTEMD" = 1 ] || die precheck "$device 不存在：先开对应的 I2C overlay（见 IMU-MODULE.md）"
    for f in "${BINS[@]}" "${SCRIPTS[@]}"; do [ -f "$src/$f" ] || die precheck "缺文件 $src/$f"; done
    python3 - "$src/robotd" "$src/robotctl" <<'PY' || die precheck 'robotd / robotctl 不是 aarch64 程序'
import sys
for p in sys.argv[1:]:
    head = open(p, 'rb').read(20)
    assert head[:4] == b'\x7fELF' and int.from_bytes(head[18:20], 'little') == 183, p
PY
    grep -a -q "$WANT_BUILD" "$src/robotd" || die precheck "robotd 不是 $WANT_BUILD 编出来的"
    grep -a -q 'lsm6dsv16x' "$src/robotd" || die precheck 'robotd 里没有 IMU 模块支持，是不是拿成了 kissqy 原版？'
    for f in "${MANIFESTS[@]}" "${BINS[@]}" "${SCRIPTS[@]}"; do [ -f "$INSTALL_DIR/$f" ] || die precheck "板上缺 $INSTALL_DIR/$f"; done
    log INFO precheck '检查通过'

    relax_and_stop

    local backup
    backup=$BACKUP_ROOT/imu-module-backup-$(date +%Y%m%d-%H%M%S)
    mkdir -p "$backup/opt" "$backup/etc"
    for f in "${BINS[@]}" "${SCRIPTS[@]}" "${MANIFESTS[@]}"; do cp -a "$INSTALL_DIR/$f" "$backup/opt/"; done
    for f in "${CONFIGS[@]}"; do if [ -f "$CONFIG_DIR/$f" ]; then cp -a "$CONFIG_DIR/$f" "$backup/etc/"; fi; done
    if [ -f "$DROPIN" ]; then cp -a "$DROPIN" "$backup/etc/dropin.conf"; fi
    BACKUP=$backup
    log INFO backup "已备份到 $backup"

    for f in "${BINS[@]}"; do install -o root -g root -m 755 "$src/$f" "$INSTALL_DIR/$f" || die files "换 $f 失败"; done
    for f in "${SCRIPTS[@]}"; do install -o root -g root -m 644 "$src/$f" "$INSTALL_DIR/$f" || die files "换 $f 失败"; done
    log INFO files '已换 robotd、robotctl 和 4 个安装脚本'
    update_manifests >>"$LOG" 2>&1 || die files '更新清单失败'
    ( cd "$INSTALL_DIR" && sha256sum -c --quiet SHA256SUMS ) >>"$LOG" 2>&1 || die files 'SHA256SUMS 核对不过'
    log INFO files 'SHA256SUMS、ELF-CHECK.json、BUILD.json 已更新并核对'

    for f in robotd.toml robotd.saved.toml; do
        if [ -f "$CONFIG_DIR/$f" ]; then
            set_trunk_imu "$CONFIG_DIR/$f" "$source" "$device" "$address" >>"$LOG" 2>&1 || die config "改 $f 失败"
        fi
    done
    log INFO config "robotd.toml / robotd.saved.toml：[trunk_imu] = $source $device $(printf '%#04x' "$address")"

    rewrite_dropin
    start_robotd || die start 'robotd 起不来，看 journalctl -u robotd'

    local mount
    mount=$(mount_state "$source")
    case $mount in
        verified*) log INFO mount "$source 的安装方向已确认：${mount#verified }" ;;
        *) log WARN mount "$source 的安装方向还没确认（$mount）：运动模式会拒绝上力，按 IMU-MODULE.md 先确认方向" ;;
    esac
    echo "完成。备份：$backup"
    echo "撤回：sudo bash $0 rollback $backup"
}

cmd_rollback() {
    [ $# -eq 1 ] && [ -d "$1/opt" ] || die usage '用法：rollback <install 打印的备份目录>'
    need_root
    local backup=$1
    log INFO rollback "从 $backup 恢复"
    relax_and_stop
    restore_from "$backup" || die rollback "恢复失败，备份还在 $backup"
    start_robotd || die rollback 'robotd 起不来，看 journalctl -u robotd'
    log INFO rollback '已恢复'
}

case ${1:-} in
    check) shift; cmd_check "$@" ;;
    install) shift; cmd_install "$@" ;;
    rollback) shift; cmd_rollback "$@" ;;
    *) sed -n '2,20p' "$0"; exit 2 ;;
esac
