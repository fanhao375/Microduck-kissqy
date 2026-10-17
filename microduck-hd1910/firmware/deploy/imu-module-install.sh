#!/usr/bin/env bash
# Microduck HD1910 · microduck-replica board-side installer: the whole replica set on kissqy V167 —
# trunk IMU module over I2C + optional servo P/D/torque-limit file (tuning) + UDP pad / pad-loss sit.
# Added 2026-10 by fanhao375 (microduck-replica); BNO08x gyro-integrated attitude; Apache-2.0 like the rest of this workspace.
#
# On a duck already upgraded to kissqy's R17 V167 (control.43) through his console, swap in the
# robotd / robotctl / padd built from this branch and the five packaging scripts it changes, add
# [trunk_imu] to robotd.toml (its wiring rewritten; bno08x_fusion / bno08x_raw_log, if the operator
# set them, kept), let robotd open /dev/i2c-* and padd bind UDP (AF_INET drop-in; the
# UDP pad itself stays off until `service.sh netpad on`). Everything replaced is backed up first; a
# failure after that restores the backup by itself, and `rollback` does it on request.
# Run on the board as root, with servo power on (robotd must finish its four startup steps).
#
#   sudo bash imu-module-install.sh check
#   sudo bash imu-module-install.sh install <dir> <bno08x|lsm6dsv16x> <i2c device> <address>
#        <dir> holds robotd, robotctl, padd (aarch64, built from this branch) and
#        calibration-config.py, rebuild-config.py, runtime.py, voice-setup.py, service.sh
#        (this branch's packaging/).  e.g. install ~/replica bno08x /dev/i2c-4 0x4B
#   sudo bash imu-module-install.sh rollback [--old-backup] <backup dir printed by install>
#        Back to the state before THAT install (programs, scripts, manifests, robotd.toml, drop-ins);
#        the calibration files are kept as they are now. Use the backup this install printed: an
#        older one may bring back kissqy's original robotd, which cannot read [imu_mounts] (refused
#        while the calibration has them). --old-backup accepts a backup made by the first version
#        of this script (no record of what failed its checksums before).
#
# Every step is logged, with category and level, to $LOG. It never writes the servos' EEPROM, never
# marks an IMU mount as verified, never writes [netpad] or a servo parameter file, never enables a
# service and never runs gamepad.py prepare-all.
set -Eeuo pipefail
export LC_COLLATE=C   # sort and comm must agree on the order of the checksum names

INSTALL_DIR=${INSTALL_DIR:-/opt/robot/feetech-ft5-r5}
CONFIG_DIR=${CONFIG_DIR:-/etc/robot/feetech-ft5}
BACKUP_ROOT=${BACKUP_ROOT:-/opt/robot}
LOG=${LOG:-/var/log/imu-module-install.log}
SYSTEMD_DIR=${SYSTEMD_DIR:-/etc/systemd/system}
ROBOTD_SOCKET=${ROBOTD_SOCKET:-/run/robotd.sock}
LOCK_FILE=${LOCK_FILE:-/run/microduck-service-management.lock}   # the one kissqy's service.sh takes
READY_TIMEOUT=${READY_TIMEOUT:-60}
NO_SYSTEMD=${NO_SYSTEMD:-0}          # tests only: skip relax, systemctl, ldd / padd --help, restarts
SKIP_ELF_RUN=${SKIP_ELF_RUN:-0}      # tests only: an x86 host cannot run ldd / --help on aarch64
WANT_BUILD=1fa8438-feetech-ft6-control.43
V167_GAIN_MODE=hd1910_model_contract_p_restore_on_relax_native_d_torque_limit_preserved
FILE_GAIN_MODE=hd1910_servo_parameters_file_relax_p_restored_d0_limit1000
BINS=(robotd robotctl padd)
SCRIPTS=(calibration-config.py rebuild-config.py runtime.py voice-setup.py service.sh)
MANIFESTS=(SHA256SUMS ELF-CHECK.json BUILD.json)
CONFIGS=(robotd.toml robotd.saved.toml hd1910-calibration.toml hd1910-calibration.saved.toml servo-test-parameters.json)
RESTORED_CONFIGS=(robotd.toml robotd.saved.toml servo-test-parameters.json)   # not the calibration
CALIBRATIONS=(hd1910-calibration.toml hd1910-calibration.saved.toml)
DROPIN=$SYSTEMD_DIR/robotd.service.d/zzzz-r17-0151.conf
PADD_DROPIN=$SYSTEMD_DIR/padd.service.d/zzzz-replica-netpad.conf
BACKUP=''          # set once install has made its backup; a later failure restores it
STOPPED=0          # set once padd (then robotd) was stopped; a failure before the backup starts them again
IN_DIE=0
SOURCE_KIND=''     # bno08x / lsm6dsv16x, for the post-start check
PARTOF_BEFORE=''   # units that are PartOf robotd, with their state before
ROLLBACK_FROM=''   # the backup a manual rollback is restoring; an interrupted one says to run it again

# The log file first; the terminal only if it is still there. After an ssh session drops (HUP is
# ignored, so the run goes on), writing to the dead terminal fails with EIO: that must never reach the
# ERR trap, or the automatic rollback would stop at its own first log line.
log() {
    local level=$1 cat=$2 line; shift 2
    line=$(printf '%s [%s][%s] %s' "$(date '+%F %T')" "$level" "$cat" "$*")
    printf '%s\n' "$line" >>"$LOG"
    # >&2 first, then 2>/dev/null: the line reaches the terminal and only a failed write's own
    # complaint is dropped (in a { ...; } 2>/dev/null group the line itself went to /dev/null).
    printf '%s\n' "$line" >&2 2>/dev/null || true
}
say() { { printf '%s\n' "$*"; } 2>/dev/null || true; }   # stdout, same rule

die() {
    # In a $(...) subshell only leave it: the caller's assignment fails and the main shell dies once.
    if [ "$BASHPID" != "$$" ]; then exit 1; fi
    trap - ERR
    trap '' INT TERM   # a second interrupt must not cut the way back short
    log ERROR "$1" "$2"
    if [ "$IN_DIE" = 0 ]; then
        IN_DIE=1
        if [ -n "$BACKUP" ]; then
            log WARN rollback "自动撤回到 $BACKUP"
            local rc=0
            # A robotd / padd this run may have started (limp: no HOME has run) must not keep the new
            # programs loaded while the old files go back.
            if [ "$NO_SYSTEMD" != 1 ]; then systemctl stop padd.service robotd.service >/dev/null 2>&1 || true; fi
            restore_from "$BACKUP" auto || rc=$?
            if [ "$rc" = 1 ]; then
                log ERROR rollback "备份没能复制回去，robotd 没有启动；请手动：sudo bash $0 rollback $BACKUP"
            else
                [ "$rc" = 0 ] || log WARN rollback '文件已复制回去，但核对有差异（见上一条）；照样启动服务'
                if start_services rollback; then
                    log WARN rollback '已撤回，板子回到安装前的样子'
                else
                    log ERROR rollback "文件已恢复，但服务没起来；看 journalctl -u robotd -u padd，或再跑：sudo bash $0 rollback $BACKUP"
                fi
                partof_report
            fi
        elif [ "$STOPPED" = 1 ]; then
            # Nothing was replaced yet: the programs on the board are the old ones; bring them back.
            log WARN start '还没换任何文件；把停掉的服务按原样启动'
            if start_services rollback; then log WARN start '原来的 robotd / padd 已重新启动'
            else log ERROR start '原来的 robotd / padd 没起来；看 journalctl -u robotd -u padd'; fi
            partof_report
        fi
        if [ -n "$ROLLBACK_FROM" ]; then
            log ERROR rollback "撤回没有做完：请重跑 sudo bash $0 rollback $ROLLBACK_FROM"
        fi
    fi
    echo "失败：$2（日志：$LOG）" >&2 2>/dev/null || true   # same order as in log()
    exit 1
}
trap 'die unexpected "第 $LINENO 行意外退出（$BASH_COMMAND）"' ERR
trap 'die signal "收到中断"' INT TERM
trap '' HUP   # an ssh session that drops must not leave the board half installed

need_root() { [ "$(id -u)" = 0 ] || [ "$NO_SYSTEMD" = 1 ] || die precheck '要用 root 运行（sudo bash …）'; }

# The lock kissqy's service.sh holds for any management operation: never two at once.
take_lock() {
    exec 9>"$LOCK_FILE"
    flock -n 9 || die lock "已有服务管理操作在进行（$LOCK_FILE 被占用）：等它结束再来"
}

installed_build() { grep -o "[0-9a-f]\{7\}-feetech-ft6-control\.[0-9]*" "$INSTALL_DIR/release_identity.py" 2>/dev/null | head -1; }

# robotd's JSON-RPC socket, without the board's runtime.py (the socket path is a variable here).
#   state          -> control | <mode>/<phase> | not-running: <why> | unknown: <why>
#                     (runtime.py relax_and_restore's rule: only a missing or refused socket means no
#                     robotd; a slow answer is asked again for 8 s)
#   relax-confirm  -> robot.relax, then wait for torque OFF, restore_pending False and
#                     temporary_gains_applied False together (V167 restores registers after OFF)
#   relax-request  -> one robot.relax, never waits
#   verify <source> -> after start, asked again for 8 s: build, and in motion the IMU source, the
#                     servo_parameters key, gain_mode
robotd_rpc() {
    python3 - "$ROBOTD_SOCKET" "$WANT_BUILD" "$V167_GAIN_MODE" "$FILE_GAIN_MODE" "$@" <<'PY'
import json, socket, sys, time
path, build, v167, file_mode, action, *args = sys.argv[1:]

def rpc(method, timeout=2):
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(timeout); s.connect(path)
        s.sendall((json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': {}}) + '\n').encode())
        with s.makefile('rb') as f: response = json.loads(f.readline(1048576))
    if response.get('error'): raise RuntimeError(str(response['error']))
    return response['result']

def settled(report):
    native = report.get('native') or {}
    hd = native.get('hd1910') or {}
    return (not report.get('torque_off_pending') and report.get('mode') == 'motion'
            and native.get('torque_state_confirmed') is False
            and hd.get('restore_pending') is False and hd.get('temporary_gains_applied') is False)

def check(report, source):
    problems = []
    if report.get('build') != build: problems.append(f"build {report.get('build')} 不是 {build}")
    native = report.get('native') or {}
    hd = native.get('hd1910') or {}
    if report.get('mode') == 'motion':
        if native.get('imu_source') != source: problems.append(f"imu_source {native.get('imu_source')} 不是 {source}")
        if 'servo_parameters' not in hd: problems.append('native.hd1910 没有 servo_parameters（不是可调版 robotd）')
        if native.get('gain_mode') not in (v167, file_mode): problems.append(f"gain_mode {native.get('gain_mode')}")
    return problems

if action == 'state':
    until = time.monotonic() + 8; last = ''
    while time.monotonic() < until:
        try: report = rpc('robot.busStatus')
        except (FileNotFoundError, ConnectionRefusedError) as exc: print(f'not-running: {exc}'[:200]); sys.exit(0)
        except Exception as exc: last = str(exc)[:200] or type(exc).__name__; time.sleep(.1); continue
        mode, phase = report.get('mode'), report.get('phase')
        print('control' if (mode, phase) == ('motion', 'control') else f'{mode}/{phase}'); sys.exit(0)
    print(f'unknown: 8 秒内没回答（{last}）')
elif action == 'relax-request':
    try: rpc('robot.relax'); print('relax requested')
    except Exception as exc: print(f'relax request not answered: {exc}'[:200])
elif action == 'relax-confirm':
    rpc('robot.relax')
    until = time.monotonic() + 8; last = ''
    while time.monotonic() < until:
        try: report = rpc('robot.busStatus')
        except Exception as exc: last = str(exc)[:200]; time.sleep(.1); continue   # one slow reply
        if settled(report): print('torque OFF, restore finished'); sys.exit(0)
        hd = (report.get('native') or {}).get('hd1910') or {}
        last = json.dumps({'mode': report.get('mode'), 'phase': report.get('phase'),
                           'torque_state_confirmed': (report.get('native') or {}).get('torque_state_confirmed'),
                           'restore_pending': hd.get('restore_pending'),
                           'temporary_gains_applied': hd.get('temporary_gains_applied')})
        time.sleep(.1)
    print('not settled: ' + last); sys.exit(1)
elif action == 'verify':
    source = args[0]
    until = time.monotonic() + 8; last = ''
    while True:
        try:
            report = rpc('robot.busStatus', timeout=5)
            problems = check(report, source)
            if not problems:
                native = report.get('native') or {}; hd = native.get('hd1910') or {}
                print(json.dumps({'mode': report.get('mode'), 'phase': report.get('phase'), 'build': report.get('build'),
                                  'imu_source': native.get('imu_source'), 'gain_mode': native.get('gain_mode'),
                                  'servo_parameters': hd.get('servo_parameters', 'absent')}, ensure_ascii=False))
                sys.exit(0)
            last = '; '.join(problems)
        except Exception as exc:
            last = f'busStatus 没回答：{exc}'[:200]
        if time.monotonic() >= until: print(last); sys.exit(1)
        time.sleep(.2)
PY
}

# Units that are PartOf robotd (a recorder, say) stop with it; they come back with robotd only if
# robotd also wants them (WantedBy=robotd.service and enabled). Report them before and after,
# name one that stayed down, never start them.
partof_record() {
    [ "$NO_SYSTEMD" = 1 ] && return 0
    local unit units
    units=$(systemctl show robotd.service -p ConsistsOf --value 2>/dev/null || true)
    PARTOF_BEFORE=''
    for unit in $units; do PARTOF_BEFORE+="$unit=$(systemctl is-active "$unit" 2>/dev/null || true) "; done
    [ -z "$PARTOF_BEFORE" ] || log INFO partof "随 robotd 停止的单元（停之前）：$PARTOF_BEFORE"
    return 0
}
partof_report() {
    [ "$NO_SYSTEMD" = 1 ] && return 0
    local entry unit was now
    for entry in $PARTOF_BEFORE; do
        unit=${entry%%=*}; was=${entry#*=}
        now=$(systemctl is-active "$unit" 2>/dev/null || true)
        if [ "$was" = active ] && [ "$now" != active ]; then
            log WARN partof "$unit 之前在运行，跟 robotd 一起停了、没有跟着起来；需要的话：sudo systemctl start $unit"
        else
            log INFO partof "$unit：之前 $was，现在 $now"
        fi
    done
    return 0
}

# padd first (its program is replaced, and a pad press must not arm the robot between the relax
# and the stop), then the torque: confirmed OFF with the HD1910 registers restored when robotd is
# driving; otherwise (waiting for the bus, commissioning, read-only, crash-looping) one relax
# request and a warning; no relax when nothing listens on the socket. Then robotd, whatever
# is-active says: a robotd in a restart loop is "activating".
relax_and_stop() {
    if [ "$NO_SYSTEMD" = 1 ]; then log INFO relax '（测试模式）跳过卸力和停服务'; return 0; fi
    local state before
    before=$(systemctl is-active robotd.service 2>/dev/null || true)
    partof_record
    STOPPED=1   # before the stop: an interrupted stop still brings padd back
    systemctl stop padd.service || [ "$(systemctl show padd.service -p LoadState --value 2>/dev/null || true)" = not-found ] \
        || die stop 'systemctl stop padd 失败；什么都没改'
    log INFO stop 'padd 已停止（它的程序要换，也不让手柄在卸力和停 robotd 之间再上力）'
    state=$(robotd_rpc state)
    case $state in
        control)
            log INFO relax '运动模式控制中：请求卸力，等扭矩 OFF 且舵机寄存器恢复完成（请扶住鸭子）'
            robotd_rpc relax-confirm >>"$LOG" 2>&1 \
                || die relax '没能确认卸力并恢复，robotd 保持运行，什么都没改。若是某个舵机不应答：先断舵机电，再 sudo systemctl stop robotd，然后重跑安装'
            ;;
        not-running:*)
            log INFO relax "robotd 没在运行（$state；systemd：${before:-unknown}），不用卸力"
            ;;
        *)
            log WARN relax "robotd 是 $state（systemd：${before:-unknown}）：扭矩状态确认不了，只发一次卸力请求；请扶住或放倒鸭子"
            robotd_rpc relax-request >>"$LOG" 2>&1 || true
            ;;
    esac
    systemctl stop robotd.service || die stop 'systemctl stop robotd 失败'
    log INFO stop "robotd 已停止（之前：${before:-unknown}）"
}

# padd under kissqy's startup_gate: it waits for robotd's four steps, then becomes padd in the same
# process. Judged by what that new process writes.
restart_padd() {
    local pid out
    systemctl reset-failed padd.service 2>/dev/null || true
    systemctl restart padd.service || return 1
    for _ in $(seq 1 40); do
        sleep 0.25
        pid=$(systemctl show padd.service -p MainPID --value 2>/dev/null || true)
        [ -n "$pid" ] && [ "$pid" != 0 ] || continue
        out=$(journalctl -u padd.service _PID="$pid" -o cat --no-pager 2>/dev/null || true)
        case $out in *'driving —'*) log INFO padd "padd 新进程 $pid 已开始 driving"; return 0 ;; esac
    done
    log ERROR padd 'padd 10 秒内没有开始 driving'
    journalctl -u padd.service -n 20 --no-pager -o cat >>"$LOG" 2>&1 || true
    return 1
}

# daemon-reload → robotd → its four startup steps → (install: busStatus check) → padd → the GameSir
# bridge only where it is enabled. Returns non-zero instead of dying, so a rollback can report.
start_services() {
    local why=$1
    if [ "$NO_SYSTEMD" = 1 ]; then log INFO start '（测试模式）跳过 daemon-reload / 启动 / 重启 padd'; return 0; fi
    systemctl daemon-reload || return 1
    systemctl reset-failed robotd.service 2>/dev/null || true
    systemctl start robotd.service || return 1
    log INFO start "robotd 已启动，等四步启动完成（最多 $READY_TIMEOUT 秒）"
    if ! python3 "$INSTALL_DIR/startup_gate.py" --socket "$ROBOTD_SOCKET" --timeout "$READY_TIMEOUT" >>"$LOG" 2>&1; then
        log ERROR start 'robotd 四步启动没完成（舵机电源开着吗？）；journalctl -u robotd 见日志'
        journalctl -u robotd.service -n 30 --no-pager -o cat >>"$LOG" 2>&1 || true
        return 1
    fi
    log INFO start 'robotd 四步启动完成'
    if [ "$why" = install ]; then
        local seen
        if ! seen=$(robotd_rpc verify "$SOURCE_KIND" 2>&1); then
            log ERROR verify "启动后的 busStatus 8 秒内一直不对：$(echo "$seen" | tail -1)"
            return 1
        fi
        log INFO verify "busStatus：$(echo "$seen" | head -1)"
    fi
    restart_padd || return 1
    if [ "$(systemctl is-enabled gamesir-xboxd.service 2>/dev/null || true)" = enabled ]; then
        systemctl start gamesir-xboxd.service || log WARN input 'gamesir-xboxd 没能启动'
    fi
    return 0
}

input_report() {   # report, never start something nobody started
    [ "$NO_SYSTEMD" = 1 ] && return 0
    local unit
    for unit in padd gamesir-xboxd; do
        log INFO input "$unit：$(systemctl is-active "$unit.service" 2>/dev/null || true) / $(systemctl is-enabled "$unit.service" 2>/dev/null || true)"
    done
    log INFO input "padd 协议族：$(systemctl show padd.service -p RestrictAddressFamilies --value 2>/dev/null || true)"
}

rewrite_dropin() {
    # runtime.py configure-startup writes the real /etc/systemd path: tests (a fake SYSTEMD_DIR) skip it.
    if [ "$NO_SYSTEMD" = 1 ] || [ "$SYSTEMD_DIR" != /etc/systemd/system ]; then log INFO dropin '（测试模式）跳过启动配置'; return 0; fi
    local mode=motion
    if grep -q -- '--commissioning' "$DROPIN" 2>/dev/null; then mode=commissioning; fi
    python3 "$INSTALL_DIR/runtime.py" configure-startup "$mode" >>"$LOG" 2>&1 || die dropin '生成启动配置失败'
    grep -q 'DeviceAllow=char-i2c rw' "$DROPIN" || die dropin "启动配置里没有 DeviceAllow=char-i2c（$DROPIN）"
    log INFO dropin "启动配置已重写（$mode 模式），允许 /dev/i2c-*"
}

# Drop-ins add to RestrictAddressFamilies, so AF_INET alone joins kissqy's AF_UNIX AF_NETLINK.
# kissqy's own padd.service and zzzz-r17-0151.conf are left alone.
padd_dropin_managed() { [ "$NO_SYSTEMD" != 1 ] || [ "$SYSTEMD_DIR" != /etc/systemd/system ]; }   # tests: only a fake dir

write_padd_dropin() {
    if ! padd_dropin_managed; then log INFO dropin '（测试模式）跳过 padd 的 AF_INET drop-in'; return 0; fi
    mkdir -p "$(dirname "$PADD_DROPIN")" || die dropin "建不了 $(dirname "$PADD_DROPIN")"
    printf '%s\n' '# microduck-replica (imu-module-install.sh): the UDP pad ([netpad]) needs one IPv4 socket.' \
        '[Service]' 'RestrictAddressFamilies=AF_INET' > "$PADD_DROPIN" || die dropin "写不了 $PADD_DROPIN"
    log INFO dropin "padd 允许 AF_INET：$PADD_DROPIN"
}

# NEEDED and the version needs of an ELF64 little-endian file, from its own tables (no readelf on
# the board): prints JSON {"needed": [...], "versions": {"libc.so.6": ["GLIBC_2.17", ...]}}.
elf_info() {
    python3 - "$1" <<'PY'
import json, struct, sys
data = open(sys.argv[1], 'rb').read()
shoff, = struct.unpack_from('<Q', data, 0x28)
shentsize, shnum, _ = struct.unpack_from('<HHH', data, 0x3A)
sections = [struct.unpack_from('<IIQQQQIIQQ', data, shoff + i * shentsize) for i in range(shnum)]
def string(table, offset):
    start = sections[table][4] + offset
    return data[start:data.index(b'\0', start)].decode()
needed, versions = [], {}
for s in sections:
    if s[1] == 6:   # SHT_DYNAMIC
        for i in range(0, s[5], 16):
            tag, value = struct.unpack_from('<qQ', data, s[4] + i)
            if tag == 0: break
            if tag == 1: needed.append(string(s[6], value))
    if s[1] == 0x6ffffffe:   # SHT_GNU_verneed
        offset = s[4]
        while True:
            _, count, name, aux, nxt = struct.unpack_from('<HHIII', data, offset)
            lib, a = string(s[6], name), offset + aux
            for _ in range(count):
                _, _, _, vname, vnext = struct.unpack_from('<IHHII', data, a)
                versions.setdefault(lib, []).append(string(s[6], vname)); a += vnext
            if not nxt: break
            offset += nxt
print(json.dumps({'needed': needed, 'versions': versions}))
PY
}

check_padd_elf() {   # src padd: NEEDED as a set = ELF-CHECK's padd, GLIBC at most 2.30
    python3 - "$INSTALL_DIR/ELF-CHECK.json" "$(elf_info "$1")" <<'PY'
import json, sys
check = json.load(open(sys.argv[1]))['padd']; info = json.loads(sys.argv[2])
if set(info['needed']) != set(check['needed']):
    sys.exit(f"padd NEEDED {sorted(info['needed'])} 和 ELF-CHECK.json {sorted(check['needed'])} 不同")
glibc = sorted({tuple(map(int, v[6:].split('.'))) for vs in info['versions'].values() for v in vs
                if v.startswith('GLIBC_2')})
if glibc and glibc[-1] > (2, 30): sys.exit('padd 需要 GLIBC ' + '.'.join(map(str, glibc[-1])) + '，板上只有 2.30 以内')
udev = sorted({v for lib, vs in info['versions'].items() if lib.startswith('libudev') for v in vs})
print(f"padd NEEDED {sorted(info['needed'])}；GLIBC 最高 {'.'.join(map(str, glibc[-1])) if glibc else '-'}；libudev 版本 {udev or '-'}")
PY
}

update_manifests() {
    python3 - "$INSTALL_DIR" "$(elf_info "$INSTALL_DIR/padd")" "${BINS[@]}" -- "${SCRIPTS[@]}" <<'PY'
import hashlib, json, sys
from pathlib import Path
base = Path(sys.argv[1]); padd = json.loads(sys.argv[2]); split = sys.argv.index('--'); bins = sys.argv[3:split]
names = bins + sys.argv[split + 1:] + ['ELF-CHECK.json', 'BUILD.json']   # SHA256SUMS lists both manifests too
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
glibc = sorted({v[6:] for vs in padd['versions'].values() for v in vs if v.startswith('GLIBC_2')},
               key=lambda v: tuple(map(int, v.split('.'))))
for manifest in ('ELF-CHECK.json', 'BUILD.json'):
    path = base / manifest
    data = json.loads(path.read_text())
    for table in (data, data.get('built_binaries', {})):
        for name in bins:
            if isinstance(table.get(name), dict):
                table[name]['sha256'] = digest(base / name)
                table[name]['bytes'] = (base / name).stat().st_size
                if name == 'padd':
                    table[name]['needed'] = padd['needed']
                    table[name]['glibc_versions'] = glibc
    if manifest == 'BUILD.json':
        data['microduck_replica_patch'] = {
            'what': 'robotd/robotctl/padd and five packaging scripts from microduck-replica (fanhao375/Microduck-kissqy): '
                    'trunk IMU module over I2C, optional servo P/D/torque-limit file, UDP pad and pad-loss sit',
            'variant': 'imu+tunable+udp-pad',
            'padd': 'replaced (built from this branch); unchanged_binaries above is kissqy\'s list and still names it',
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

# SHA256SUMS rows that do not verify, one name per line. A kissqy console upgrade keeps the installed
# model folders but writes the new list, so a fresh V167 already has rows that do not match (seen:
# models/<id>/deployment-contract.json); only rows this script touched, or rows that newly fail, count.
sums_failures() { ( cd "$INSTALL_DIR" && sha256sum -c SHA256SUMS 2>/dev/null || true ) | sed -n 's/: FAILED.*$//p' | sort -u; }

check_sums() {   # $1 = file with the names that already failed before install (P0); 0 ok
    local now ours f extra
    now=$(sums_failures)
    for f in "${BINS[@]}" "${SCRIPTS[@]}" ELF-CHECK.json BUILD.json; do
        if printf '%s\n' "$now" | grep -qxF "$f"; then log ERROR sums "$f 的 SHA256SUMS 行核对不过"; return 1; fi
    done
    extra=$(comm -23 <(printf '%s\n' "$now" | sed '/^$/d') <(sed '/^$/d' "$1" | sort -u))
    if [ -n "$extra" ]; then log ERROR sums "安装前没有、现在核对不过的行：$(echo $extra)"; return 1; fi
    if [ -s "$1" ]; then log WARN sums "安装前就核对不过、这次没碰的行（原样保留）：$(echo $(cat "$1"))"; fi
    return 0
}

# A servo-test-parameters.json left by an older kissqy build is read by this robotd at its next start:
# a valid one takes effect at the next HOME, one outside this robotd's ranges (e.g. kp = 0, which the
# old console allowed) would leave robotd in waiting_for_bus. Prints the content; non-zero = invalid.
servo_file_state() {
    python3 - "$CONFIG_DIR/servo-test-parameters.json" <<'PY'
import json, sys
raw = open(sys.argv[1], encoding='utf-8').read().strip()
print(raw)
try:
    data = json.loads(raw)
    limits = {'kp': (1, 32), 'kd': (0, 64), 'torque_limit': (200, 1000)}
    assert isinstance(data, dict) and set(data) == set(limits), 'keys must be kp, kd, torque_limit'
    for key, (low, high) in limits.items():
        assert type(data[key]) is int and low <= data[key] <= high, f'{key} must be an integer {low}..{high}'
except (ValueError, AssertionError) as exc:
    print(exc); sys.exit(1)
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
# What the module publishes and whether its reports are logged are the operator's choices, not
# its wiring: an old section's lines for them are carried into the new one as they were.
kept_key = re.compile(r'^\s*"?(bno08x_fusion|bno08x_raw_log)"?\s*=')
kept = []
m = header.search(text)
if m:
    nxt = re.compile(r'^\[', re.M).search(text, m.end())
    old = text[m.end():nxt.start() if nxt else len(text)]
    kept = [line for line in old.splitlines() if kept_key.match(line)]
    block += ''.join(line + '\n' for line in kept)
    text = text[:m.start()] + block + ('\n' + text[nxt.start():] if nxt else '')
else:
    text = text.rstrip('\n') + '\n\n' + block
path.write_text(text)
print(f'{path.name}: [trunk_imu] source={source} {device} {address:#04x}'
      + (f'; kept {" ".join(line.split("=")[0].strip() for line in kept)}' if kept else ''))
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

replace_file() {   # src dst: new inode then rename, so a running padd never makes it "Text file busy"
    local tmp
    tmp="$(dirname "$2")/.replica-$(basename "$2").$$"
    cp -a "$1" "$tmp" && mv -f "$tmp" "$2"
}

# $1 backup, $2 auto (after a failed install) | rollback (on request).
# Programs, scripts and manifests, robotd.toml (both) and an older servo parameter file come back;
# the calibration files never do (an IMU re-confirmed since is kept; a difference is reported). A
# rollback first keeps the configs it replaces in $backup/etc-at-rollback/.
# Returns 0 restored and checked, 1 copying failed (do not start anything), 2 restored but a file or
# the checksum list differs (start the services anyway and say so). robotd and padd must be stopped.
restore_from() {
    local backup=$1 why=${2:-rollback} f name rc=0
    for name in "${BINS[@]}" "${SCRIPTS[@]}"; do
        [ -f "$backup/opt/$name" ] || log WARN rollback "备份里没有 $name（旧备份）：留着现在的"
    done
    for f in "$backup"/opt/*; do replace_file "$f" "$INSTALL_DIR/$(basename "$f")" || return 1; done
    if [ "$why" = rollback ]; then
        for name in "${RESTORED_CONFIGS[@]}"; do
            if [ -f "$backup/etc/$name" ] && [ -f "$CONFIG_DIR/$name" ] && ! cmp -s "$backup/etc/$name" "$CONFIG_DIR/$name"; then
                mkdir -p "$backup/etc-at-rollback" && cp -a "$CONFIG_DIR/$name" "$backup/etc-at-rollback/$name" || return 1
                log INFO rollback "撤回前的 $name 另存在 $backup/etc-at-rollback/$name"
            fi
        done
    fi
    for name in "${RESTORED_CONFIGS[@]}"; do
        if [ -f "$backup/etc/$name" ]; then replace_file "$backup/etc/$name" "$CONFIG_DIR/$name" || return 1; fi
    done
    for name in "${CALIBRATIONS[@]}"; do
        if [ -f "$backup/etc/$name" ] && ! cmp -s "$backup/etc/$name" "$CONFIG_DIR/$name"; then
            log WARN rollback "$name 跟备份时不一样（之后重新标定或确认过方向？）：保持现在的，不恢复；备份时的版本在 $backup/etc/$name"
        fi
    done
    if [ ! -f "$backup/etc/servo-test-parameters.json" ] && [ -f "$CONFIG_DIR/servo-test-parameters.json" ]; then
        log WARN rollback 'servo-test-parameters.json 不在备份里（安装之后才写的）：留在原处；不可调的 robotd 不读它，kissqy 的 install/upgrade 会删它'
    fi
    if [ "$NO_SYSTEMD" != 1 ] && [ -f "$backup/etc/dropin.conf" ]; then replace_file "$backup/etc/dropin.conf" "$DROPIN" || return 1; fi
    if ! padd_dropin_managed; then
        :
    elif [ -f "$backup/etc/padd-dropin.absent" ]; then
        rm -f "$PADD_DROPIN" || return 1
    elif [ -f "$backup/etc/padd-dropin.conf" ]; then
        { mkdir -p "$(dirname "$PADD_DROPIN")" && replace_file "$backup/etc/padd-dropin.conf" "$PADD_DROPIN"; } || return 1
    else
        log WARN rollback "备份里没有 padd drop-in 的记录（旧备份）：$PADD_DROPIN 不动"
    fi
    for f in "$backup"/opt/*; do
        cmp -s "$f" "$INSTALL_DIR/$(basename "$f")" || { log ERROR rollback "$(basename "$f") 和备份不一致"; rc=2; }
    done
    for name in "${RESTORED_CONFIGS[@]}"; do
        if [ -f "$backup/etc/$name" ] && ! cmp -s "$backup/etc/$name" "$CONFIG_DIR/$name"; then
            log ERROR rollback "$name 和备份不一致"; rc=2
        fi
    done
    if [ -f "$backup/sha256-p0.txt" ]; then
        check_sums_subset "$backup/sha256-p0.txt" || rc=2
    else
        log WARN rollback "旧备份没有安装前的核对记录；现在核对不过的行：$(echo $(sums_failures))"
    fi
    if ls "$INSTALL_DIR"/models/*/deployment-execution.json.orig >/dev/null 2>&1; then
        log WARN rollback '有 deployment-execution.json.orig：joint-filters 改过的头腿滤波不会自动恢复（service.sh joint-filters <id> 0.50 0.70，或把 .orig 拷回）'
    fi
    return $rc
}

check_sums_subset() {   # after a rollback: every failing row must have failed before the install
    local extra
    extra=$(comm -23 <(sums_failures | sed '/^$/d') <(sed '/^$/d' "$1" | sort -u))
    if [ -n "$extra" ]; then log ERROR rollback "撤回后核对不过、安装前却是好的行：$(echo $extra)"; return 1; fi
    return 0
}

cmd_check() {
    local kind
    echo "安装目录：$INSTALL_DIR"
    echo "已装版本：$(installed_build || true)（这个脚本要求 $WANT_BUILD）"
    if [ -f "$INSTALL_DIR/robotd" ]; then
        if grep -a -q 'servo-test-parameters.json' "$INSTALL_DIR/robotd"; then kind='microduck-replica（IMU 模块 + 可调）'
        elif grep -a -q 'lsm6dsv16x' "$INSTALL_DIR/robotd"; then kind='带 IMU 模块支持（不可调）'
        else kind='kissqy 原版（不认 IMU 模块）'; fi
        echo "robotd：$(sha256sum "$INSTALL_DIR/robotd" | cut -c1-16)…  $kind"
    fi
    if [ -f "$INSTALL_DIR/padd" ]; then
        if grep -a -q 'netpad on' "$INSTALL_DIR/padd"; then kind='microduck-replica（UDP 手柄 + 掉线坐下）'; else kind='kissqy 原版'; fi
        echo "padd：$(sha256sum "$INSTALL_DIR/padd" | cut -c1-16)…  $kind"
    fi
    if [ "$NO_SYSTEMD" != 1 ]; then
        echo "robotd：$(robotd_rpc state 2>/dev/null || echo 读不了)"
        echo "padd 协议族：$(systemctl show padd.service -p RestrictAddressFamilies --value 2>/dev/null || true)"
        for kind in robotd padd gamesir-xboxd; do
            echo "$kind：$(systemctl is-active "$kind.service" 2>/dev/null || true) / $(systemctl is-enabled "$kind.service" 2>/dev/null || true)"
        done
        for kind in $(systemctl show robotd.service -p ConsistsOf --value 2>/dev/null || true); do
            echo "随 robotd 停止的 $kind：$(systemctl is-active "$kind" 2>/dev/null || true)"
        done
    fi
    echo "padd AF_INET drop-in：$( [ -f "$PADD_DROPIN" ] && echo "$PADD_DROPIN" || echo 无)"
    echo "I2C 设备：$(ls /dev/i2c-* 2>/dev/null | tr '\n' ' ')"
    grep -A5 -E '^\[\s*"?trunk_imu"?\s*\]' "$CONFIG_DIR/robotd.toml" 2>/dev/null || echo 'robotd.toml 里没有 [trunk_imu]（默认读总线 IMU 小板）'
    for kind in bno08x lsm6dsv16x; do echo "安装方向 $kind：$(mount_state "$kind" 2>/dev/null || echo '读不了标定文件')"; done
    if grep -q 'DeviceAllow=char-i2c rw' "$DROPIN" 2>/dev/null; then echo '启动配置：允许 /dev/i2c-*'; else echo '启动配置：不允许 /dev/i2c-*'; fi
    python3 - "$CONFIG_DIR" <<'PY' || echo '（参数文件 / 滤波 / netpad 需要 Python 3.11 的 tomllib 才能读）'
import json, sys, tomllib
from pathlib import Path
config = Path(sys.argv[1])
servo = config / 'servo-test-parameters.json'
print('舵机参数文件：' + (servo.read_text().strip() + '（下次 HOME 生效）' if servo.exists() else '无（V167 原版：P=合同 5、D=0、转矩 1000）'))
for name in ('robotd.toml', 'robotd.saved.toml'):
    path = config / name
    if not path.exists(): continue
    data = tomllib.loads(path.read_text())
    netpad = data.get('netpad')
    print(f'{name} [netpad]：' + (json.dumps(netpad, ensure_ascii=False) if netpad else '无（关）'))
walk = Path(str(tomllib.loads((config / 'robotd.toml').read_text()).get('policy', {}).get('walk', '')))
execution = walk.parent / 'deployment-execution.json'
if execution.exists():
    profile = json.loads(execution.read_text())
    orig = execution.with_name(execution.name + '.orig')
    print(f"Walk 头/腿滤波：{profile.get('head_lowpass')} / {profile.get('legs_lowpass')}"
          f"{'（有 .orig，改过）' if orig.exists() else ''}")
PY
    echo "SHA256SUMS 核对不过的行：$(echo $(sums_failures)) "
}

cmd_install() {
    [ $# -eq 4 ] || die usage '用法：install <文件目录> <bno08x|lsm6dsv16x> <i2c 设备> <地址>'
    local src=$1 source=$2 device=$3 address f
    [[ $4 =~ ^(0[xX][0-9a-fA-F]+|[0-9]+)$ ]] || die usage "地址要写成 0x4B 或 75 这样，不是 $4"
    address=$(( $4 ))
    need_root
    take_lock
    log INFO precheck "开始安装：$src $source $device $(printf '%#04x' "$address")"
    [ "$(installed_build)" = "$WANT_BUILD" ] \
        || die precheck "板上是 $(installed_build || echo 未知版本)，不是 $WANT_BUILD：先用 kissqy 中控 1.0.167 升级到 V167"
    case $source in
        bno08x) [[ $address == 74 || $address == 75 ]] || die precheck 'BNO08x 地址只能是 0x4A 或 0x4B' ;;
        lsm6dsv16x) [[ $address == 106 || $address == 107 ]] || die precheck 'LSM6DSV16X 地址只能是 0x6A 或 0x6B' ;;
        *) die precheck "source 只能是 bno08x 或 lsm6dsv16x，不是 $source" ;;
    esac
    SOURCE_KIND=$source
    [ -c "$device" ] || [ "$NO_SYSTEMD" = 1 ] || die precheck "$device 不存在：先开对应的 I2C overlay（见 IMU-MODULE.md）"
    for f in "${BINS[@]}" "${SCRIPTS[@]}"; do [ -f "$src/$f" ] || die precheck "缺文件 $src/$f"; done
    python3 - "$INSTALL_DIR" "$src/robotd" "$src/robotctl" "$src/padd" <<'PY' >>"$LOG" 2>&1 || die precheck 'robotd / robotctl / padd 不是完整的 aarch64 程序（见日志）'
import sys
sys.path.insert(0, sys.argv[1])
from elf_integrity import validate_elf   # the board's own check, the one kissqy's install runs
for p in sys.argv[2:]:
    print(p, validate_elf(p))
PY
    grep -a -q "$WANT_BUILD" "$src/robotd" || die precheck "robotd 不是 $WANT_BUILD 编出来的"
    grep -a -q 'lsm6dsv16x' "$src/robotd" || die precheck 'robotd 里没有 IMU 模块支持，是不是拿成了 kissqy 原版？'
    { grep -a -q 'servo-test-parameters.json' "$src/robotd" && grep -a -q "$FILE_GAIN_MODE" "$src/robotd"; } \
        || die precheck 'robotd 不是可调版（没有 servo-test-parameters.json），是不是拿成了只带 IMU 的旧版？'
    grep -a -q 'netpad on' "$src/padd" || die precheck 'padd 不认识 [netpad]，是不是拿成了 kissqy 原版 padd？'
    for f in "${MANIFESTS[@]}" "${BINS[@]}" "${SCRIPTS[@]}" elf_integrity.py startup_gate.py; do
        [ -f "$INSTALL_DIR/$f" ] || die precheck "板上缺 $INSTALL_DIR/$f"
    done
    local elf
    elf=$(check_padd_elf "$src/padd" 2>&1) || die precheck "$(echo "$elf" | tail -1)"
    log INFO precheck "$elf"
    bash -n "$src/service.sh" || die precheck 'service.sh 语法不对'
    { grep -q '^servo-parameters)' "$src/service.sh" && grep -q '^netpad)' "$src/service.sh"; } \
        || die precheck 'service.sh 不是这个分支的（没有 servo-parameters / netpad）'
    { grep -q 'def relax_and_restore' "$src/runtime.py" && grep -q 'def set_netpad' "$src/runtime.py"; } \
        || die precheck 'runtime.py 不是这个分支的（没有 relax_and_restore / set_netpad）'
    if [ "$NO_SYSTEMD" = 1 ] || [ "$SKIP_ELF_RUN" = 1 ]; then
        log INFO precheck '（测试模式）跳过 ldd 和 padd --help'
    else
        local run_dir
        run_dir=$(mktemp -d)
        install -m 755 "$src/padd" "$run_dir/padd"
        ldd "$run_dir/padd" > "$run_dir/ldd.txt" 2>&1 || true   # to a file first: no pipe to lose under pipefail
        if grep -q 'not found' "$run_dir/ldd.txt"; then
            cat "$run_dir/ldd.txt" >>"$LOG"; rm -rf "$run_dir"; die precheck 'padd 在板上缺库或缺符号版本（ldd 见日志）'
        fi
        # BIND_NOW: loading resolves every symbol, so --help proves this padd runs on this board.
        "$run_dir/padd" --help >>"$LOG" 2>&1 || { rm -rf "$run_dir"; die precheck 'padd --help 在板上跑不起来（见日志）'; }
        rm -rf "$run_dir"
        log INFO precheck 'padd 在板上能加载（ldd、--help 通过）'
    fi
    if [ -f "$CONFIG_DIR/servo-test-parameters.json" ]; then
        local servo
        servo=$(servo_file_state 2>&1) || die precheck "已有 servo-test-parameters.json 超出这个 robotd 收的范围（$(echo $servo)），装上后 robotd 会停在 waiting_for_bus：先删掉或改好它（sudo rm $CONFIG_DIR/servo-test-parameters.json），什么都没改"
        log WARN tuning "已有 servo-test-parameters.json：$(echo "$servo" | head -1)；装上后下次 HOME 上力就用它（不想要：service.sh servo-parameters-clear）"
    fi
    log INFO precheck '检查通过'

    relax_and_stop

    local backup
    backup=$BACKUP_ROOT/imu-module-backup-$(date +%Y%m%d-%H%M%S)
    mkdir -p "$backup/opt" "$backup/etc" || die backup "建不了 $backup"
    for f in "${BINS[@]}" "${SCRIPTS[@]}" "${MANIFESTS[@]}"; do cp -a "$INSTALL_DIR/$f" "$backup/opt/" || die backup "备份 $f 失败"; done
    for f in "${CONFIGS[@]}"; do
        if [ -f "$CONFIG_DIR/$f" ]; then cp -a "$CONFIG_DIR/$f" "$backup/etc/" || die backup "备份 $f 失败"; fi
    done
    if [ -f "$DROPIN" ]; then cp -a "$DROPIN" "$backup/etc/dropin.conf" || die backup '备份 robotd 启动配置失败'; fi
    if [ -f "$PADD_DROPIN" ]; then cp -a "$PADD_DROPIN" "$backup/etc/padd-dropin.conf" || die backup '备份 padd drop-in 失败'
    else : > "$backup/etc/padd-dropin.absent"; fi
    sums_failures > "$backup/sha256-p0.txt"
    BACKUP=$backup
    log INFO backup "已备份到 $backup"
    if [ -s "$backup/sha256-p0.txt" ]; then log WARN sums "安装前 SHA256SUMS 就核对不过的行：$(echo $(cat "$backup/sha256-p0.txt"))"; fi
    if [ -f "$CONFIG_DIR/servo-test-parameters.json" ]; then log INFO backup '已有 servo-test-parameters.json：已备份，留在原处'; fi

    for f in "${BINS[@]}"; do install -o root -g root -m 755 "$src/$f" "$INSTALL_DIR/$f" || die files "换 $f 失败"; done
    for f in "${SCRIPTS[@]}"; do
        local mode=644
        if [ "$f" = service.sh ]; then mode=755; fi
        install -o root -g root -m "$mode" "$src/$f" "$INSTALL_DIR/$f" || die files "换 $f 失败"
    done
    log INFO files '已换 robotd、robotctl、padd 和 5 个安装脚本'
    update_manifests >>"$LOG" 2>&1 || die files '更新清单失败'
    check_sums "$backup/sha256-p0.txt" || die files 'SHA256SUMS 核对不过'
    log INFO files 'SHA256SUMS、ELF-CHECK.json、BUILD.json 已更新；换过的行全部核对通过，没有新的不一致'

    for f in robotd.toml robotd.saved.toml; do
        if [ -f "$CONFIG_DIR/$f" ]; then
            set_trunk_imu "$CONFIG_DIR/$f" "$source" "$device" "$address" >>"$LOG" 2>&1 || die config "改 $f 失败"
        fi
    done
    log INFO config "robotd.toml / robotd.saved.toml：[trunk_imu] = $source $device $(printf '%#04x' "$address")"

    rewrite_dropin
    write_padd_dropin
    start_services install || die start 'robotd / padd 没有按新版本起来（见日志）'
    partof_report

    local mount
    mount=$(mount_state "$source" 2>/dev/null) || mount='读不了标定文件'
    case $mount in
        verified*) log INFO mount "$source 的安装方向已确认：${mount#verified }" ;;
        *) log WARN mount "$source 的安装方向还没确认（$mount）：运动模式会拒绝上力，按 IMU-MODULE.md 先确认方向" ;;
    esac
    if [ -f "$CONFIG_DIR/servo-test-parameters.json" ]; then
        log WARN tuning "有舵机参数文件 $(tr -d '\n' < "$CONFIG_DIR/servo-test-parameters.json")：下次 HOME 上力时生效"
    fi
    input_report
    say "完成。备份：$backup"
    say "撤回（回到这次安装之前）：sudo bash $0 rollback $backup"
}

cmd_rollback() {
    local old=0
    if [ "${1:-}" = --old-backup ]; then old=1; shift; fi
    # An empty or wrong argument must never reach restore_from: "$1/opt" would then be /opt itself.
    { [ $# -eq 1 ] && [ -n "$1" ] && [ -f "$1/opt/robotd" ] && [ -d "$1/etc" ]; } \
        || die usage '用法：rollback [--old-backup] <install 打印的备份目录>（要有 opt/robotd 和 etc/）'
    need_root
    local backup=$1 rc=0 name
    if [ ! -f "$backup/sha256-p0.txt" ] && [ "$old" != 1 ]; then
        die usage "$backup 是第一版安装脚本打的旧备份（没有安装前的核对记录，也没有 padd、service.sh、padd 配置）；用这次安装新打的备份，确实要用它就加 --old-backup"
    fi
    for name in "${CALIBRATIONS[@]}"; do
        if grep -q 'imu_mounts' "$CONFIG_DIR/$name" 2>/dev/null && ! grep -a -q 'lsm6dsv16x' "$backup/opt/robotd"; then
            die usage "备份里的 robotd 是 kissqy 原版，读不了现在 $name 里的 [imu_mounts]（撤回后会一直等总线）。用这次安装新打的备份；真要回原版，先删掉两份标定文件里的 [imu_mounts.*] 再撤回"
        fi
    done
    take_lock
    ROLLBACK_FROM=$backup
    log INFO rollback "从 $backup 恢复（回到那次安装之前；标定文件保持现在的）"
    relax_and_stop
    STOPPED=0   # from here on a failure must not start anything on half-restored files
    restore_from "$backup" rollback || rc=$?
    if [ "$rc" = 1 ]; then die rollback "复制备份失败，robotd 没有启动；备份还在 $backup"; fi
    start_services rollback || die rollback 'robotd / padd 没起来，看 journalctl -u robotd -u padd'
    partof_report
    input_report
    if [ "$rc" = 2 ]; then
        log WARN rollback '已恢复并启动，但核对有差异（见上面的 ERROR）'
        exit 2
    fi
    log INFO rollback '已恢复'
}

case ${1:-} in
    check) shift; cmd_check "$@" ;;
    install) shift; cmd_install "$@" ;;
    rollback) shift; cmd_rollback "$@" ;;
    *) sed -n '2,27p' "$0"; exit 2 ;;
esac
