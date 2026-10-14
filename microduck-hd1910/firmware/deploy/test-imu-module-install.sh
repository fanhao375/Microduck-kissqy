#!/usr/bin/env bash
# Added 2026-10 by fanhao375 (microduck-replica): fake-board tests for imu-module-install.sh; Apache-2.0 like the rest of this workspace.
#
# Runs the installer against fake boards in a temporary directory. It never touches the real
# systemd, robotd or /etc: systemctl / journalctl are stubs on PATH, robotd is a small fake that
# answers busStatus / relax / startupSubscribe on a UNIX socket in the temporary directory.
#
#   sudo bash deploy/test-imu-module-install.sh <release dir> <kissqy V167 packaging dir> [IMU-only robotd]
#     <release dir>                robotd, robotctl, padd (aarch64) built from this branch (cargo board)
#     <kissqy V167 packaging dir>  packaging/ of a kissqy 78bef11 checkout after prepare-V167.py: the
#                                  installed V167 the fake board starts from
#     [IMU-only robotd]            optional: a robotd from the IMU-only build, which must be refused
#   PY=<python 3.11+> picks the interpreter the installer and the checks run with (default python3).
# Needs Linux, root (the installer uses install -o root), sha256sum, comm, cmp.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
SCRIPT=$HERE/deploy/imu-module-install.sh
RELEASE=${1:?release dir} KISSQY=${2:?kissqy packaging dir} IMU_ONLY=${3:-}
PY=$(command -v "${PY:-python3}")
BASE_PATH=$PATH
W=$(mktemp -d /tmp/imu-install-test.XXXXXX)
PASS=0 FAIL=0 SKIP=0
ok() { if eval "$2"; then echo "PASS $1"; PASS=$((PASS + 1)); else echo "FAIL $1"; FAIL=$((FAIL + 1)); fi; }
skip() { echo "SKIP $1"; SKIP=$((SKIP + 1)); }

# --- fakes ------------------------------------------------------------------------------------
write_fakes() {   # $R: python3 wrapper, systemctl / journalctl stubs, fake robotd
    printf '#!/bin/bash\nexec %q "$@"\n' "$PY" > "$R/bin/python3"
    cat > "$R/bin/systemctl" <<EOF
#!/bin/bash
R=$R
EOF
    cat >> "$R/bin/systemctl" <<'EOF'
echo "$*" >> "$R/systemctl.log"
unit=; for a in "$@"; do case $a in *.service) unit=${a%.service} ;; esac; done
case $1 in
  is-active)
    st=$(cat "$R/units/$unit.active" 2>/dev/null || echo inactive)
    [ "${2:-}" = --quiet ] || echo "$st"; [ "$st" = active ] ;;
  is-enabled)
    st=$(cat "$R/units/$unit.enabled" 2>/dev/null || echo disabled); echo "$st"; [ "$st" = enabled ] ;;
  stop)
    for u in "${@:2}"; do u=${u%.service}; echo inactive > "$R/units/$u.active"
      if [ "$u" = robotd ] && [ -f "$R/units/robotd.pid" ]; then
        kill "$(cat "$R/units/robotd.pid")" 2>/dev/null; rm -f "$R/units/robotd.pid" "$R/robotd.sock"; fi
    done ;;
  start|restart)
    for u in "${@:2}"; do u=${u%.service}
      case $u in
        robotd) [ -f "$R/units/robotd.pid" ] || { python3 "$R/fake_robotd.py" "$R/robotd.sock" "$R/robotd-after.json" >>"$R/fake-robotd.log" 2>&1 & echo $! > "$R/units/robotd.pid"; } ;;
        padd) echo $((RANDOM + 1000)) > "$R/units/padd.pid" ;;
      esac
      echo active > "$R/units/$u.active"
    done ;;
  show)
    prop=; while [ $# -gt 0 ]; do [ "$1" = -p ] && prop=$2; shift; done
    case $prop in
      MainPID) cat "$R/units/padd.pid" 2>/dev/null || echo 0 ;;
      LoadState) echo loaded ;;
      RestrictAddressFamilies)
        f='AF_UNIX AF_NETLINK'; [ -f "$R/systemd/padd.service.d/zzzz-replica-netpad.conf" ] && f="$f AF_INET"; echo "$f" ;;
      *) echo ;;
    esac ;;
esac
exit 0
EOF
    cat > "$R/bin/journalctl" <<EOF
#!/bin/bash
for a in "\$@"; do case \$a in _PID=*) [ -f $R/padd-driving ] && echo 'driving — Start once stands up (fake padd)' ;; esac; done
exit 0
EOF
    chmod +x "$R/bin/python3" "$R/bin/systemctl" "$R/bin/journalctl"
    cat > "$R/fake_robotd.py" <<'EOF'
import json, os, socketserver, sys, threading
path, scenario = sys.argv[1], json.load(open(sys.argv[2]))
calls = os.path.join(os.path.dirname(path), 'robotd-calls.log')
lock = threading.Lock()
armed = scenario['mode'] == 'motion' and scenario['phase'] == 'control' and scenario.get('armed', False)
state = {'torque': armed, 'restore': armed, 'applied': armed}

def report():
    with lock:
        r = {'build': scenario.get('build', '1fa8438-feetech-ft6-control.43'), 'mode': scenario['mode'], 'phase': scenario['phase']}
        if scenario['mode'] == 'motion' and scenario['phase'] == 'control':
            hd = {'restore_pending': state['restore'], 'temporary_gains_applied': state['applied']}
            if scenario.get('servo_parameters_key', True): hd['servo_parameters'] = None
            r['native'] = {'torque_state_confirmed': state['torque'], 'imu_source': scenario.get('imu_source', 'bno08x'),
                           'gain_mode': scenario.get('gain_mode', 'hd1910_model_contract_p_restore_on_relax_native_d_torque_limit_preserved'),
                           'hd1910': hd}
        return r

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            request = json.loads(line); method = request.get('method')
            with open(calls, 'a') as f: f.write(method + '\n')
            if method == 'robot.busStatus': result = report()
            elif method == 'robot.relax':
                with lock:
                    state['torque'] = False
                    if scenario.get('settles', True): state['restore'] = state['applied'] = False
                result = {'accepted': True}
            elif method == 'robot.startupSubscribe':
                good = scenario.get('startup_ok', True)
                result = {'steps': {str(n): {'status': 'success' if good or n == 1 else 'waiting', 'detail': 'fake'} for n in range(1, 5)}}
            else: result = {}
            self.wfile.write((json.dumps({'jsonrpc': '2.0', 'id': request.get('id'), 'result': result}) + '\n').encode())
            self.wfile.flush()

class Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True

# Bind and listen under a temporary name, then rename into place: a client woken by the new name
# (startup_gate watches the directory) never meets a bound socket that is not listening yet.
temporary = path + '.tmp'
for stale in (path, temporary):
    if os.path.exists(stale): os.unlink(stale)
server = Server(temporary, Handler)
os.rename(temporary, path)
server.serve_forever()
EOF
}

board() {   # a fresh fake board at $R; $1 = a (NO_SYSTEMD) | b (stubbed systemd + fake robotd)
    stop_fake
    R=$W/board-$((++BOARDS)); mkdir -p "$R"/{opt,etc,src,systemd,bin,units}
    cp -a "$KISSQY/." "$R/opt/"
    cp "$RELEASE/robotd" "$RELEASE/robotctl" "$RELEASE/padd" "$R/src/"
    for f in calibration-config.py rebuild-config.py runtime.py voice-setup.py service.sh; do cp "$HERE/packaging/$f" "$R/src/"; done
    printf '["bus"]\n"protocol" = "feetech"\n\n["policy"]\n"action_scale" = 0.7\n' > "$R/etc/robotd.toml"; cp "$R/etc/robotd.toml" "$R/etc/robotd.saved.toml"
    printf '[imu_mounts.bno08x]\nquat = [0.459554518, 0.539556409, 0.532044137, 0.463268349]\nverified = true\n' > "$R/etc/hd1910-calibration.toml"
    cp "$R/etc/hd1910-calibration.toml" "$R/etc/hd1910-calibration.saved.toml"
    write_fakes
    echo '{"mode": "motion", "phase": "control"}' > "$R/robotd-after.json"
    touch "$R/padd-driving"
    export INSTALL_DIR=$R/opt CONFIG_DIR=$R/etc BACKUP_ROOT=$R LOG=$R/install.log SYSTEMD_DIR=$R/systemd \
        ROBOTD_SOCKET=$R/robotd.sock READY_TIMEOUT=8 PATH=$R/bin:$BASE_PATH
    if [ "$1" = a ]; then export NO_SYSTEMD=1 SKIP_ELF_RUN=0; else export NO_SYSTEMD=0 SKIP_ELF_RUN=1; fi
}
BOARDS=0

fake_before() {   # a running robotd before install: $1 = scenario JSON
    echo "$1" > "$R/robotd-before.json"
    "$PY" "$R/fake_robotd.py" "$R/robotd.sock" "$R/robotd-before.json" >>"$R/fake-robotd.log" 2>&1 &
    echo $! > "$R/units/robotd.pid"; echo active > "$R/units/robotd.active"
    for _ in $(seq 1 50); do [ -S "$R/robotd.sock" ] && break; sleep 0.1; done
}
stop_fake() { [ -n "${R:-}" ] && [ -f "$R/units/robotd.pid" ] && kill "$(cat "$R/units/robotd.pid")" 2>/dev/null; true; }
trap stop_fake EXIT

state() { (cd "$R/opt" && sha256sum robotd robotctl padd runtime.py service.sh SHA256SUMS | sha256sum | cut -c1-12); }
backup_of() { grep -o "$R/imu-module-backup-[0-9-]*" <<<"$1" | head -1; }
toml() { "$PY" -c "import tomllib,sys;d=tomllib.load(open(sys.argv[1],'rb'));$2" "$3"; }

# --- A: NO_SYSTEMD (the 21 checks of the first version, then the new ones) ----------------------
board a
( cd "$R/opt" && sha256sum -c --quiet SHA256SUMS ) >/dev/null 2>&1 && echo "PASS 原版安装目录 SHA256SUMS 自洽" || echo "NOTE 原版 SHA256SUMS 本来就不全对"
orig=$(state)
bash "$SCRIPT" check | sed 's/^/  check> /'
bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4C >/dev/null 2>&1; ok "地址 0x4C 被拒" "[ \$? -ne 0 ]"
bash "$SCRIPT" install "$R/src" lsm6dsv16x /dev/i2c-4 0x4B >/dev/null 2>&1; ok "LSM6DSV16X 用 0x4B 被拒" "[ \$? -ne 0 ]"
mkdir -p "$R/badsrc"; cp "$R"/src/* "$R/badsrc/"; cp "$KISSQY/robotd" "$R/badsrc/robotd"
bash "$SCRIPT" install "$R/badsrc" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; ok "kissqy 原版 robotd 被拒" "[ \$? -ne 0 ]"
sed -i 's/control\.43/control.40/' "$R/opt/release_identity.py"
bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; ok "板上是 control.40 被拒" "[ \$? -ne 0 ]"
sed -i 's/control\.40/control.43/' "$R/opt/release_identity.py"
ok "被拒后安装目录没动" "[ \"\$(state)\" = $orig ]"
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); echo "$out" | tail -2 | sed 's/^/  install> /'
BK=$(backup_of "$out")
ok "robotd 已换成我们的" "cmp -s $R/opt/robotd $R/src/robotd"
ok "SHA256SUMS 核对通过" "(cd $R/opt && sha256sum -c --quiet SHA256SUMS)"
ok "robotd.toml 有 trunk_imu bno08x 75" "toml x \"assert d['trunk_imu']=={'source':'bno08x','i2c_device':'/dev/i2c-4','i2c_address':75},d\" $R/etc/robotd.toml"
ok "robotd.saved.toml 也有" "grep -q trunk_imu $R/etc/robotd.saved.toml"
ok "原有配置保留" "toml x \"assert d['policy']['action_scale']==0.7\" $R/etc/robotd.toml"
ok "BUILD.json 带补丁说明" "grep -q microduck_replica_patch $R/opt/BUILD.json"
ok "日志里有安装方向已确认" "grep -q '安装方向已确认' $R/install.log"
sleep 1; bash "$SCRIPT" install "$R/src" lsm6dsv16x /dev/i2c-3 0x6B >/dev/null 2>&1
ok "再装一次：trunk_imu 被替换成 lsm6dsv16x 0x6B 且只有一段" "\"$PY\" -c \"import tomllib;t=open('$R/etc/robotd.toml').read();d=tomllib.loads(t);assert d['trunk_imu']['source']=='lsm6dsv16x' and d['trunk_imu']['i2c_address']==107 and t.count('trunk_imu')==1\""
ok "lsm6dsv16x 安装方向没确认 → 警告" "grep -q 'lsm6dsv16x 的安装方向还没确认' $R/install.log"
bash "$SCRIPT" check | sed 's/^/  check> /'
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; ok "撤回成功" "[ \$? -eq 0 ]"
ok "撤回后安装目录恢复原样" "[ \"\$(state)\" = $orig ]"
ok "撤回后 robotd.toml 没有 trunk_imu" "! grep -q trunk_imu $R/etc/robotd.toml"
printf '#!/bin/bash\ncase "$2" in *robotd.saved.toml) exit 1;; esac\nexec %q "$@"\n' "$PY" > "$R/bin/python3"
bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
ok "改配置失败 → 安装报错" "[ $rc -ne 0 ]"
ok "改配置失败 → 自动撤回，安装目录回到原样" "[ \"\$(state)\" = $orig ]"
ok "日志写了已撤回" "grep -q '已撤回，板子回到安装前的样子' $R/install.log"
printf '#!/bin/bash\nexec %q "$@"\n' "$PY" > "$R/bin/python3"

# new in this version, still NO_SYSTEMD
board a; orig=$(state)
echo '{"kp": 3, "kd": 0, "torque_limit": 700}' > "$R/etc/servo-test-parameters.json"
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); rc=$?; BK=$(backup_of "$out")
ok "整套安装成功" "[ $rc -eq 0 ]"
ok "service.sh 换成本分支的且是 755" "cmp -s $R/opt/service.sh $R/src/service.sh && [ \$(stat -c %a $R/opt/service.sh) = 755 ]"
ok "service.sh、padd 的 SHA256SUMS 行已更新" "(cd $R/opt && grep -E '  (service.sh|padd)\$' SHA256SUMS | sha256sum -c --quiet)"
ok "padd 已换，ELF-CHECK / BUILD.json 的 padd sha256 跟着改" "cmp -s $R/opt/padd $R/src/padd && \"$PY\" -c \"import json,hashlib,sys;h=hashlib.sha256(open('$R/opt/padd','rb').read()).hexdigest();assert json.load(open('$R/opt/ELF-CHECK.json'))['padd']['sha256']==h==json.load(open('$R/opt/BUILD.json'))['built_binaries']['padd']['sha256']\""
ok "BUILD.json 标出整套变体" "grep -q 'imu+tunable+udp-pad' $R/opt/BUILD.json"
ok "padd 的 AF_INET drop-in 写到（假）systemd 目录" "grep -qx 'RestrictAddressFamilies=AF_INET' $R/systemd/padd.service.d/zzzz-replica-netpad.conf"
ok "已有参数文件：进了备份、留在原处、内容没变、日志提示下次 HOME 生效" "cmp -s $R/etc/servo-test-parameters.json $BK/etc/servo-test-parameters.json && grep -q '下次 HOME 上力' $R/install.log"
ok "NO_SYSTEMD：跳过 ldd / --help 和重启" "grep -q '跳过 ldd 和 padd --help' $R/install.log && grep -q '跳过 daemon-reload / 启动 / 重启 padd' $R/install.log"
ok "备份里记下安装前的核对结果和 padd drop-in 原来没有" "[ -f $BK/sha256-p0.txt ] && [ -f $BK/etc/padd-dropin.absent ]"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
ok "撤回：成功、目录回原样、drop-in 删掉" "[ $rc -eq 0 ] && [ \"\$(state)\" = $orig ] && [ ! -e $R/systemd/padd.service.d/zzzz-replica-netpad.conf ]"

board a; orig=$(state)
echo '{"kp": 0, "kd": 0, "torque_limit": 681}' > "$R/etc/servo-test-parameters.json"
bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
ok "旧版留下的参数文件超范围（kp=0）→ 启动前拒绝，什么都没改" "[ $rc -ne 0 ] && [ \"\$(state)\" = $orig ] && grep -q 'waiting_for_bus' $R/install.log"

board a; orig=$(state)
cp "$R"/src/* "$R/badsrc/" 2>/dev/null || { mkdir -p "$R/badsrc"; cp "$R"/src/* "$R/badsrc/"; }
cp "$KISSQY/padd" "$R/badsrc/padd"
bash "$SCRIPT" install "$R/badsrc" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
ok "源目录放的是 kissqy 原版 padd → 拒绝，什么都没改" "[ $rc -ne 0 ] && [ \"\$(state)\" = $orig ] && grep -q '原版 padd' $R/install.log"
cp "$R/src/padd" "$R/badsrc/padd"; cp "$KISSQY/service.sh" "$R/badsrc/service.sh"
bash "$SCRIPT" install "$R/badsrc" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
ok "源目录放的是 kissqy 原版 service.sh → 拒绝" "[ $rc -ne 0 ] && [ \"\$(state)\" = $orig ]"
if [ -n "$IMU_ONLY" ] && [ -f "$IMU_ONLY" ]; then
    cp "$R/src/service.sh" "$R/badsrc/service.sh"; cp "$IMU_ONLY" "$R/badsrc/robotd"
    bash "$SCRIPT" install "$R/badsrc" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
    ok "只带 IMU、不可调的 robotd → 拒绝" "[ $rc -ne 0 ] && [ \"\$(state)\" = $orig ] && grep -q '不是可调版' $R/install.log"
else
    skip "只带 IMU、不可调的 robotd → 拒绝（没给第 3 个参数）"
fi

board a
contract=$(cd "$R/opt" && grep -o 'models/[0-9a-f]*/deployment-contract.json' SHA256SUMS | head -1)
printf '\n' >> "$R/opt/$contract"; orig=$(state); before=$(sha256sum "$R/opt/$contract")
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); rc=$?; BK=$(backup_of "$out")
ok "安装前 $contract 已不一致 → 安装照样成功，日志列出这一行" "[ $rc -eq 0 ] && grep -q \"安装前 SHA256SUMS 就核对不过的行：$contract\" $R/install.log"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
ok "同一情况手动撤回成功，那一行原样保留" "[ $rc -eq 0 ] && [ \"\$(state)\" = $orig ] && [ \"\$(sha256sum $R/opt/$contract)\" = \"$before\" ]"
printf '#!/bin/bash\ncase "$2" in *robotd.saved.toml) exit 1;; esac\nexec %q "$@"\n' "$PY" > "$R/bin/python3"
bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
ok "同一情况安装中途失败 → 自动撤回完成" "[ $rc -ne 0 ] && [ \"\$(state)\" = $orig ] && grep -q '已撤回，板子回到安装前的样子' $R/install.log"
printf '#!/bin/bash\nexec %q "$@"\n' "$PY" > "$R/bin/python3"

board a
mkdir -p "$R/systemd/padd.service.d"; printf '[Service]\n# an older drop-in\n' > "$R/systemd/padd.service.d/zzzz-replica-netpad.conf"
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); BK=$(backup_of "$out")
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1
ok "原来就有的 padd drop-in：撤回后恢复原内容" "grep -q 'an older drop-in' $R/systemd/padd.service.d/zzzz-replica-netpad.conf"

board a; orig=$(state)
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); BK=$(backup_of "$out")
cp /bin/sleep "$R/opt/padd"; "$R/opt/padd" 60 & SLEEPER=$!; sleep 0.2
cp "$R/src/padd" "$R/opt/padd" 2>/dev/null; busy=$?
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
kill $SLEEPER 2>/dev/null; wait $SLEEPER 2>/dev/null
ok "padd 正在运行时撤回：直接 cp 会 Text file busy，撤回照样成功并恢复 padd" "[ $busy -ne 0 ] && [ $rc -eq 0 ] && cmp -s $R/opt/padd $BK/opt/padd && [ \"\$(state)\" = $orig ]"

board a
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); BK=$(backup_of "$out")
printf '# broken after install\n' >> "$R/opt/startup_gate.py"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
ok "撤回后出现安装前没有的核对差异 → 退出码 2（文件照样恢复），日志写明" "[ $rc -eq 2 ] && grep -q '撤回后核对不过、安装前却是好的行：startup_gate.py' $R/install.log && cmp -s $R/opt/robotd $BK/opt/robotd"

# --- B: stubbed systemd + fake robotd ----------------------------------------------------------
# The device check is real here, so the I2C device is /dev/null (a character device every host has).
board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
echo enabled > "$R/units/gamesir-xboxd.enabled"; echo active > "$R/units/padd.active"
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B 2>&1); rc=$?; BK=$(backup_of "$out")
ok "B 控制中：卸力并等恢复完成，再停 robotd、停 padd，最后安装成功" "[ $rc -eq 0 ] && grep -q 'robot.relax' $R/robotd-calls.log && grep -q '运动模式控制中' $R/install.log"
ok "B 顺序：stop robotd → stop padd → daemon-reload → start robotd → restart padd" "\"$PY\" -c \"
import sys
log=open('$R/systemctl.log').read().splitlines()
want=['stop robotd.service','stop padd.service','daemon-reload','start robotd.service','restart padd.service']
idx=[next(i for i,l in enumerate(log) if l==w) for w in want]
assert idx==sorted(idx),(idx,log)\""
ok "B 启动后核对 busStatus、padd 新进程开始 driving" "grep -q 'busStatus：' $R/install.log && grep -q '已开始 driving' $R/install.log"
ok "B gamesir-xboxd 是 enabled → 一并 start；从不 enable" "grep -qx 'start gamesir-xboxd.service' $R/systemctl.log && ! grep -q '^enable' $R/systemctl.log"
: > "$R/systemctl.log"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
ok "B 手动撤回：停 robotd 和 padd、删 drop-in、重启服务、目录回原样" "[ $rc -eq 0 ] && grep -qx 'stop padd.service' $R/systemctl.log && grep -qx 'restart padd.service' $R/systemctl.log && [ ! -e $R/systemd/padd.service.d/zzzz-replica-netpad.conf ] && [ \"\$(state)\" = $orig ]"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true, "settles": false}'
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B 卸力后恢复一直没完成 → 拒绝，robotd 没停、没备份、什么都没改" "[ $rc -ne 0 ] && ! grep -q '^stop robotd' $R/systemctl.log && ! ls -d $R/imu-module-backup-* >/dev/null 2>&1 && [ \"\$(state)\" = $orig ]"

for scenario in '{"mode": "motion", "phase": "waiting_for_bus"}' '{"mode": "commissioning", "phase": "waiting_for_bus"}'; do
    board b; fake_before "$scenario"
    bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
    ok "B $(echo "$scenario" | tr -d '{}"') → 只发一次卸力请求 + 警告，照样停 robotd 并装好" "[ $rc -eq 0 ] && [ \$(grep -c robot.relax $R/robotd-calls.log) -eq 1 ] && grep -q '扭矩状态确认不了' $R/install.log && grep -qx 'stop robotd.service' $R/systemctl.log"
done

board b; echo activating > "$R/units/robotd.active"; echo disabled > "$R/units/gamesir-xboxd.enabled"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B robotd 在自动重启（activating）、socket 不通 → 警告后照样 stop robotd，装好" "[ $rc -eq 0 ] && grep -q 'unreachable' $R/install.log && grep -qx 'stop robotd.service' $R/systemctl.log"
ok "B gamesir-xboxd 是 disabled → 不碰它" "! grep -q 'gamesir-xboxd.service' <(grep -v '^is-enabled\\|^is-active' $R/systemctl.log)"

board b; orig=$(state); echo '{"mode": "motion", "phase": "control", "servo_parameters_key": false}' > "$R/robotd-after.json"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B 起来的 robotd 没有 servo_parameters（不是可调版）→ 失败并自动撤回" "[ $rc -ne 0 ] && grep -q '不是可调版 robotd' $R/install.log && grep -q '已撤回，板子回到安装前的样子' $R/install.log && [ \"\$(state)\" = $orig ]"

board b; orig=$(state); echo '{"mode": "motion", "phase": "control", "imu_source": "bus"}' > "$R/robotd-after.json"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B 起来的 robotd 没用模块 IMU（imu_source=bus）→ 失败并自动撤回" "[ $rc -ne 0 ] && grep -q 'imu_source bus 不是 bno08x' $R/install.log && [ \"\$(state)\" = $orig ]"

board b; orig=$(state); rm -f "$R/padd-driving"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B padd 新进程 10 秒内没有 driving → 失败并自动撤回" "[ $rc -ne 0 ] && grep -q 'padd 10 秒内没有开始 driving' $R/install.log && [ \"\$(state)\" = $orig ]"

board b; orig=$(state); echo '{"mode": "motion", "phase": "control", "startup_ok": false}' > "$R/robotd-after.json"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B 四步启动不完成 → 失败，文件撤回（服务起不来如实报告）" "[ $rc -ne 0 ] && grep -q '四步启动没完成' $R/install.log && [ \"\$(state)\" = $orig ] && grep -q '文件已恢复，但服务没起来' $R/install.log"

board b
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B 2>&1); BK=$(backup_of "$out")
printf '#!/bin/bash\nfor a; do last=$a; done\n[ "$(basename "$last")" = robotd ] && exit 1\nexec /bin/mv "$@"\n' > "$R/bin/mv"; chmod +x "$R/bin/mv"
: > "$R/systemctl.log"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
rm -f "$R/bin/mv"
ok "B 撤回时复制失败 → 报错、不启动 robotd" "[ $rc -ne 0 ] && grep -q '复制备份失败' $R/install.log && ! grep -q '^start robotd' $R/systemctl.log"

board a; orig=$(state)
bash "$SCRIPT" rollback "" >/dev/null 2>&1; rc1=$?
bash "$SCRIPT" rollback "$R/opt" >/dev/null 2>&1; rc2=$?
ok "rollback 参数为空或不是备份目录 → 拒绝（不会把 /opt 当备份），什么都没改" "[ $rc1 -ne 0 ] && [ $rc2 -ne 0 ] && [ \"\$(state)\" = $orig ]"

stop_fake
echo "== $PASS passed, $FAIL failed, $SKIP skipped (work dir $W)"
[ "$FAIL" = 0 ]
