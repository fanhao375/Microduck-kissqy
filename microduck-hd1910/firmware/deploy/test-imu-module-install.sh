#!/usr/bin/env bash
# Added 2026-10 by fanhao375 (microduck-replica): fake-board tests for imu-module-install.sh; BNO08x gyro-integrated attitude; Apache-2.0 like the rest of this workspace.
#
# Runs the installer against fake boards in a temporary directory. It never touches the real
# systemd, robotd or /etc: systemctl / journalctl are stubs on PATH, robotd is a small fake that
# answers busStatus / relax / startupSubscribe on a UNIX socket in the temporary directory.
#
#   sudo bash deploy/test-imu-module-install.sh <release dir> <kissqy V167 packaging dir> <IMU-only robotd>
#     <release dir>                robotd, robotctl, padd (aarch64) built from this branch (cargo board)
#     <kissqy V167 packaging dir>  packaging/ of a kissqy 78bef11 checkout after prepare-V167.py: the
#                                  installed V167 the fake board starts from
#     <IMU-only robotd>            a robotd from the IMU-only build (hd1910-v167-trunk-imu-i2c): most fake
#                                  boards start with it installed, as a duck that already runs the IMU
#                                  module does; as a source it must be refused
#   PY=<python 3.11+> picks the interpreter the installer and the checks run with (default python3).
# Needs Linux, root (the installer uses install -o root), sha256sum, comm, cmp.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
SCRIPT=$HERE/deploy/imu-module-install.sh
RELEASE=${1:?release dir} KISSQY=${2:?kissqy packaging dir} IMU_ONLY=${3:?IMU-only robotd}
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
      if [ "$u" = robotd ]; then
        for p in $(cat "$R/units/consists" 2>/dev/null); do echo inactive > "$R/units/${p%.service}.active"; done
        if [ -f "$R/units/robotd.pid" ]; then
          kill "$(cat "$R/units/robotd.pid")" 2>/dev/null; rm -f "$R/units/robotd.pid" "$R/robotd.sock"; fi
      fi
    done ;;
  start|restart)
    for u in "${@:2}"; do u=${u%.service}
      case $u in
        # 9>&-: systemd would not hand robotd the installer's lock descriptor; a child of this stub must not either.
        robotd) [ -f "$R/units/robotd.pid" ] || { python3 "$R/fake_robotd.py" "$R/robotd.sock" "$R/robotd-after.json" >>"$R/fake-robotd.log" 2>&1 9>&- & echo $! > "$R/units/robotd.pid"; } ;;
        padd) echo $((RANDOM + 1000)) > "$R/units/padd.pid" ;;
      esac
      echo active > "$R/units/$u.active"
    done ;;
  show)
    prop=; while [ $# -gt 0 ]; do [ "$1" = -p ] && prop=$2; shift; done
    case $prop in
      MainPID) cat "$R/units/padd.pid" 2>/dev/null || echo 0 ;;
      ConsistsOf) cat "$R/units/consists" 2>/dev/null || echo ;;
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
import json, os, socketserver, sys, threading, time
path, scenario = sys.argv[1], json.load(open(sys.argv[2]))
slowed = []
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
                           'hd1910': hd,
                           'imu_reader': {'state': scenario.get('imu_reader_state', 'streaming'), 'last_fault': None}}
        return r

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            request = json.loads(line); method = request.get('method')
            with open(calls, 'a') as f: f.write(method + '\n')
            if method == 'robot.busStatus':
                if scenario.get('hang'): time.sleep(30)                      # never answers in time
                if scenario.get('slow_first') and not slowed:
                    slowed.append(1); time.sleep(2.5)                         # one answer after 2.5 s
                result = report()
            elif method == 'robot.relax':
                # V167: torque goes OFF first, the HD1910 registers are restored after that.
                with lock: state['torque'] = False
                if scenario.get('settles', True):
                    def settle():
                        with lock: state['restore'] = state['applied'] = False
                    threading.Timer(scenario.get('restore_delay', 0.6), settle).start()
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

board() {   # a fresh fake board at $R; $1 = a (NO_SYSTEMD) | b (stubbed systemd + fake robotd);
            # $2 = original: kissqy's robotd installed instead of the IMU-only one
    stop_fake
    unset I2C_FAKE   # a fake I2C bus is per board (fakebus)
    R=$W/board-$((++BOARDS)); mkdir -p "$R"/{opt,etc,src,systemd,bin,units}
    cp -a "$KISSQY/." "$R/opt/"
    if [ "${2:-}" != original ]; then
        cp "$IMU_ONLY" "$R/opt/robotd"
        sed -i "s/^[0-9a-f]\{64\}  robotd\$/$(sha256sum "$R/opt/robotd" | cut -c1-64)  robotd/" "$R/opt/SHA256SUMS"
    fi
    cp "$RELEASE/robotd" "$RELEASE/robotctl" "$RELEASE/padd" "$R/src/"
    for f in calibration-config.py rebuild-config.py runtime.py voice-setup.py service.sh; do cp "$HERE/packaging/$f" "$R/src/"; done
    printf '["bus"]\n"protocol" = "feetech"\n\n["policy"]\n"action_scale" = 0.7\n' > "$R/etc/robotd.toml"; cp "$R/etc/robotd.toml" "$R/etc/robotd.saved.toml"
    printf '[imu_mounts.bno08x]\nquat = [0.459554518, 0.539556409, 0.532044137, 0.463268349]\nverified = true\n' > "$R/etc/hd1910-calibration.toml"
    cp "$R/etc/hd1910-calibration.toml" "$R/etc/hd1910-calibration.saved.toml"
    write_fakes
    echo '{"mode": "motion", "phase": "control"}' > "$R/robotd-after.json"
    touch "$R/padd-driving"
    export INSTALL_DIR=$R/opt CONFIG_DIR=$R/etc BACKUP_ROOT=$R LOG=$R/install.log SYSTEMD_DIR=$R/systemd \
        ROBOTD_SOCKET=$R/robotd.sock READY_TIMEOUT=8 LOCK_FILE=$R/lock PATH=$R/bin:$BASE_PATH
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

# The installer on a pty of its own as session leader, like an ssh login; "<text>@<file>" closes the
# master once <file> contains <text>, as sshd does when the connection drops.
cat > "$W/ptyrun.py" <<'EOF'
import os, pty, select, sys, time
text, path = sys.argv[1].split('@', 1)
pid, fd = pty.fork()
if pid == 0:
    os.execvp(sys.argv[2], sys.argv[2:])
closed = False; deadline = time.time() + 240
while time.time() < deadline:
    done, status = os.waitpid(pid, os.WNOHANG)
    if done: sys.exit(os.waitstatus_to_exitcode(status))
    if not closed:
        if select.select([fd], [], [], 0.05)[0]:
            try: os.read(fd, 65536)
            except OSError: pass
        try:
            if text in open(path, encoding='utf-8', errors='replace').read(): os.close(fd); closed = True
        except OSError: pass
    else:
        time.sleep(0.05)
os.kill(pid, 9); sys.exit(124)
EOF
hangup_after() { "$PY" "$W/ptyrun.py" "$1@$R/install.log" bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B; }
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
# The operator picks the fusion and a raw log (one key quoted, one bare: both spellings must be
# recognised); a later install rewrites the wiring and must keep both, in both files.
for f in robotd.toml robotd.saved.toml; do
    sed -i 's/^"i2c_address" = 75$/&\n"bno08x_fusion" = "gyro"\nbno08x_raw_log = "\/run\/bno08x-raw.csv"/' "$R/etc/$f"
done
sleep 1; bash "$SCRIPT" install "$R/src" lsm6dsv16x /dev/i2c-3 0x6B >/dev/null 2>&1
ok "再装一次：trunk_imu 被替换成 lsm6dsv16x 0x6B 且只有一段" "\"$PY\" -c \"import tomllib;t=open('$R/etc/robotd.toml').read();d=tomllib.loads(t);assert d['trunk_imu']['source']=='lsm6dsv16x' and d['trunk_imu']['i2c_address']==107 and t.count('trunk_imu')==1\""
for f in robotd.toml robotd.saved.toml; do
    ok "再装一次：$f 的 bno08x_fusion / bno08x_raw_log 保留" "toml x \"t=d['trunk_imu'];assert (t['bno08x_fusion'],t['bno08x_raw_log'],t['i2c_device'])==('gyro','/run/bno08x-raw.csv','/dev/i2c-3'),t\" $R/etc/$f"
done
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
cp "$R/src/service.sh" "$R/badsrc/service.sh"; cp "$IMU_ONLY" "$R/badsrc/robotd"
bash "$SCRIPT" install "$R/badsrc" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
ok "只带 IMU、不可调的 robotd → 拒绝" "[ $rc -ne 0 ] && [ \"\$(state)\" = $orig ] && grep -q '不是可调版' $R/install.log"

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
ok "B 控制中：卸力后等扭矩先关、寄存器随后恢复，再停 robotd，最后安装成功" "[ $rc -eq 0 ] && grep -q 'robot.relax' $R/robotd-calls.log && grep -q '运动模式控制中' $R/install.log && grep -q 'torque OFF, restore finished' $R/install.log"
ok "B 顺序：stop padd → stop robotd → daemon-reload → start robotd → restart padd" "\"$PY\" -c \"
import sys
log=open('$R/systemctl.log').read().splitlines()
want=['stop padd.service','stop robotd.service','daemon-reload','start robotd.service','restart padd.service']
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
ok "B 卸力后恢复一直没完成 → 拒绝，robotd 没停、没备份、什么都没改，停掉的 padd 重新起来" "[ $rc -ne 0 ] && ! grep -q '^stop robotd' $R/systemctl.log && ! ls -d $R/imu-module-backup-* >/dev/null 2>&1 && [ \"\$(state)\" = $orig ] && grep -qx 'restart padd.service' $R/systemctl.log && grep -q '先断舵机电' $R/install.log"

for scenario in '{"mode": "motion", "phase": "waiting_for_bus"}' '{"mode": "commissioning", "phase": "waiting_for_bus"}'; do
    board b; fake_before "$scenario"
    bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
    ok "B $(echo "$scenario" | tr -d '{}"') → 只发一次卸力请求 + 警告，照样停 robotd 并装好" "[ $rc -eq 0 ] && [ \$(grep -c robot.relax $R/robotd-calls.log) -eq 1 ] && grep -q '扭矩状态确认不了' $R/install.log && grep -qx 'stop robotd.service' $R/systemctl.log"
done

board b; echo activating > "$R/units/robotd.active"; echo disabled > "$R/units/gamesir-xboxd.enabled"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B robotd 在自动重启（activating）、socket 不存在 → 算没在运行、不卸力，照样 stop robotd，装好" "[ $rc -eq 0 ] && grep -q 'robotd 没在运行（not-running' $R/install.log && grep -qx 'stop robotd.service' $R/systemctl.log"
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

# --- round-2 review cases -------------------------------------------------------------------------
board b
fake_before '{"mode": "motion", "phase": "control", "armed": true, "slow_first": true}'
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B 第一次 busStatus 慢 2.5 秒 → 再问一次，按控制中卸力确认，装好" "[ $rc -eq 0 ] && grep -q '运动模式控制中' $R/install.log && grep -q 'robot.relax' $R/robotd-calls.log"

board b
fake_before '{"mode": "motion", "phase": "control", "armed": true, "hang": true}'
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B busStatus 8 秒都不回答 → 扭矩状态未知：卸力请求 + 警告，照样停 robotd，装好" "[ $rc -eq 0 ] && grep -q '8 秒内没回答' $R/install.log && grep -q '扭矩状态确认不了' $R/install.log && grep -q 'robot.relax' $R/robotd-calls.log"

board b
for kind in deployment-contract deployment-execution; do
    row=$(cd "$R/opt" && grep -o "models/[0-9a-f]*/$kind.json" SHA256SUMS | head -1); printf '\n' >> "$R/opt/$row"
done
orig=$(state)
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B 2>&1); rc=$?; BK=$(backup_of "$out")
ok "B 板上预先 2 行不一致（contract + execution，真板就是这样）→ 安装成功并列出两行" "[ $rc -eq 0 ] && [ \$(wc -l < $BK/sha256-p0.txt) -eq 2 ] && grep -q 'deployment-contract.json models/.*deployment-execution.json\|deployment-execution.json models/.*deployment-contract.json' $R/install.log"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
ok "B 同一板子手动撤回成功（两行照旧不一致）" "[ $rc -eq 0 ] && [ \"\$(state)\" = $orig ]"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control"}'
printf '#!/bin/bash\nfor a; do last=$a; done\ncase "$last" in */imu-module-backup-*/opt/) echo "cp: No space left on device" >&2; exit 1;; esac\nexec /bin/cp "$@"\n' > "$R/bin/cp"; chmod +x "$R/bin/cp"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
rm -f "$R/bin/cp"
ok "B 停了 padd、robotd 之后备份失败 → 原来的服务重新启动，文件没动" "[ $rc -ne 0 ] && [ \"\$(cat $R/units/robotd.active)\" = active ] && grep -qx 'restart padd.service' $R/systemctl.log && grep -q '原来的 robotd / padd 已重新启动' $R/install.log && [ \"\$(state)\" = $orig ]"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control"}'
printf '#!/bin/bash\ncase "$1" in +%%Y%%m%%d-%%H%%M%%S) exit 1;; esac\nexec /bin/date "$@"\n' > "$R/bin/date"; chmod +x "$R/bin/date"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
rm -f "$R/bin/date"
ok "B 没有 die 的命令意外失败（set -e）→ ERR 陷阱：报第几行，原来的服务重新启动" "[ $rc -ne 0 ] && grep -q '意外退出' $R/install.log && [ \"\$(cat $R/units/robotd.active)\" = active ] && grep -qx 'restart padd.service' $R/systemctl.log && [ \"\$(state)\" = $orig ]"

board b; orig=$(state); rm -f "$R/padd-driving"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1 & INSTALLER=$!
for _ in $(seq 1 200); do grep -q '四步启动完成' "$R/install.log" 2>/dev/null && break; sleep 0.1; done
kill -TERM "$INSTALLER"; touch "$R/padd-driving"; wait "$INSTALLER"; rc=$?
ok "B 安装中途收到 TERM → 自动撤回并把服务起回来" "[ $rc -ne 0 ] && grep -q '收到中断' $R/install.log && grep -q '已撤回，板子回到安装前的样子' $R/install.log && [ \"\$(state)\" = $orig ]"

board b
echo 'duck-rec.service' > "$R/units/consists"; echo active > "$R/units/duck-rec.active"
fake_before '{"mode": "motion", "phase": "control"}'
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B >/dev/null 2>&1; rc=$?
ok "B 随 robotd 停的单元（duck-rec）：前后都报告，没跟着起来就提示，不代起" "[ $rc -eq 0 ] && grep -q 'duck-rec.service=active' $R/install.log && grep -q 'duck-rec.service 之前在运行' $R/install.log && ! grep -q 'start duck-rec' $R/systemctl.log"

board a; orig=$(state)
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); BK=$(backup_of "$out")
sed -i 's/0.459554518/0.470000000/' "$R/etc/hd1910-calibration.toml" "$R/etc/hd1910-calibration.saved.toml"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
ok "撤回前重新标定过 → 标定保持现在的、提示备份里那份在哪；robotd.toml 恢复前另存" "[ $rc -eq 0 ] && grep -q '0.470000000' $R/etc/hd1910-calibration.toml && grep -q \"hd1910-calibration.toml 跟备份时不一样\" $R/install.log && grep -q trunk_imu $BK/etc-at-rollback/robotd.toml && ! grep -q trunk_imu $R/etc/robotd.toml && [ \"\$(state)\" = $orig ]"

board a
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); BK=$(backup_of "$out")
OLD=$R/imu-module-backup-20261014-202402; mkdir -p "$OLD/opt" "$OLD/etc"
for f in robotd robotctl calibration-config.py rebuild-config.py runtime.py voice-setup.py SHA256SUMS ELF-CHECK.json BUILD.json; do cp -a "$BK/opt/$f" "$OLD/opt/"; done
cp -a "$BK"/etc/robotd*.toml "$BK"/etc/hd1910*.toml "$OLD/etc/"
bash "$SCRIPT" rollback "$OLD" >/dev/null 2>&1; rc1=$?
bash "$SCRIPT" rollback --old-backup "$OLD" >/dev/null 2>&1; rc2=$?
ok "第一版脚本的旧备份：默认拒绝；加 --old-backup 才撤，并提示 padd、service.sh 不在备份里" "[ $rc1 -ne 0 ] && grep -q '旧备份' $R/install.log && [ $rc2 -eq 0 ] && grep -q '备份里没有 padd' $R/install.log && cmp -s $R/opt/robotd $BK/opt/robotd"

board a original
out=$(bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B 2>&1); BK=$(backup_of "$out")
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc1=$?
sed -i '/imu_mounts/,$d' "$R/etc/hd1910-calibration.toml" "$R/etc/hd1910-calibration.saved.toml"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc2=$?
ok "备份里是 kissqy 原版 robotd 而标定有 [imu_mounts] → 拒绝；删掉 imu_mounts 后才撤" "[ $rc1 -ne 0 ] && grep -q '读不了现在 hd1910-calibration.toml 里的 \[imu_mounts\]' $R/install.log && [ $rc2 -eq 0 ] && cmp -s $R/opt/robotd $KISSQY/robotd"

board a; orig=$(state)
flock "$R/lock" sleep 5 & HOLDER=$!; sleep 0.3
bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 0x4B >/dev/null 2>&1; rc=$?
kill $HOLDER 2>/dev/null; wait $HOLDER 2>/dev/null
ok "另一个服务管理操作占着锁 → 拒绝，什么都没改" "[ $rc -ne 0 ] && grep -q '已有服务管理操作在进行' $R/install.log && [ \"\$(state)\" = $orig ]"

# --- review C: the ssh session drops (HUP ignored, the terminal gone) ------------------------------
board b
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
printf '#!/bin/bash\nsleep 0.5\nexec /usr/bin/install "$@"\n' > "$R/bin/install"; chmod +x "$R/bin/install"
hangup_after '已备份到'; rc=$?; rm -f "$R/bin/install"
ok "B 备份后 ssh 断线（换文件中途）→ 照样装完：服务都起来、没有意外退出" "[ $rc -eq 0 ] && ! grep -q '意外退出' $R/install.log && [ \"\$(cat $R/units/robotd.active)\" = active ] && grep -qx 'restart padd.service' $R/systemctl.log && cmp -s $R/opt/robotd $R/src/robotd"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
echo '{"mode": "motion", "phase": "control", "startup_ok": false}' > "$R/robotd-after.json"
hangup_after '已备份到'; rc=$?
ok "B 备份后断线、随后启动失败 → 自动撤回照样做完，文件回原样" "[ $rc -ne 0 ] && ! grep -q '意外退出' $R/install.log && grep -q '自动撤回到' $R/install.log && grep -q '文件已恢复\|已撤回' $R/install.log && [ \"\$(state)\" = $orig ]"

board b
fake_before '{"mode": "motion", "phase": "control", "armed": true, "restore_delay": 3}'
hangup_after '运动模式控制中'; rc=$?
ok "B 等卸力时断线（padd 已停）→ 照样装完，服务都起来" "[ $rc -eq 0 ] && ! grep -q '意外退出' $R/install.log && [ \"\$(cat $R/units/robotd.active)\" = active ] && [ \"\$(cat $R/units/padd.active)\" = active ]"

# --- auto / detect: the module found on a fake I2C bus (I2C_FAKE, read by probe_i2c's FakeDev) ----
fakebus() { printf '%s\n' "$1" > "$R/bus.json"; export I2C_FAKE=$R/bus.json; }
C1='"0x19": {"0x00": "0x1e"}, "0x68": {"0x00": "0x0f"}'   # the C1 HAT's BMI088, both halves
L6A='"0x6a": {"0x0f": "0x70"}' L6B='"0x6b": {"0x0f": "0x70"}' BNO='"raw": {"0x4b": "14010000"}'
trunk_is() { toml x "t=d.get('trunk_imu') or {};assert (t.get('source'),t.get('i2c_device'),t.get('i2c_address'))==($1),t" "$R/etc/$2"; }
untouched() { [ "$(state)" = "$1" ] && ! grep -q trunk_imu "$R/etc/robotd.toml" && ! ls -d "$R"/imu-module-backup-* >/dev/null 2>&1; }
auto_ok() {   # name, bus JSON, expected "'source','device',address"
    board a; fakebus "$2"
    bash "$SCRIPT" install "$R/src" auto >"$R/out.txt" 2>&1; rc=$?
    ok "auto $1 → 装成 $3" "[ $rc -eq 0 ] && trunk_is \"$3\" robotd.toml && trunk_is \"$3\" robotd.saved.toml && grep -q '按识别到的装' $R/install.log && grep -q '测试模式：总线是假的' $R/install.log"
}
auto_refused() {   # name, bus JSON, text the log must contain
    board a; fakebus "$2"; local before; before=$(state)
    bash "$SCRIPT" install "$R/src" auto >"$R/out.txt" 2>&1; rc=$?
    ok "auto $1 → 拒绝，什么都没改" "[ $rc -ne 0 ] && untouched $before && grep -qF \"$3\" $R/install.log"
}

auto_ok 'J5：C1 + 0x6B' '{"/dev/i2c-3": {"busy": ["0x18"], "regs": {'"$C1, $L6B"'}}, "/dev/i2c-4": {}}' "'lsm6dsv16x','/dev/i2c-3',107"
ok "auto 打印识别到的模块（地址十六进制）、接法没变就不警告" "grep -q 'IMU 模块（自动识别）：lsm6dsv16x /dev/i2c-3 0x6b' $R/out.txt && grep -q '按识别到的装：lsm6dsv16x /dev/i2c-3 0x6b' $R/install.log && ! grep -q '接法变了' $R/install.log"
ok "auto 进度和每个地址的读数写到终端（log 的终端输出修好了）" "grep -q '\[detect\] /dev/i2c-3 0x6b WHO_AM_I：0x70 / 0x70' $R/out.txt"
auto_ok 'J5：C1 + 0x6A' '{"/dev/i2c-3": {"regs": {'"$C1, $L6A"'}}}' "'lsm6dsv16x','/dev/i2c-3',106"
auto_ok 'J5：D1 板载 0x6A + 模块 0x6B' '{"/dev/i2c-3": {"regs": {'"$L6A, $L6B"'}}, "/dev/i2c-4": {}}' "'lsm6dsv16x','/dev/i2c-3',107"
ok "auto D1：日志写明 0x6a 当头部 IMU 跳过" "grep -q '0x6a 当作 D1 版 HAT 板载的头部 IMU 跳过' $R/install.log"
auto_ok '27/28：BNO085 0x4B' '{"/dev/i2c-4": {'"$BNO"'}}' "'bno08x','/dev/i2c-4',75"
auto_ok '27/28：LSM6DSV16X 0x6A' '{"/dev/i2c-4": {"regs": {'"$L6A"'}}, "/dev/i2c-3": {"regs": {'"$C1"'}}}' "'lsm6dsv16x','/dev/i2c-4',106"
auto_ok 'J5：C1 + BNO 0x4A' '{"/dev/i2c-3": {"regs": {'"$C1"'}, "raw": {"0x4a": "00000000"}}}' "'bno08x','/dev/i2c-3',74"

auto_refused 'J5 只有 D1 板载 0x6A' '{"/dev/i2c-3": {"regs": {'"$L6A"'}}, "/dev/i2c-4": {}}' 'D1 版 HAT 板载的头部 IMU，不是躯干模块'
ok "auto D1 只有板载：提示 D1 上模块必须 0x6B" "grep -q '躯干模块必须设成 0x6B' $R/install.log"
auto_refused 'J5 只有 0x6B、没认出 C1（R30 没焊的 D1）' '{"/dev/i2c-3": {"regs": {'"$L6B"'}}}' '分不清它是躯干模块还是头部 IMU'
auto_refused '半个 C1（0x68 没应答）+ 0x6A' '{"/dev/i2c-3": {"regs": {"0x19": {"0x00": "0x1e"}, '"$L6A"'}}}' '没认出 C1 版 HAT'
auto_refused 'BMI088 两个地址 EBUSY + 0x6A' '{"/dev/i2c-3": {"regs": {'"$C1, $L6A"'}, "busy": ["0x19", "0x68"]}}' '没认出 C1 版 HAT'
auto_refused 'C1 + 0x6A + 0x6B' '{"/dev/i2c-3": {"regs": {'"$C1, $L6A, $L6B"'}}}' '找到 2 颗 IMU 模块'
auto_refused 'BNO（27/28）+ D1 + 模块 0x6B' '{"/dev/i2c-4": {'"$BNO"'}, "/dev/i2c-3": {"regs": {'"$L6A, $L6B"'}}}' '找到 2 颗 IMU 模块'
ok "auto 多颗：只给真候选的命令（0x6b、0x4b），跳过的 0x6a 没有命令" "grep -q 'install $R/src lsm6dsv16x /dev/i2c-3 0x6b' $R/install.log && grep -q 'install $R/src bno08x /dev/i2c-4 0x4b' $R/install.log && ! grep -q 'install $R/src lsm6dsv16x /dev/i2c-3 0x6a' $R/install.log"
auto_refused '两条线都不存在' '{}' '都不存在'
auto_refused '两条线都空' '{"/dev/i2c-3": {}, "/dev/i2c-4": {}}' '两条线上都没找到躯干 IMU 模块'
auto_refused '27/28 被 python3 占着、J5 上 C1 + 0x6B' '{"/dev/i2c-4": {"held": {"by": "python3"}}, "/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}}' '被 python3 占用'
auto_refused '27/28 打不开（EACCES）、J5 上 C1 + 0x6B' '{"/dev/i2c-4": {"open_error": "EACCES"}, "/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}}' '打不开（EACCES）'
auto_refused '0x4B ETIMEDOUT、J5 上 C1 + 0x6B' '{"/dev/i2c-4": {"errors": {"0x4b": "ETIMEDOUT"}}, "/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}}' 'ETIMEDOUT'
auto_refused '0x6B EIO（不当没有）、27/28 上 BNO' '{"/dev/i2c-3": {"regs": {'"$C1"'}, "errors": {"0x6b": "EIO"}}, "/dev/i2c-4": {'"$BNO"'}}' '读出错：出错（EIO）'
auto_refused '0x6B WHO_AM_I 读到 0x6c' '{"/dev/i2c-3": {"regs": {'"$C1"', "0x6b": {"0x0f": "0x6c"}}}}' 'WHO_AM_I 不是 0x70'
auto_refused 'BNO 头通道 9' '{"/dev/i2c-4": {"raw": {"0x4b": "14000900"}}}' '不像 BNO08x'
auto_refused '0x4B 两次读数不一致（一次应答一次没有）' '{"/dev/i2c-4": {'"$BNO"', "flaky": {"0x4b": ["ok", "ENXIO"]}}}' '两次读得不一样'

board a; before=$(state)
fakebus '{"_ops_log": "'"$R"'/ops.log", "/dev/i2c-3": {"regs": {'"$C1, $L6B"'}, "busy": ["0x6b"]}}'
bash "$SCRIPT" install "$R/src" auto >/dev/null 2>&1; rc=$?
ok "auto 候选地址 0x6b EBUSY → 拒绝；先 I2C_SLAVE 拿到 EBUSY，这个地址一次读都没做" "[ $rc -ne 0 ] && untouched $before && grep -q '被内核驱动占着' $R/install.log && grep -q '/dev/i2c-3 SLAVE 0x6b' $R/ops.log && ! grep -q '/dev/i2c-3 \(RDWR\|READ\) 0x6b' $R/ops.log"

board a; before=$(state)
bash "$SCRIPT" install "$R/src" auto extra >/dev/null 2>&1; rc1=$?
bash "$SCRIPT" install "$R/src" >/dev/null 2>&1; rc2=$?
bash "$SCRIPT" install "$R/src" bno08x /dev/i2c-4 >/dev/null 2>&1; rc3=$?
ok "用法：auto 多一个参数 / 只给目录 / 显式少地址 → 用法错，什么都没改" "[ $rc1 -ne 0 ] && [ $rc2 -ne 0 ] && [ $rc3 -ne 0 ] && untouched $before && [ \$(grep -c '用法：install <文件目录> auto' $R/install.log) -eq 3 ]"

board a; fakebus '{"/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}}'
CONFIG_DIR=/etc/robot/feetech-ft5 INSTALL_DIR=/opt/robot/feetech-ft5-r5 bash "$SCRIPT" detect >/dev/null 2>&1; rc=$?
ok "I2C_FAKE 在真路径上不认（die，退出码 1）" "[ $rc -eq 1 ] && grep -q 'I2C_FAKE 只给假板子测试用' $R/install.log"

detect_rc() {   # bus JSON -> sets rc, out; checks nothing changed
    fakebus "$1"; out=$(bash "$SCRIPT" detect 2>"$R/detect.err"); rc=$?
}
board a; before=$(state)
detect_rc '{"/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}}'
ok "detect 一颗 → 0，打印识别到的（十六进制），每个地址的读数在终端" "[ $rc -eq 0 ] && grep -q '识别到：lsm6dsv16x /dev/i2c-3 0x6b' <<<\"\$out\" && grep -q '/dev/i2c-3 0x19 chip ID：0x1e / 0x1e' $R/detect.err"
detect_rc '{"/dev/i2c-3": {"regs": {'"$C1, $L6A, $L6B"'}}}'
ok "detect 多颗 → 2，列出两条命令" "[ $rc -eq 2 ] && [ \$(grep -c 'install <文件目录> lsm6dsv16x /dev/i2c-3' <<<\"\$out\") -eq 2 ]"
detect_rc '{"/dev/i2c-3": {}, "/dev/i2c-4": {}}'
ok "detect 没有 → 3，提到总线小板不归这个脚本管" "[ $rc -eq 3 ] && grep -q 'ID 200' <<<\"\$out\""
detect_rc '{"/dev/i2c-4": {"held": {"by": "robotd"}}, "/dev/i2c-3": {}}'
ok "detect 没探全（被 robotd 占着）→ 4，说出是谁占着" "[ $rc -eq 4 ] && grep -q '被 robotd 占用' <<<\"\$out\""
ok "detect 从头到尾什么都没改（没备份、没 trunk_imu、安装目录一样）" "untouched $before"
for case in 00000000:0 04000000:0 00100000:0 14800000:0 01100000:4 ffffffff:4 14000600:4; do
    detect_rc '{"/dev/i2c-4": {"raw": {"0x4b": "'"${case%%:*}"'"}}}'
    ok "detect BNO 头 ${case%%:*} → 退出码 ${case##*:}" "[ $rc -eq ${case##*:} ]"
done
detect_rc '{"/dev/i2c-4": {"raw": {"0x4b": "ffffffff"}}}'
ok "BNO 头 ff ff ff ff → 提示芯片可能正在复位" "[ $rc -eq 4 ] && grep -q '可能正在复位' <<<\"\$out\""

board a
for f in robotd.toml robotd.saved.toml; do printf '\n["trunk_imu"]\n"source" = "bno08x"\n"i2c_device" = "/dev/i2c-4"\n"i2c_address" = 75\n' >> "$R/etc/$f"; done
fakebus '{"/dev/i2c-3": {"regs": {'"$C1"'}, "raw": {"0x4a": "14010000"}}, "/dev/i2c-4": {}}'
out=$(bash "$SCRIPT" install "$R/src" auto 2>/dev/null); rc=$?
ok "auto 接法变了（bno08x i2c-4 0x4B → i2c-3 0x4A）而方向是以前确认的 → 装上并醒目提醒重新确认" "[ $rc -eq 0 ] && trunk_is \"'bno08x','/dev/i2c-3',74\" robotd.toml && grep -q 'IMU 的接法变了' $R/install.log && grep -q '^注意：IMU 的接法变了' <<<\"\$out\""

# B: stubbed systemd + fake robotd
board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
echo '{"mode": "motion", "phase": "control", "imu_source": "lsm6dsv16x"}' > "$R/robotd-after.json"
fakebus '{"/dev/i2c-3": {"busy": ["0x18"], "regs": {'"$C1, $L6B"'}}, "/dev/i2c-4": {}}'
out=$(bash "$SCRIPT" install "$R/src" auto 2>&1); rc=$?; BK=$(backup_of "$out")
ok "B auto 控制中：先卸力确认、停 robotd，再识别，之后才备份；装成 lsm6dsv16x i2c-3 0x6B" "[ $rc -eq 0 ] && grep -q 'torque OFF, restore finished' $R/install.log && trunk_is \"'lsm6dsv16x','/dev/i2c-3',107\" robotd.toml && \"$PY\" -c \"
log=open('$R/install.log',encoding='utf-8').read().splitlines()
at=lambda s: next(i for i,l in enumerate(log) if s in l)
order=[at('停服务之前：结论 one'),at('robotd 已停止'),at('停了 robotd 之后：结论 one'),at('按识别到的装'),at('已备份到')]
assert order==sorted(order),order\""
ok "B auto 启动核对等到 IMU 读取 streaming，imu_source = 识别出的型号" "grep -q '\"imu_source\": \"lsm6dsv16x\".*\"imu_reader\": \"streaming\"' $R/install.log"
: > "$R/systemctl.log"
bash "$SCRIPT" rollback "$BK" >/dev/null 2>&1; rc=$?
ok "B auto 装完手动撤回 → 目录回原样、没有 trunk_imu" "[ $rc -eq 0 ] && [ \"\$(state)\" = $orig ] && ! grep -q trunk_imu $R/etc/robotd.toml"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
fakebus '{"/dev/i2c-3": {"regs": {'"$L6A"'}}, "/dev/i2c-4": {}}'
bash "$SCRIPT" install "$R/src" auto >/dev/null 2>&1; rc=$?
ok "B auto 停服务之前就明摆着没有 → 拒绝，一个服务都没停、没卸力" "[ $rc -ne 0 ] && untouched $orig && ! grep -q '^stop' $R/systemctl.log && ! grep -q robot.relax $R/robotd-calls.log && [ \"\$(cat $R/units/robotd.active)\" = active ]"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
fakebus '{"/dev/i2c-4": {"held": {"by": "robotd", "while_file": "'"$R"'/units/robotd.active"}, '"$BNO"'}, "/dev/i2c-3": {"busy": ["0x18"], "regs": {'"$C1"'}}}'
out=$(bash "$SCRIPT" install "$R/src" auto 2>&1); rc=$?
ok "B auto 27/28 被运行中的 robotd 占着 → 先卸力停 robotd，再探到 BNO 并装上" "[ $rc -eq 0 ] && grep -q '被 robotd 占用' $R/install.log && grep -q '停服务之前：结论 unsure' $R/install.log && grep -q '停了 robotd 之后：结论 one' $R/install.log && trunk_is \"'bno08x','/dev/i2c-4',75\" robotd.toml && grep -qx 'stop robotd.service' $R/systemctl.log"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
fakebus '{"/dev/i2c-4": {"held": {"by": "robotd", "while_file": "'"$R"'/units/robotd.active"}}, "/dev/i2c-3": {"regs": {'"$L6A"'}}}'
bash "$SCRIPT" install "$R/src" auto >/dev/null 2>&1; rc=$?
ok "B auto 停了 robotd 再探还是没有 → 拒绝，原来的 robotd / padd 按原样起回来，文件没动" "[ $rc -ne 0 ] && untouched $orig && grep -q '停了 robotd 之后：结论 none' $R/install.log && grep -q '原来的 robotd / padd 已重新启动' $R/install.log && [ \"\$(cat $R/units/robotd.active)\" = active ] && grep -qx 'restart padd.service' $R/systemctl.log"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
echo '{"mode": "motion", "phase": "control", "imu_source": "lsm6dsv16x", "imu_reader_state": "opening"}' > "$R/robotd-after.json"
fakebus '{"/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}}'
bash "$SCRIPT" install "$R/src" auto >/dev/null 2>&1; rc=$?
ok "B auto 装完 IMU 读取一直不是 streaming（15 秒）→ 失败并自动撤回" "[ $rc -ne 0 ] && grep -q 'IMU 读取 opening，不是 streaming' $R/install.log && grep -q '已撤回，板子回到安装前的样子' $R/install.log && [ \"\$(state)\" = $orig ]"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4C >/dev/null 2>&1; rc=$?
ok "B 显式写法地址 0x4C → 卸力之前就拒绝（来源检查没被挪到停服务之后）" "[ $rc -ne 0 ] && ! grep -q '^stop' $R/systemctl.log && ! grep -q robot.relax $R/robotd-calls.log && [ \"\$(state)\" = $orig ]"
unset I2C_FAKE

# --- the output is a pipe whose reader is gone (ssh without -t killed, | tee killed): SIGPIPE ------
board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
printf '#!/bin/bash\nsleep 0.5\nexec /usr/bin/install "$@"\n' > "$R/bin/install"; chmod +x "$R/bin/install"
bash "$SCRIPT" install "$R/src" bno08x /dev/null 0x4B 2>&1 | head -c 200 >/dev/null; rc=${PIPESTATUS[0]}
rm -f "$R/bin/install"
ok "B 输出管道的读端提前退出（显式写法）→ 照样装完：服务都起来、没有意外退出" "[ $rc -eq 0 ] && ! grep -q '意外退出' $R/install.log && cmp -s $R/opt/robotd $R/src/robotd && [ \"\$(cat $R/units/robotd.active)\" = active ] && grep -qx 'restart padd.service' $R/systemctl.log"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
echo '{"mode": "motion", "phase": "control", "imu_source": "lsm6dsv16x"}' > "$R/robotd-after.json"
fakebus '{"/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}, "/dev/i2c-4": {}}'
printf '#!/bin/bash\nsleep 0.5\nexec /usr/bin/install "$@"\n' > "$R/bin/install"; chmod +x "$R/bin/install"
bash "$SCRIPT" install "$R/src" auto 2>&1 | head -c 200 >/dev/null; rc=${PIPESTATUS[0]}
rm -f "$R/bin/install"
ok "B 输出管道的读端提前退出（auto）→ 照样装完" "[ $rc -eq 0 ] && ! grep -q '意外退出' $R/install.log && trunk_is \"'lsm6dsv16x','/dev/i2c-3',107\" robotd.toml && [ \"\$(cat $R/units/robotd.active)\" = active ] && grep -qx 'restart padd.service' $R/systemctl.log"

board b; orig=$(state)
fake_before '{"mode": "motion", "phase": "control", "armed": true}'
fakebus '{"/dev/i2c-4": {"held": {"by": "robotd", "while_file": "'"$R"'/units/robotd.active"}}, "/dev/i2c-3": {"regs": {'"$L6A"'}}}'
bash "$SCRIPT" install "$R/src" auto 2>&1 | head -c 200 >/dev/null; rc=${PIPESTATUS[0]}
ok "B 输出管道的读端提前退出、停了 robotd 后识别失败 → 照样把原来的服务起回来" "[ $rc -eq 1 ] && untouched $orig && grep -q '原来的 robotd / padd 已重新启动' $R/install.log && [ \"\$(cat $R/units/robotd.active)\" = active ] && grep -qx 'restart padd.service' $R/systemctl.log"

board a; fakebus '{"/dev/i2c-3": {"regs": {'"$C1, $L6B"'}}}'
bash "$SCRIPT" detect 2>&1 | (exit 0); rc=${PIPESTATUS[0]}
ok "detect 输出管道的读端一开始就没了 → 照样探完，退出码 0，日志写完结论" "[ $rc -eq 0 ] && grep -q 'detect：结论 one' $R/install.log"
unset I2C_FAKE

stop_fake
echo "== $PASS passed, $FAIL failed, $SKIP skipped (work dir $W)"
[ "$FAIL" = 0 ]
