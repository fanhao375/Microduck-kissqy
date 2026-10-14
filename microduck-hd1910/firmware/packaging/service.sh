#!/usr/bin/env bash
# Modified 2026-10 by fanhao375 (microduck-replica): optional servo P/D/torque-limit file and joint filters (servo-parameters, servo-parameters-clear, joint-filters); UDP pad (netpad); see IMU-MODULE.md at the repository root.
# Manages the existing official service. All motor commands go through robotd's socket.
set -Eeuo pipefail
# Root runs installation in a user-owned staging tree; keep it free of root
# bytecode directories so later cleanup does not leave permission errors.
export PYTHONDONTWRITEBYTECODE=1
bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
install_dir=/opt/robot/feetech-ft5-r5
config_dir=/etc/robot/feetech-ft5
params_path=$config_dir/robotd.toml
calibration_path=$config_dir/hd1910-calibration.toml
dropin=/etc/systemd/system/robotd.service.d/zzzz-r17-0151.conf
state_dir=/var/lib/robot/feetech-ft5/r5
socket_path=/run/robotd.sock
temporary_dir=
input_paused=0
input_setup_complete=true
install_phase=idle
service_user=root
service_group=robot
bus_port=/dev/ttyS2

install_log() { printf '[固件安装 / %s] %s · %s\n' "$install_phase" "$1" "$2"; }
install_step() {
    install_phase=$1; local detail=$2; shift 2
    "$@"
    install_log 成功 "$detail"
}
optional_install_step() {
    install_phase=$1; local detail=$2 result; shift 2
    if "$@"; then install_log 成功 "$detail"; else
        result=$?; input_setup_complete=false
        install_log 失败 "$detail；退出码 $result，实际原因见上方；主服务继续独立安装"
    fi
}

fail() { echo "错误：$*" >&2; return 1; }
require_root() { [[ $EUID -eq 0 ]] || fail '请用 sudo bash service.sh 执行。'; }
require_installed() { [[ -x $install_dir/robotd ]] || fail '请先执行 install。'; }
client() { "$install_dir/robotd" --socket "$socket_path" --params "$params_path" "$@"; }
restart_service() {
    systemctl reset-failed robotd.service
    local result=0
    systemctl restart robotd.service || result=$?
    if [[ $result != 0 ]]; then
        echo 'robotd 启动失败；未重建配置或重复启动。实际错误：' >&2
        journalctl -u robotd.service -n 30 --no-pager -o cat >&2 || true
        return "$result"
    fi
    # This observer cannot stop or restart robotd. It receives the existing
    # four-step push events once; systemd itself has no readiness dependency.
    install_phase='等待机器人启动'
    local event_args=()
    [[ ${MICRODUCK_INSTALL_LOG:-} != 1 ]] || event_args=(--install-log)
    if python3 "$install_dir/startup_gate.py" --timeout 60 "${event_args[@]}"; then return 0; fi
    # The failed observer never stops the service; preserve native failure details.
    journalctl -u robotd.service -n 30 --no-pager -o cat >&2 || true
    return 78
}

on_error() {
    local result=$?
    trap - ERR
    if [[ -n $temporary_dir ]]; then rm -rf -- "$temporary_dir"; fi
    if [[ $input_paused == 1 ]]; then
        install_log 失败 "退出码 $result；实际原因见上方；安装未完成，未重新连接数据通道" >&2
    elif [[ ${MICRODUCK_INSTALL_LOG:-} == 1 ]]; then
        install_log 失败 "退出码 $result；实际原因见上方；未改动运行服务" >&2
    fi
    exit "$result"
}
trap on_error ERR

identity() {
    service_user=$(systemctl show robotd.service -p User --value)
    service_group=$(systemctl show robotd.service -p Group --value)
    service_user=${service_user:-root}
    service_group=${service_group:-$(id -gn "$service_user")}
    id "$service_user" >/dev/null
    getent group "$service_group" >/dev/null
}

write_dropin() {
    local mode=$1
    python3 "$install_dir/runtime.py" configure-startup "$mode"
    systemctl daemon-reload
}


# padd, and the GameSir bridge only where it is enabled, once robotd is back. A robotd that took
# longer than startup_gate's 60 s leaves padd exited with 78, which systemd does not restart.
# Nothing disabled is enabled or started here.
start_input_services() {
    systemctl start padd.service
    if [[ $(systemctl is-enabled gamesir-xboxd.service 2>/dev/null || true) == enabled ]]; then
        systemctl start gamesir-xboxd.service
    fi
}

padd_allows_inet() {
    local families
    families=$(systemctl show padd.service -p RestrictAddressFamilies --value)
    if [[ -z $families ]]; then return 0; fi
    if [[ $families == '~'* ]]; then [[ " ${families#\~} " != *" AF_INET "* ]]; return; fi
    [[ " $families " == *" AF_INET "* ]]
}

# Restart padd alone and read what its new process says: "netpad on —" and "driving —" for on,
# "driving —" without "netpad on —" for off. Up to 10 s; robotd keeps running throughout.
restart_padd_checked() {
    local want=$1 port=${2:-} pid log
    systemctl reset-failed padd.service 2>/dev/null || true
    systemctl restart padd.service
    for _ in $(seq 1 40); do
        sleep 0.25
        pid=$(systemctl show padd.service -p MainPID --value)
        [[ -n $pid && $pid != 0 ]] || continue
        log=$(journalctl -u padd.service _PID="$pid" -o cat --no-pager 2>/dev/null || true)
        [[ $log == *'driving —'* ]] || continue
        if [[ $want == on && $log != *'netpad on —'* ]]; then
            echo "padd 新进程 $pid 没有读 UDP（日志里没有 netpad on —）。" >&2
            return 1
        fi
        if [[ $want == off && $log == *'netpad on —'* ]]; then
            echo "padd 新进程 $pid 仍在读 UDP。" >&2
            return 1
        fi
        if [[ $want == on && -n $port ]] && command -v ss >/dev/null; then
            ss -H -ulpn "sport = :$port" | grep -q padd || { echo "UDP 端口 $port 没有被 padd 监听。" >&2; return 1; }
        fi
        echo "padd 新进程 $pid 已按 netpad $want 运行。"
        return 0
    done
    echo 'padd 10 秒内没有进入 driving；最近日志：' >&2
    journalctl -u padd.service -n 20 --no-pager -o cat >&2 || true
    return 1
}

# robotd's four startup steps done? padd waits for them (kissqy's startup_gate), so a netpad check
# against a robotd that is not there yet would blame padd for robotd.
robotd_ready() { python3 "$install_dir/startup_gate.py" --timeout "${1:-1}" >/dev/null 2>&1; }
robotd_state_hint() { if robotd_ready 1; then echo 'robotd 已就绪'; else echo 'robotd 还没完成四步启动'; fi; }

# A netpad change whose padd check failed. on: put back the files this run wrote and restart padd.
# off is never rolled back: it is the way back to the Bluetooth pad.
netpad_failed() {
    local want=$1 why
    why=$(robotd_state_hint)
    if [[ $want == on ]]; then
        python3 "$install_dir/runtime.py" restore-netpad || true
        systemctl restart padd.service || true
        start_input_services || true
        fail "padd 没有按 netpad on 启动（$why）；这次写过的配置已恢复，padd 已重启。"
        return 1   # fail only returns: called after ||, errexit does not stop here
    fi
    python3 "$install_dir/runtime.py" netpad-commit >/dev/null || true
    start_input_services || true
    fail "netpad off 已写入，但 padd 10 秒内没有开始驾驶（$why）；off 不回滚：robotd 就绪后 padd 会按 off 启动，或 sudo bash service.sh reconnect。"
}

verify_servo_loaded() {
    local running
    running=$(systemctl show robotd.service -p ExecStart --value)
    if [[ $running == *--commissioning* ]]; then
        echo '标定模式不读取舵机参数：已保存，切到运动模式后上力时生效。'
        return 0
    fi
    python3 "$install_dir/runtime.py" verify-servo "$1"
}

verify_motion_models() {
    local running
    running=$(systemctl show robotd.service -p ExecStart --value)
    if [[ $running != *--commissioning* ]]; then python3 "$install_dir/runtime.py" verify-models; fi
}


relax_service() {
    python3 "$bundle_dir/runtime.py" relax-confirmed
}

request_relax_service() {
    # Recovery must also work after initialization failed. Request a stop when
    # IPC is reachable, then stop the sole bus owner before reopening the HAT.
    python3 "$bundle_dir/runtime.py" relax-request
}

show_service() {
    systemctl show robotd.service -p ActiveState -p SubState -p MainPID -p ExecStart --no-pager
}

install_bundle() {
    # Requested model replacement: no deployment snapshot, rollback or readiness gate.
    local install_mode=commissioning
    local model_args=()
    [[ ${1:-} != reset-models ]] || model_args=(--replace-models)
    export MICRODUCK_INSTALL_LOG=1
    install_phase=preparing
    identity
    # A valid archive checksum alone cannot detect a library that was already
    # truncated before packaging. Reject invalid ELF ranges before touching
    # the running robot or any input producer; this is installation-only work.
    install_step '检查可执行文件' '校验可执行文件与推理库的完整性、装载范围和 AArch64 架构' python3 "$bundle_dir/elf_integrity.py" "$bundle_dir"
    install_phase='准备本机配置'
    temporary_dir=$(mktemp -d)
    python3 "$bundle_dir/rebuild-config.py" --prepare "$temporary_dir"
    local base_args=(--source "$temporary_dir/robotd.toml")
    local previous_args=()
    if [[ -f $install_dir/robotd-profile.toml ]]; then
        previous_args=(--previous-overrides "$install_dir/robotd-profile.toml")
    fi
    if [[ -n ${2:-} && $2 == --base-params ]]; then
        base_args=(--source "$3")
    fi
    python3 "$bundle_dir/configure.py" "${base_args[@]}" "${previous_args[@]}" "${model_args[@]}" --overrides "$bundle_dir/robotd-profile.toml" --output "$temporary_dir/robotd.toml"
    install_mode=commissioning
    if [[ -f $temporary_dir/hd1910-calibration.toml ]]; then
        if python3 -c 'import sys,tomllib; sys.exit(tomllib.load(open(sys.argv[1],"rb")).get("motion_enabled") is not True)' "$temporary_dir/hd1910-calibration.toml"; then
            install_mode=motion
        fi
    fi
    install_log 成功 '本次安装配置已生成；本机标定与已保存参数已读取'
    # padd is a separate official daemon. Stop both input producers before
    # replacing their executable/code; robotd alone does not stop padd.
    input_paused=1
    # Stopping an absent/already stopped service is optional during replacement.
    # Atomic file installation also avoids truncating running executable inodes.
    optional_install_step '停止旧实体手柄服务' '停止 padd 与 GameSir 输入桥' systemctl stop padd.service gamesir-xboxd.service
    install_phase='请求停止运动'
    request_relax_service || install_log 提示 '松开请求未确认；继续停止旧 robotd 进程'
    install_step '停止旧机器人服务' 'systemd 停止旧 robotd，释放串口与旧 IPC socket' systemctl stop robotd.service
    if [[ $bundle_dir != "$install_dir" ]]; then
        install -d -m 755 "$install_dir"
        install_step '替换固件文件' '逐文件原子替换服务文件' python3 "$bundle_dir/install-files.py" "$bundle_dir" "$install_dir" "${model_args[@]}"
    fi
    chmod 755 "$install_dir/robotd" "$install_dir/service.sh" "$install_dir/configure.py"
    install_phase='安装本机配置'
    install -d -m 755 -o "$service_user" -g "$service_group" "$config_dir"
    # Retire the removed manual P/D/torque override before starting this release.
    rm -f -- "$config_dir/servo-test-parameters.json"
    python3 - "$temporary_dir/robotd.toml" "$params_path" "$install_dir" <<'PYPARAMS'
from pathlib import Path
import sys
sys.path.insert(0,sys.argv[3])
from configure import atomic_write
path=Path(sys.argv[2]);data=Path(sys.argv[1]).read_bytes()
atomic_write(path,data)
atomic_write(path.with_name(path.stem+'.saved'+path.suffix),data)
PYPARAMS
    chown "$service_user:$service_group" "$params_path"
    if [[ -f $temporary_dir/hd1910-calibration.toml ]]; then
        python3 "$install_dir/calibration-config.py" copy "$temporary_dir/hd1910-calibration.toml" --output "$calibration_path"
        chown "$service_user:$service_group" "$calibration_path"
    fi
    install_log 成功 '服务参数与本机标定已安装'
    install_phase='准备音库文件'
    python3 "$install_dir/voice-setup.py" "$params_path" --prepare || install_log 失败 '音库文件未准备好；实际原因见上方；继续启动主服务'
    install_step '安装主服务启动配置' "写入 robotd 启动命令；串口 /dev/ttyS2；模式 $install_mode" python3 "$install_dir/runtime.py" configure-startup "$install_mode"
    install_step '重载服务配置' 'systemd 读取本次安装的服务配置' systemctl daemon-reload
    restart_service
    install_phase='启动机器人服务'
    install_log 成功 'robotd 四步启动完成；控制循环已运行，未请求运动'
    printf '{"robotd_ready":true,"mode":"%s"}\n' "$install_mode"
    # Input configuration belongs after core readiness. A failed input setup
    # has no authority to stop the already running robot service.
    input_setup_complete=true
    install_phase='安装实体手柄配置'
    if python3 "$install_dir/gamepad.py" prepare-all "$install_dir"; then
        optional_install_step '重载手柄配置' 'systemd 读取输入桥与 padd 的服务配置' systemctl daemon-reload
        if [[ $input_setup_complete == true ]]; then
            optional_install_step '启动实体手柄服务' 'robotd 已就绪；启动 GameSir 输入桥与官方 padd' systemctl restart gamesir-xboxd.service padd.service
        fi
    else
        input_setup_complete=false
        install_log 失败 '实际原因见上方；跳过手柄启动，robotd 继续运行'
    fi
    install_phase='配置系统音频'
    python3 "$install_dir/voice-setup.py" "$params_path" --system || install_log 失败 '音频配置未完成；实际原因见上方；robotd 继续运行'
    # Already installed recovery policy produces no duplicate commands/logs.
    install_phase='附属系统设置'
    python3 "$install_dir/runtime.py" disable-automatic-recovery || install_log 失败 '系统设置未完成；实际原因见上方；robotd 继续运行'
    install_phase='配置按需摄像头'
    python3 "$install_dir/camera-setup.py" "$install_dir" || install_log 失败 '摄像头配置未完成；实际原因见上方；robotd 与输入服务继续运行'
    input_paused=0
    rm -rf -- "$temporary_dir"; temporary_dir=
    install_phase=finished
    install_phase='完成安装'
    install_log 成功 "R17 / 0.15.1 已安装；robotd 四步启动已完成，模式 $install_mode；未请求运动"
    echo "{\"upgrade_installed\":true,\"robotd_ready\":true,\"mode\":\"$install_mode\",\"input_setup_complete\":$input_setup_complete}"
}

if [[ ${1:-} == models || ${1:-} == connection ]]; then
    python3 "$bundle_dir/runtime.py" "$1"
    exit 0
fi
require_root
if [[ ${1:-} == power || ${1:-} == power-relaxed ]]; then
    [[ $# -eq 2 ]] || fail '用法：power {reboot|shutdown}'
    python3 "$bundle_dir/runtime.py" power "$2"
    exit 0
fi
exec 9>/run/microduck-service-management.lock
flock -n 9 || fail '已有服务管理操作正在执行。'
case ${1:-install} in
install|start) install_bundle "$@" ;;
upgrade)
    [[ $# -eq 1 ]] || fail '用法：upgrade'
    install_bundle "$@"
    ;;
status)
    require_installed
    show_service
    python3 "$install_dir/runtime.py" connection
    client bus-status
    ;;
reset-models)
    [[ $# -eq 1 ]] || fail '用法：reset-models（明确恢复随包模型并删除其他模型）'
    install_bundle "$@"
    ;;
report)
    require_installed
    report_path=$(mktemp /tmp/robotd-ft5-r5-XXXXXX.json)
    if client bus-status > "$report_path"; then chmod 644 "$report_path"; cat "$report_path"; echo "报告：$report_path" >&2; else rm -f -- "$report_path"; exit 1; fi
    ;;
bus)
    require_installed
    shift
    client bus "$@"
    ;;
enable)
    require_installed
    effective_mode=$(systemctl show robotd.service -p ExecStart --value)
    [[ $effective_mode == *"$install_dir/robotd"* ]] || fail '当前服务路径不匹配，请先 install。'
    if [[ $effective_mode != *--commissioning* ]]; then
        if python3 "$install_dir/runtime.py" verify-running motion "$bus_port"; then
            echo '当前已经是 HAT 运动循环。'
            show_service
            exit 0
        fi
    fi
    # The helper validates the saved calibration, then opts into the motion backend.
    python3 "$install_dir/calibration-config.py" check "$calibration_path"
    identity
    request_relax_service
    systemctl stop robotd.service
    python3 "$install_dir/calibration-config.py" enable "$calibration_path"
    chown "$service_user:$service_group" "$calibration_path"
    write_dropin motion
    restart_service
    python3 "$install_dir/runtime.py" verify-calibration
    show_service
    echo '已切入运动模式；有效反馈和 Home 条件满足后可站起测试。'
    ;;
disable)
    require_installed
    switch_mode=commissioning
    current_exec=$(systemctl show robotd.service -p ExecStart --value)
    [[ $current_exec == *"$install_dir/robotd"* ]] || fail '先升级正式服务再切换诊断模式。'
    request_relax_service
    systemctl stop robotd.service
    write_dropin "$switch_mode"
    restart_service
    python3 "$install_dir/runtime.py" verify-calibration
    show_service
    echo "已请求停扭矩并切换同一服务到 $switch_mode；实际总线状态见上方。"
    ;;
relax)
    require_installed
    relax_service
    ;;
init|enable-policy|disable-policy)
    require_installed
    shift_command=$1
    shift
    client "$shift_command" "$@"
    ;;
reconnect)
    require_installed
    [[ $# -eq 1 ]] || fail '用法：reconnect'
    request_relax_service
    restart_service
    # microduck-replica: padd, and the GameSir bridge only where it is enabled (unchanged for anyone
    # who uses the bridge; a duck whose bridge was disabled on purpose keeps it off).
    start_input_services
    echo 'robotd 已按现有配置重新启动；输入连接复用，不重写配置。'
    ;;
pad-settings)
    require_installed
    [[ $# -eq 3 ]] || fail '用法：pad-settings 嘴巴百分比 头部幅度rad'
    python3 "$install_dir/gamepad.py" validate-settings "$2" "$3"
    python3 "$install_dir/gamepad.py" check-live-settings
    python3 "$install_dir/gamepad.py" save-settings "$2" "$3"
    python3 "$install_dir/gamepad.py" verify-settings
    echo '嘴巴与头部幅度已运行中更新；网页及真实手柄共同生效，未卸力或重启。'
    ;;
action-scale)
    require_installed
    [[ $# -eq 3 ]] || fail '用法：action-scale 模型标识 缩放'
    python3 "$install_dir/runtime.py" validate-scale "$3"
    # Applying numbers does not import or replace a model. Do not reparse all
    # training YAML or launch a second inference validator for this operation.
    request_relax_service
    systemctl stop robotd.service
    python3 "$install_dir/runtime.py" set-action-scale "$2" "$3"
    restart_service
    echo "动作缩放已应用：$3；重新 HOME 后可启用动作。"
    ;;

servo-parameters)
    require_installed
    [[ $# -eq 4 ]] || fail '用法：servo-parameters P D 转矩限制'
    servo_json=$(python3 "$install_dir/runtime.py" validate-servo "$2" "$3" "$4")
    # Allowed but outside the values tried on a duck so far: say so, never ask.
    if (( 10#$2 > 10 || 10#$4 < 500 )); then
        echo "警告：P=$2 大于 10 或转矩限制=$4 小于 500，超出常用范围；下次 HOME 时扶住或吊起鸭子。" >&2
    fi
    python3 "$install_dir/runtime.py" relax-and-restore
    systemctl stop robotd.service || { restart_service || true; start_input_services || true; fail 'robotd 停止失败；已尝试重新启动，未写入。'; }
    if ! python3 "$install_dir/runtime.py" set-servo-parameters "$2" "$3" "$4"; then
        restart_service || true
        start_input_services || true
        fail '舵机参数写入失败；robotd 已重新启动，原设置未变。'
    fi
    restart_service || { start_input_services || true; echo '参数文件已写入；robotd 起来后下次 HOME 上力时生效。' >&2; exit 78; }
    verify_servo_loaded "$servo_json" || { start_input_services || true; fail '参数文件已写入，但控制器回读不一致（见上）。'; }
    start_input_services
    echo "舵机参数已应用：P=$2 D=$3 转矩限制=$4；下次 HOME 上力时写入。卸力时 P 恢复原值，D=0、转矩 1000（同 V167）。"
    ;;
servo-parameters-clear)
    require_installed
    [[ $# -eq 1 ]] || fail '用法：servo-parameters-clear'
    python3 "$install_dir/runtime.py" relax-and-restore
    systemctl stop robotd.service || { restart_service || true; start_input_services || true; fail 'robotd 停止失败；已尝试重新启动，未写入。'; }
    if ! python3 "$install_dir/runtime.py" clear-servo-parameters; then
        restart_service || true
        start_input_services || true
        fail '恢复默认失败；robotd 已重新启动。'
    fi
    restart_service || { start_input_services || true; echo '参数文件已删除；robotd 起来后下次 HOME 按 V167 原版上力。' >&2; exit 78; }
    verify_servo_loaded none || { start_input_services || true; fail '参数文件已删除，但控制器回读不一致（见上）。'; }
    start_input_services
    echo '已恢复默认（未设置）：按 V167 原版，P=合同 5、D=0、转矩 1000。'
    ;;
joint-filters)
    require_installed
    [[ $# -eq 4 ]] || fail '用法：joint-filters 模型标识 头部滤波 腿部滤波'
    # Model id, active Walk and its execution profile are checked here, before anything is relaxed.
    python3 "$install_dir/runtime.py" validate-filters "$2" "$3" "$4" >/dev/null
    python3 "$install_dir/runtime.py" relax-and-restore
    systemctl stop robotd.service || { restart_service || true; start_input_services || true; fail 'robotd 停止失败；已尝试重新启动，未写入。'; }
    if ! python3 "$install_dir/runtime.py" set-joint-filters "$2" "$3" "$4"; then
        restart_service || true
        start_input_services || true
        fail '滤波写入失败；robotd 已重新启动，原设置未变。'
    fi
    restart_service || { start_input_services || true; echo '滤波已写入；robotd 起来后读取新配置。' >&2; exit 78; }
    verify_motion_models || { start_input_services || true; fail '滤波已写入，但模型核对没过（见上）。'; }
    start_input_services
    echo "头腿滤波已应用：头 $3、腿 $4；重新 HOME 后生效。"
    ;;
netpad)
    require_installed
    [[ ${2:-} == on || ${2:-} == off ]] || fail '用法：netpad on [端口] [超时毫秒] | netpad off'
    [[ $2 == off && $# -eq 2 || $2 == on && $# -le 4 ]] || fail '用法：netpad on [端口] [超时毫秒] | netpad off（off 不带参数）'
    # Both robotd.toml and robotd.saved.toml are checked, each against its own [safety]
    # deadman_ms, before anything moves.
    python3 "$install_dir/runtime.py" validate-netpad "${@:2}" >/dev/null
    if [[ $2 == on ]]; then
        grep -aq 'netpad on' "$install_dir/padd" || fail '板上的 padd 不认识 [netpad]（不是 microduck-replica 版）；未改动。'
        padd_allows_inet || fail "padd.service 不允许 AF_INET（$(systemctl show padd.service -p RestrictAddressFamilies --value)）；先用 imu-module-install.sh 安装；未改动。"
        # padd starts only after robotd's four steps; checking it before then would blame padd.
        robotd_ready 10 || fail 'robotd 还没完成四步启动（看 sudo bash service.sh status）；netpad 没有改动。'
    fi
    python3 "$install_dir/runtime.py" relax-and-restore
    netpad_result=$(python3 "$install_dir/runtime.py" set-netpad "${@:2}")
    netpad_port=$(python3 -c 'import json,sys; print(json.loads(sys.argv[1]).get("port") or "")' "$netpad_result")
    if [[ $2 == off ]] && ! robotd_ready 10; then
        python3 "$install_dir/runtime.py" netpad-commit >/dev/null
        systemctl restart padd.service || true
        start_input_services || true
        echo 'netpad off 已写入；robotd 还没完成四步启动，padd 会等它就绪后按 off（蓝牙手柄、网页手柄）启动。'
        exit 0
    fi
    restart_padd_checked "$2" "$netpad_port" || netpad_failed "$2"
    python3 "$install_dir/runtime.py" netpad-commit >/dev/null
    start_input_services
    python3 -c 'import json,sys; print(json.loads(sys.argv[1])["message"])' "$netpad_result"
    ;;

*)
    echo '用法：sudo bash service.sh {install|upgrade|status|connection|report|bus ...|enable|disable|init|enable-policy|disable-policy|relax|reconnect|models|power reboot/shutdown|servo-parameters P D 转矩限制|servo-parameters-clear|joint-filters 模型标识 头部滤波 腿部滤波|netpad on [端口] [超时毫秒]|netpad off}' >&2
    exit 2
    ;;
esac
