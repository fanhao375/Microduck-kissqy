#!/usr/bin/env python3
# Modified 2026-10 by fanhao375 (microduck-replica): trunk IMU module over I2C; optional servo P/D/torque-limit file and joint filters; UDP pad ([netpad]); see IMU-MODULE.md at the repository root.
"""HAT deployment helpers. Motor operations use the single robotd IPC owner."""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import socket
import subprocess
import sys
import tempfile
import time
import tomllib
from management_profile import PROFILE, SLOTS, DEFAULTS

from configure import dumps, atomic_write
from release_identity import BUILD, CONSOLE_VERSION

# Installation and management read the same shipped default; no UI-side scale.
DEFAULT_POLICY = DEFAULTS['policy']
DEFAULT_ACTION_SCALE = DEFAULT_POLICY['action_scale']
DEFAULT_JOINT_ACTION_SCALE = DEFAULTS.get('joint_action_scale',1.0)

BASE = Path('/opt/robot/feetech-ft5-r5')
CONFIG = Path('/etc/robot/feetech-ft5/robotd.toml')
MODEL_DIR = BASE / 'models'
STATE_DIR = Path('/var/lib/robot/feetech-ft5/r5')
DROPINS = Path('/etc/systemd/system/robotd.service.d')
DROPIN_NAME = 'zzzz-r17-0151.conf'
SYSTEMD_DIR = Path('/etc/systemd/system')
PROC = Path('/proc')
# Keep the R17 management identifier; report the package version separately.
MANAGEMENT_REVISION = 'R17'
PORT = '/dev/ttyS2'


def run(*args, timeout=20):
    p = subprocess.run(list(map(str, args)), capture_output=True, text=True, errors='replace', timeout=timeout)
    if p.returncode:
        raise RuntimeError((p.stderr or p.stdout or '命令未成功').strip()[-1500:])
    return p.stdout.strip()


def atomic(path, data, mode=0o644):
    atomic_write(Path(path),data,mode=mode)


def config():
    return tomllib.loads(CONFIG.read_text())


def effective():
    return run('systemctl', 'show', 'robotd.service', '-p', 'ExecStart', '--value')


def detect_port():
    command = effective()
    match = re.search(r'--port(?:=|\s+)(/dev/[A-Za-z0-9/_-]+)', command)
    if match:
        port = match[1]
    else:
        # Read old deployment evidence; every new startup uses the fixed HAT port.
        cfg_match = re.search(r'--params(?:=|\s+)(/[^\s;]+)', command)
        path = Path(cfg_match[1]) if cfg_match else CONFIG
        port = tomllib.loads(path.read_text()).get('bus', {}).get('port')
    if not isinstance(port, str) or not port.startswith('/dev/'):
        raise ValueError('无法核对现有服务的串口配置')
    return port


def effective_config_path():
    match = re.search(r'--params(?:=|\s+)(/[^\s;]+)', effective())
    return Path(match[1]) if match else CONFIG


def configure_hat_port():
    cfg = config()
    if cfg.get('bus',{}).get('port')==PORT:return
    cfg.setdefault('bus', {})['port'] = PORT
    atomic(CONFIG, dumps(cfg))


def verify_calibration():
    """Check the file robotd actually loaded, independently of live servo samples."""
    path=CONFIG.with_name('hd1910-calibration.toml')
    if config().get('bus',{}).get('feetech_calibration')!=str(path):
        raise ValueError('服务参数没有指向本机统一标定路径')
    expected=tomllib.loads(path.read_text())
    report=rpc('robot.busCommand',{'action':'calibration-status'},timeout=3)
    actual=report.get('calibration',{})
    if report.get('calibration_path')!=str(path):raise ValueError('运行服务读取了其他标定文件：'+str(report.get('calibration_path')))
    def same(a,b):
        if type(a) in (int,float) and type(b) in (int,float):return abs(a-b)<1e-8
        return a==b
    for field in ('motion_enabled','imu_mount_verified','units_verified','current_ma_per_count'):
        if not same(actual.get(field),expected.get(field,0 if field=='current_ma_per_count' else False)):raise ValueError('标定运行回读不一致：'+field)
    if len(actual.get('imu_mount_quat',[]))!=4 or any(not same(a,b) for a,b in zip(actual['imu_mount_quat'],expected['imu_mount_quat'])):
        raise ValueError('IMU 安装四元数未按保存文件加载')
    loaded_mounts,saved_mounts=actual.get('imu_mounts',{}),expected.get('imu_mounts',{})
    if set(loaded_mounts)!=set(saved_mounts):raise ValueError('模块 IMU 安装未按保存文件加载')
    for name,saved in saved_mounts.items():
        loaded=loaded_mounts[name]
        if loaded.get('verified',False)!=saved.get('verified',False) or len(loaded.get('quat',[]))!=4 or any(not same(a,b) for a,b in zip(loaded['quat'],saved['quat'])):
            raise ValueError('模块 IMU 安装未按保存文件加载：'+name)
    if len(actual.get('joints',[]))!=15:raise ValueError('运行服务未加载 15 个关节标定')
    for row,loaded in zip(expected['joints'],actual['joints']):
        for key in ('name','id','zero_raw','direction','calibrated','min_rad','max_rad'):
            if not same(loaded.get(key),row.get(key)):raise ValueError('关节标定运行回读不一致：'+row['name']+' / '+key)
    count=sum(r.get('calibrated') is True for r in actual['joints'])
    print(f'标定保存与运行回读一致：{path}；已保存 {count}/15；ID 33 零位 {actual["joints"][8]["zero_raw"]}；IMU 已确认 {actual["imu_mount_verified"]}',flush=True)
    return {'calibration_verified':True,'calibration_path':str(path),'saved_calibrated_count':count,'imu_mount_quat':actual['imu_mount_quat']}


def unit(mode):
    # char-i2c: a trunk IMU module on the board's I2C ([trunk_imu] in robotd.toml) is opened by
    # robotd itself; without it /dev/i2c-* fails with EPERM. Harmless for the bus board.
    if mode not in ('commissioning', 'motion'): raise ValueError('服务模式无效')
    extra = ' --commissioning' if mode == 'commissioning' else ''
    return (f'# R17 / official 0.15.1 / self-trained HD1910 exports.\n'
            f'[Unit]\nOnFailure=\n[Service]\nEnvironment=ORT_DYLIB_PATH={BASE}/onnxruntime/libonnxruntime.so.1.28.0\nUnsetEnvironment=ALSA_CONFIG_PATH\nExecStart=\nExecStart={BASE}/robotd --socket /run/robotd.sock --params {CONFIG} --port {PORT}{extra}\n'
            f'ExecStartPre=\nExecStartPost=\n'
            f'ExecStopPost=\nRestart=always\nRestartSec=2s\n'
            f'ExecStopPost=-/usr/bin/logger -t robotd-exit "result=${{SERVICE_RESULT}} code=${{EXIT_CODE}} status=${{EXIT_STATUS}}"\n'
            f'TimeoutStartSec=30s\nReadWritePaths=/run {CONFIG.parent} {MODEL_DIR} {STATE_DIR}\nPrivateDevices=no\nDeviceAllow={PORT} rw\nDeviceAllow=char-alsa rw\nDeviceAllow=char-i2c rw\nSupplementaryGroups=audio\n')


def configure_startup(mode):
    """Write only the current fixed HAT startup profile."""
    selected = unit(mode)
    MODEL_DIR.mkdir(parents=True,exist_ok=True)
    STATE_DIR.mkdir(parents=True,exist_ok=True,mode=0o700)
    configure_hat_port()
    atomic(DROPINS / DROPIN_NAME, selected)
    return {'port': PORT, 'mode': mode}


def disable_automatic_recovery():
    units=('updaterd.service','robot-boot-check.timer','robot-boot-check.service')
    rule='[Unit]\nConditionPathExists=/run/microduck-explicit-official-recovery\n'
    pending=[name for name in units if not (SYSTEMD_DIR/(name+'.d')/'zzzz-r17-manual.conf').is_file()
             or (SYSTEMD_DIR/(name+'.d')/'zzzz-r17-manual.conf').read_text()!=rule]
    if not pending:return
    failed=False
    for name in pending:
        # A drop-in also works when the unit itself is an /etc file: systemctl
        # mask cannot replace that file. The manual console installer is separate.
        path=SYSTEMD_DIR/(name+'.d')/'zzzz-r17-manual.conf'
        result=subprocess.run(['systemctl','disable','--now',name],check=False,capture_output=True)
        if result is not None and result.returncode:
            failed=True
            detail=result.stderr or result.stdout or '未返回原因'
            if isinstance(detail,bytes):detail=detail.decode(errors='replace')
            print(f'{name} 停止/禁用指令失败：{detail.strip()}',file=sys.stderr,flush=True)
        else:atomic(path,rule)
    run('systemctl','daemon-reload')
    if os.environ.get('MICRODUCK_INSTALL_LOG')=='1':
        print('[固件安装 / 停用自动覆盖固件] '+('失败' if failed else '成功')+' · 已安装一次性系统配置；'+('停止/禁用指令有失败，原因见上方；robotd 继续运行' if failed else '自动更新与 golden 恢复已停用'),flush=True)
    else:
        print('自动更新、开机 golden 恢复已停用；保留中控手动固件升级。',flush=True)

def argument(argv, name):
    values = []
    for index, value in enumerate(argv):
        if value == name and index + 1 < len(argv): values.append(argv[index + 1])
        elif value.startswith(name + '='): values.append(value.split('=', 1)[1])
    return values[0] if len(values) == 1 else None


def running_process():
    pid = int(run('systemctl', 'show', 'robotd.service', '-p', 'MainPID', '--value') or '0')
    if pid <= 0: return {'pid': 0, 'running': False, 'error': '机器人服务未运行'}
    try:
        argv = (PROC / str(pid) / 'cmdline').read_bytes().decode().rstrip('\0').split('\0')
        if not argv or not argv[0]: raise ValueError('服务进程已经退出')
        try: executable = str((PROC / str(pid) / 'exe').readlink())
        except PermissionError: executable = None  # Read-only status may run as radxa.
        if str(pid) != run('systemctl', 'show', 'robotd.service', '-p', 'MainPID', '--value'):
            raise ValueError('服务正在切换进程，请刷新')
        return {'pid': pid, 'running': True, 'argv': argv, 'executable': executable,
                'port': argument(argv, '--port'), 'params': argument(argv, '--params'),
                'mode': 'commissioning' if '--commissioning' in argv else 'motion',
                'uart_setup': '--uart-setup' in argv}
    except (OSError, ValueError) as exc:
        return {'pid': pid, 'running': False, 'error': str(exc)}


def verify_running(mode, port):
    if port != PORT: raise ValueError('服务进程必须使用 HAT /dev/ttyS2')
    process = running_process()
    argv = process.get('argv', [])
    if (not process['running'] or process.get('executable') != str(BASE / 'robotd')
            or not argv or argv[0] != str(BASE / 'robotd')
            or process.get('params') != str(CONFIG) or process.get('port') != port
            or process.get('mode') != mode
            or argument(argv, '--socket') != '/run/robotd.sock'
            or '--fake' in argv or '--read-only' in argv):
        raise ValueError('实际服务进程与所选配置不一致：' + json.dumps(process, ensure_ascii=False))
    print(f'实际进程已核对：PID {process["pid"]}；{port}；{mode}')


def motion_readiness():
    import importlib.util
    try:
        spec = importlib.util.spec_from_file_location('r17_saved_calibration', Path(__file__).with_name('calibration-config.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        path=CONFIG.with_name('hd1910-calibration.toml')
        module.validate(tomllib.loads(path.read_text()),module.configured_source(path))
        return {'motion_ready': True, 'motion_readiness_message': '本机保存的 15 关节标定与 IMU 已通过检查'}
    except (OSError, ValueError, TypeError) as exc:
        return {'motion_ready': False, 'motion_readiness_message': str(exc)}


def connection_status():
    selected = detect_port()
    process = running_process()
    expected_mode = 'commissioning' if '--commissioning' in effective() else 'motion'
    matches = (selected == PORT and process['running'] and process.get('port') == PORT
               and process.get('mode') == expected_mode
               and process.get('params') == str(CONFIG)
               and process.get('uart_setup') is False)
    return {'management_revision': MANAGEMENT_REVISION, 'profile':'hd1910-selftrained-v1',
            'release':'Microduck-Robotd-FT6-R17', 'console_version': CONSOLE_VERSION, 'management_capabilities': {feature:True for feature in PROFILE['features']}, 'boot_port': PORT,
            'selected_port': selected, **motion_readiness(),
            'actual_port': process.get('port'), 'actual_mode': process.get('mode'),
            'pid': process['pid'], 'running': process['running'], 'process_matches': bool(matches),
            'error': process.get('error'), 'device_present': Path(selected).is_char_device()}


def rpc(method, params=None, timeout=15):
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(timeout); s.connect('/run/robotd.sock')
        s.sendall((json.dumps({'jsonrpc':'2.0','id':1,'method':method,'params':params or {}})+'\n').encode())
        with s.makefile('rb') as f: response = json.loads(f.readline(1048576))
    if response.get('error'): raise RuntimeError(str(response['error']))
    result = response['result']
    if isinstance(result, dict) and result.get('accepted') is False:
        raise RuntimeError(result.get('reason', '服务拒绝请求'))
    return result


def torque_off(report):
    if report.get('torque_off_pending'): return False
    if report.get('mode') == 'commissioning': return report.get('torque_state') == 'off'
    return report.get('mode') == 'motion' and report.get('native', {}).get('torque_state_confirmed') is False


def relax_confirmed():
    rpc('robot.relax')
    until = time.monotonic() + 8
    while time.monotonic() < until:
        if torque_off(rpc('robot.busStatus')): return
        time.sleep(.1)
    raise RuntimeError('未确认全部关节扭矩 OFF；操作未继续')


def request_relax():
    """Best-effort stop before reconnecting the HAT service."""
    try:
        rpc('robot.relax', timeout=2)
        return {'requested': True, 'message': '已请求旧服务停止；正在重新连接 HAT'}
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        return {'requested': False, 'message': '旧服务未返回停止确认；将在 HAT 重新初始化',
                'detail': str(exc)[-300:]}


def power(action):
    from power_control import power as submit_power
    return submit_power(action,runner=run)



def validate_scale(value):
    scale=float(value)
    if scale not in tuple(round(n/100,2) for n in range(10,101)):raise ValueError('缩放只能选择 0.10 到 1.00，步长 0.01')
    return scale

def model_execution_profile(path):
    """Read the model-bound execution settings without reparsing training YAML."""
    from model_packages import EXECUTION_FILE,describe_directory
    directory=Path(path).parent
    if not (directory/EXECUTION_FILE).exists():return None
    return describe_directory(directory)['execution_profile']

def set_action_scale(ident,value):
    scale=validate_scale(value)
    if not re.fullmatch('[a-f0-9]{32}',ident):raise ValueError('模型标识无效')
    if ident not in {row['id'] for row in PROFILE['builtin_models']}:raise ValueError('请选择内置模型')
    data=tomllib.loads(CONFIG.read_text())
    policy=data.setdefault('policy',{})
    active={Path(str(policy.get(slot,''))).parent.name for slot in ('walk','sitstand')}
    if ident not in active:raise ValueError('请选择已安装且启用的自训模型')
    walk=Path(str(policy.get('walk','')))
    profile=model_execution_profile(walk) if walk.parent.name==ident else None
    if profile is not None:
        from model_packages import EXECUTION_FILE
        # Native Walk already uses this value as its base scale. Updating the
        # old global would leave this model unchanged and affect other actions.
        updated={**profile,'action_scale':scale}
        atomic_write(walk.parent/EXECUTION_FILE,json.dumps(updated,ensure_ascii=False,indent=2)+'\n')
        return {'model':ident,'scale_scope':'joint','joint_action_scale':scale,
                'action_scale':policy.get('action_scale',DEFAULT_ACTION_SCALE),
                'execution_profile':updated,'sitstand_base_action_scale':1.0,
                'message':'联合模型基础缩放已保存；头腿滤波与其他动作保持原值。重启后读取新配置。'}
    policy['action_scale']=scale
    policy.pop('r17_inference_scale',None)
    for key in ('walk_action_scale','hold_action_scale','action_scales'):policy.pop(key,None)
    text=dumps(data)
    atomic_write(CONFIG,text)
    atomic_write(CONFIG.with_name(CONFIG.stem+'.saved'+CONFIG.suffix),text)
    return {'model':ident,'action_scale':scale,'sitstand_base_action_scale':1.0,'message':'动作缩放已保存；重启后重新 HOME。'}

# Optional servo P/D/torque limit (microduck-replica): robotd reads this file at startup and writes
# it at the next HOME arm (duck-control hd1910_test). Relax leaves P restored, D=0 and torque limit
# 1000, as V167 does. No file = V167 unchanged (contract P, D=0, limit 1000). Same ranges as robotd.
SERVO_FILE_NAME = 'servo-test-parameters.json'
SERVO_LIMITS = {'kp': (1, 32), 'kd': (0, 64), 'torque_limit': (200, 1000)}

def servo_path():
    return CONFIG.with_name(SERVO_FILE_NAME)

def validate_servo(kp, kd, torque_limit):
    values = {}
    for name, text in zip(('kp', 'kd', 'torque_limit'), (kp, kd, torque_limit)):
        text = str(text); low, high = SERVO_LIMITS[name]
        if not re.fullmatch('[0-9]{1,4}', text) or not low <= int(text) <= high:
            raise ValueError(f'{name} 必须是 {low} 到 {high} 的整数')
        values[name] = int(text)
    return values

def servo_parameters():
    path = servo_path()
    if not path.exists(): return None
    try:
        data = json.loads(path.read_text())
        # robotd (serde) accepts JSON integers only; strings or booleans would leave it waiting for the bus.
        if not isinstance(data, dict) or set(data) != set(SERVO_LIMITS) or any(type(v) is not int for v in data.values()):
            raise ValueError('须为 kp/kd/torque_limit 三个整数')
        return validate_servo(**data)
    except (OSError, ValueError, TypeError) as exc: return {'error': f'{SERVO_FILE_NAME} 无效：{exc}'}

def set_servo_parameters(kp, kd, torque_limit):
    values = validate_servo(kp, kd, torque_limit)
    atomic_write(servo_path(), json.dumps(values, ensure_ascii=False) + '\n', mode=0o644)
    return {'servo_parameters': values, 'message': '舵机参数已保存；下次 HOME 上力时写入。卸力时 P 恢复原值，D=0、转矩 1000（同 V167）。'}

def clear_servo_parameters():
    path = servo_path(); existed = path.exists()
    path.unlink(missing_ok=True)
    return {'servo_parameters': None, 'removed': existed, 'message': '已恢复默认（未设置）：按 V167 原版，P=合同 5、D=0、转矩 1000。若曾在上力中重启过 robotd，P 可能停在旧值，给舵机断一次电即恢复。'}

def verify_servo_parameters(expected):
    """Poll robotd until busStatus reports the file it loaded (motion mode only)."""
    wanted = None if expected == 'none' else json.loads(expected)
    until = time.monotonic() + 8; last = '等待控制器启动'
    while time.monotonic() < until:
        try:
            bus = rpc('robot.busStatus', timeout=2)
            hd = (bus.get('native') or {}).get('hd1910') or {}
            if 'servo_parameters' not in hd:
                last = '控制器没有回报 servo_parameters（板上 robotd 不是可调版本？）'
            elif hd['servo_parameters'] == wanted:
                return {'servo_parameters_verified': True, 'servo_parameters': wanted, 'gain_mode': (bus.get('native') or {}).get('gain_mode')}
            else:
                last = '控制器加载的是 ' + json.dumps(hd['servo_parameters'], ensure_ascii=False)
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            last = str(exc)
        time.sleep(.1)
    raise ValueError('舵机参数已保存，但控制器回读不一致：' + last[-800:])

def relax_and_restore():
    """Before stopping robotd for a servo/filter change. In motion control: relax, then wait until
    torque is OFF and the HD1910 restore has finished, otherwise refuse (robotd keeps running, so the
    in-memory restore data is not lost). Not in control (bad file, waiting for bus, commissioning,
    unreachable): nothing is armed by this robotd, request a relax and let the caller stop it.
    Only a socket that is missing or refused means no robotd; a slow answer is asked again for 8 s,
    and still nothing means the torque state is unknown: request a relax and say so."""
    until = time.monotonic() + 8; last = ''; report = None
    while report is None and time.monotonic() < until:
        try:
            report = rpc('robot.busStatus', timeout=2)
        except (FileNotFoundError, ConnectionRefusedError) as exc:
            return {'restored': None, 'message': 'robotd 不在运行（' + str(exc)[-120:] + '），直接停止'}
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            last = str(exc)[-200:]; time.sleep(.1)
    if report is None:
        request_relax()
        return {'restored': None, 'message': '扭矩状态未确认（robotd 8 秒内没回答：' + last + '），已请求卸力；请扶住或放倒鸭子'}
    if report.get('mode') != 'motion' or report.get('phase') != 'control':
        request_relax()
        return {'restored': None, 'message': f"robotd 处于 {report.get('mode')} / {report.get('phase')}，不在控制阶段，直接停止"}
    rpc('robot.relax')
    until = time.monotonic() + 8; last = ''
    while time.monotonic() < until:
        try:
            report = rpc('robot.busStatus', timeout=2)
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            last = str(exc)[-200:]; time.sleep(.1); continue   # one slow reply is not a refusal
        hd = (report.get('native') or {}).get('hd1910') or {}
        if torque_off(report) and hd.get('restore_pending') is False and hd.get('temporary_gains_applied') is False:
            return {'restored': True, 'message': '已卸力：P 已恢复原值，D=0、转矩 1000（同 V167）'}
        last = json.dumps({'torque_off': torque_off(report), 'restore_pending': hd.get('restore_pending'),
                           'temporary_gains_applied': hd.get('temporary_gains_applied')}, ensure_ascii=False)
        time.sleep(.1)
    raise RuntimeError('未确认卸力并恢复 P 原值，操作未继续，robotd 保持运行：' + last +
                       '。若是某个舵机不应答：先断舵机电，再 sudo systemctl stop robotd，然后重试这条命令')

def validate_filter(value):
    lowpass = float(value)
    if lowpass not in tuple(round(n/100, 2) for n in range(10, 101)): raise ValueError('滤波只能选择 0.10 到 1.00，步长 0.01')
    return lowpass

def check_joint_filters(ident, head, legs):
    """Every read-only check of joint-filters (values, model id, the active Walk, its joint execution
    profile), run before robotd is relaxed and stopped. Returns what set_joint_filters writes."""
    head, legs = validate_filter(head), validate_filter(legs)
    if not re.fullmatch('[a-f0-9]{32}', ident): raise ValueError('模型标识无效')
    walk = Path(str(tomllib.loads(CONFIG.read_text()).get('policy', {}).get('walk', '')))
    if walk.parent.name != ident: raise ValueError('只能修改当前启用的 Walk 模型')
    from model_packages import EXECUTION_FILE, describe_directory, execution_profile
    info = describe_directory(walk.parent)
    if 'execution_profile' not in info: raise ValueError('该模型没有联合执行配置；滤波由 robotd.toml 决定')
    updated = {**info['execution_profile'], 'head_lowpass': head, 'legs_lowpass': legs}
    text = json.dumps(updated, ensure_ascii=False, indent=2) + '\n'
    contract = json.loads((walk.parent/'deployment-contract.json').read_text())
    execution_profile(text.encode(), {**{k: info[k] for k in ('slot', 'label', 'task', 'sha256')}, 'contract': contract})
    return walk.parent/EXECUTION_FILE, updated, text

def set_joint_filters(ident, head, legs):
    """Edit only head_lowpass/legs_lowpass of the active joint Walk; scale stays with set_action_scale."""
    path, updated, text = check_joint_filters(ident, head, legs)
    original = path.with_name(path.name + '.orig')
    if not original.exists(): atomic_write(original, path.read_bytes())   # the first original is never replaced
    atomic_write(path, text)
    return {'model': ident, 'execution_profile': updated, 'message': '头腿滤波已保存；缩放保持原值。重启后读取新配置。'}

# UDP pad (microduck-replica): [netpad] in robotd.toml and robotd.saved.toml. padd reads it at start;
# robotd checks it at its own start (robotd-params). Same rules as robotd-params: the types and
# ranges are checked whatever `enabled` says (strict parse: port u16, timeout_ms u64, enabled bool),
# and with enabled = true port 1..65535 and 0 < timeout_ms < that file's [safety] deadman_ms (500
# when unset). Off is never refused and keeps the numbers already there. Each file is checked
# against its own deadman. A file this run writes gets a <name>.netpad-previous copy first (not
# .previous: kissqy's atomic_write rewrites that on every write); service.sh either drops the copies
# (netpad-commit) or puts exactly those files back (restore-netpad), so nothing flips twice.
NETPAD_BACKUP = '.netpad-previous'
NETPAD_DEFAULTS = {'port': 4210, 'timeout_ms': 250}
NETPAD_DEADMAN_MS = 500
U64_MAX = 2**64 - 1

def netpad_paths():
    saved = CONFIG.with_name(CONFIG.stem + '.saved' + CONFIG.suffix)
    return [CONFIG] + ([saved] if saved.exists() else [])

def netpad_number(text, name, low, high):
    text = str(text)
    if not re.fullmatch('[0-9]{1,20}', text) or not low <= int(text) <= high:
        raise ValueError(f'{name} 必须是 {low} 到 {high} 的整数')
    return int(text)

def netpad_plan(action, *args):
    """The [netpad] each file gets for `netpad on [port] [timeout_ms]` / `netpad off`; nothing written."""
    if action not in ('on', 'off'): raise ValueError('用法：netpad on [端口] [超时毫秒] 或 netpad off')
    if action == 'off' and args: raise ValueError('netpad off 不带参数（保留文件里已有的端口和超时）')
    if len(args) > 2: raise ValueError('用法：netpad on [端口] [超时毫秒]')
    port = netpad_number(args[0], '端口', 1, 65535) if args else None
    timeout = netpad_number(args[1], 'timeout_ms', 1, U64_MAX) if len(args) > 1 else None
    plans = []
    for path in netpad_paths():
        data = tomllib.loads(path.read_text())
        section = data.get('netpad', {})
        if not isinstance(section, dict): raise ValueError(f'{path.name}: netpad 不是一节')
        safety = data.get('safety', {})
        deadman = safety.get('deadman_ms', NETPAD_DEADMAN_MS) if isinstance(safety, dict) else None
        if type(deadman) is not int or not 0 <= deadman <= U64_MAX:
            raise ValueError(f'{path.name}: safety.deadman_ms 不是整数，robotd 读不了这个文件；先修好它')
        old_port, old_timeout = section.get('port'), section.get('timeout_ms')
        port_ok = type(old_port) is int and 0 <= old_port <= 65535
        timeout_ok = type(old_timeout) is int and 0 <= old_timeout <= U64_MAX
        updated = dict(section)
        if action == 'off' and 'netpad' not in data:
            updated = None   # never had the section: nothing to switch off, the file stays as it is
        elif action == 'off':
            # Off is a way back, never refused: only a value robotd could not even parse is reset.
            updated['enabled'] = False
            if 'port' in section and not port_ok: updated['port'] = NETPAD_DEFAULTS['port']
            if 'timeout_ms' in section and not timeout_ok: updated['timeout_ms'] = NETPAD_DEFAULTS['timeout_ms']
        else:
            new_port = port if port is not None else old_port if port_ok and old_port >= 1 else NETPAD_DEFAULTS['port']
            new_timeout = timeout if timeout is not None else old_timeout if 'timeout_ms' in section else NETPAD_DEFAULTS['timeout_ms']
            if type(new_timeout) is not int or not 0 < new_timeout < deadman:
                raise ValueError(f'{path.name}: timeout_ms {new_timeout!r} 必须大于 0 且小于 safety.deadman_ms {deadman}'
                                 '（超过 deadman 会把掉线客户端最后的摇杆再发一遍）；用 netpad on 端口 超时毫秒 指定')
            updated.update(enabled=True, port=new_port, timeout_ms=new_timeout)
        plans.append((path, data, updated, deadman))
    return plans

def validate_netpad(action, *args):
    return [{'file': str(path), 'netpad': updated, 'deadman_ms': deadman} for path, _, updated, deadman in netpad_plan(action, *args)]

def set_netpad(action, *args):
    plans = netpad_plan(action, *args)
    texts = []
    for path, data, updated, _ in plans:
        if updated is None or data.get('netpad') == updated: continue
        text = dumps({**data, 'netpad': updated})
        if tomllib.loads(text).get('netpad') != updated: raise ValueError(f'{path.name}: [netpad] 写出后读回不一致，未写入')
        texts.append((path, text))
    netpad_commit()   # a copy left by an earlier run is not this run's to restore
    for path, _ in texts:   # every copy first, so restore-netpad can put back what was written
        atomic_write(path.with_name(path.name + NETPAD_BACKUP), path.read_bytes())
    for path, text in texts:
        atomic_write(path, text)
    first = plans[0][2] or {'enabled': False}
    state = f"开（UDP 端口 {first['port']}，超时 {first['timeout_ms']} ms）" if first['enabled'] else '关（读蓝牙手柄与网页手柄）'
    return {'netpad': [{'file': str(path), 'netpad': updated} for path, _, updated, _ in plans],
            'written': [str(path) for path, _ in texts], 'port': first.get('port'),
            'message': f'[netpad] {state}；写入 {len(texts)} 份配置，padd 重启后生效。'}

def netpad_backups():
    return [path for path in netpad_paths() if path.with_name(path.name + NETPAD_BACKUP).exists()]

def restore_netpad():
    """Put back the files the last set-netpad wrote (only those) and drop their copies."""
    restored = []
    for path in netpad_backups():
        backup = path.with_name(path.name + NETPAD_BACKUP)
        atomic_write(path, backup.read_bytes()); backup.unlink(); restored.append(str(path))
    return {'restored': restored,
            'message': ('已恢复 ' + '、'.join(restored)) if restored else '这次没有写过配置，不用恢复'}

def netpad_commit():
    """The change stays: drop the copies set-netpad made."""
    dropped = []
    for path in netpad_backups():
        path.with_name(path.name + NETPAD_BACKUP).unlink(); dropped.append(str(path))
    return {'dropped': dropped}

def model_list():
    from model_packages import catalog
    result=catalog(BASE,CONFIG);policy=tomllib.loads(CONFIG.read_text()).get('policy',{})
    for row in result['models']:
        row['action_scale']=policy.get('action_scale',DEFAULT_ACTION_SCALE) if row.get('slot')=='walk' else 1.0
        if row.get('slot')=='walk' and row.get('execution_profile'):
            row['action_scale']=row['execution_profile']['action_scale']
        if row.get('slot')=='roulade':
            row['action_scale']=next((a.get('params',{}).get('action_scale',policy.get('action_scale',DEFAULT_ACTION_SCALE)) for a in policy.get('skill',[]) if a.get('name')=='roulade'),policy.get('action_scale',DEFAULT_ACTION_SCALE))
    return {**result,'management_actions':['action-scale','joint-action-scale','servo-parameters','joint-filters'],'servo_parameters':servo_parameters(),'servo_limits':SERVO_LIMITS,'action_scale':policy.get('action_scale',DEFAULT_ACTION_SCALE),'default_action_scale':DEFAULT_ACTION_SCALE,'default_joint_action_scale':DEFAULT_JOINT_ACTION_SCALE,'port':detect_port()}

def verify_loaded_models():
    paths={slot:path for slot,path in config()['policy'].items() if slot in SLOTS and path and path!='none'}
    expected={slot:hashlib.sha256(Path(path).read_bytes()).hexdigest() for slot,path in paths.items()}
    expected_profiles={slot:model_execution_profile(path) for slot,path in paths.items()}
    until=time.monotonic()+8;last='等待控制器加载'
    while time.monotonic()<until:
        try:
            bus=rpc('robot.busStatus',timeout=2)
            actual=bus.get('loaded_models') or {}
            policy=config()['policy']
            settings={key:policy.get(key,default) for key,default in DEFAULT_POLICY.items()}
            params_match=all(bus.get(key)==value for key,value in settings.items())
            # The ONNX digest is unchanged when only its execution sidecar is
            # edited. Confirm the loaded profile, including unchanged filters.
            params_match=params_match and all((actual.get(slot) or {}).get('runtime_execution_profile')==profile
                                             for slot,profile in expected_profiles.items())
            if 'roulade' in expected:
                scale=next((a.get('params',{}).get('action_scale',policy.get('action_scale',DEFAULT_ACTION_SCALE)) for a in policy.get('skill',[]) if a.get('name')=='roulade'),policy.get('action_scale',DEFAULT_ACTION_SCALE))
                params_match=params_match and (bus.get('model_action_scales') or {}).get('roulade')==scale
            if params_match and bus.get('build')==BUILD and all((actual.get(slot) or {}).get('sha256')==sha for slot,sha in expected.items()):
                print('已确认控制器实际加载：',json.dumps(actual,ensure_ascii=False),flush=True)
                return {'loaded_models_verified':True,'models':actual,'action_scale':bus['action_scale'],'head_lowpass':bus['head_lowpass'],'legs_lowpass':bus['legs_lowpass']}
            last=json.dumps({'build':bus.get('build'),'loaded_models':actual},ensure_ascii=False)
        except (OSError,ValueError,RuntimeError,KeyError) as exc:last=str(exc)
        time.sleep(.1)
    raise ValueError('安装文件已更新，但未确认运行模型一致：'+last[-1500:])


def main():
    action, *args = sys.argv[1:]
    if action == 'disable-automatic-recovery':disable_automatic_recovery();return
    if action == 'validate-scale': print(validate_scale(*args)); return
    if action == 'validate-servo': print(json.dumps(validate_servo(*args))); return
    if action == 'validate-filters': print(json.dumps(check_joint_filters(*args)[1], ensure_ascii=False)); return
    if action == 'verify-servo': print(json.dumps(verify_servo_parameters(*args), ensure_ascii=False)); return
    if action == 'validate-netpad': print(json.dumps(validate_netpad(*args), ensure_ascii=False)); return
    if action == 'port': print(detect_port()); return
    if action == 'config-path': print(effective_config_path()); return
    if action == 'connection': print(json.dumps(connection_status(), ensure_ascii=False)); return
    if action == 'verify-running': verify_running(*args); return
    if action == 'verify-models': print(json.dumps(verify_loaded_models(),ensure_ascii=False)); return
    if action == 'verify-calibration': print(json.dumps(verify_calibration(),ensure_ascii=False)); return
    if action == 'bus-status': result = rpc('robot.busStatus', timeout=2)
    elif action == 'models': result = model_list()
    else:
        if os.geteuid() != 0: raise PermissionError('此操作需要 sudo')
        if action == 'configure-startup': result = configure_startup(*args)
        elif action == 'power': result = power(*args)
        elif action == 'set-action-scale': result = set_action_scale(*args)
        elif action == 'relax-and-restore': result = relax_and_restore()
        elif action == 'set-servo-parameters': result = set_servo_parameters(*args)
        elif action == 'clear-servo-parameters': result = clear_servo_parameters()
        elif action == 'set-joint-filters': result = set_joint_filters(*args)
        elif action == 'set-netpad': result = set_netpad(*args)
        elif action == 'restore-netpad': result = restore_netpad()
        elif action == 'netpad-commit': result = netpad_commit()
        elif action == 'relax-confirmed': relax_confirmed(); result = {'torque':'off'}
        elif action == 'relax-request': result = request_relax()
        else: raise ValueError('不支持的运行管理操作')
    if os.environ.get('MICRODUCK_INSTALL_LOG')=='1' and action=='configure-startup':
        return  # The installer owns this step result, with port and mode.
    elif os.environ.get('MICRODUCK_INSTALL_LOG')=='1' and action=='relax-request':
        label='成功' if result.get('requested') else '提示'
        detail='旧 robotd 已接收松开请求；进程停止由下一步 systemd 确认' if result.get('requested') else '旧 robotd 未返回松开确认：'+result.get('detail',result.get('message','旧 robotd 不可达'))
        print(f'[固件安装 / 请求停止运动] {label} · {detail}',flush=True)
    else:print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    try: main()
    except Exception as exc: print(str(exc), file=sys.stderr); sys.exit(1)
