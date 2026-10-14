# Added 2026-10 by fanhao375 (microduck-replica): optional servo P/D/torque-limit file and joint filters; Apache-2.0 like the rest of this workspace.
"""Optional servo P/D/torque limit file and joint Walk head/legs filters (service.sh commands)."""
import json
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import runtime
from configure import dumps

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'policies/walk'
ID = '1cbf8730eadd75a5704332c9f3c10f60'
NAMES = ('policy.onnx', 'deployment-contract.json', 'manifest.json', 'deployment-execution.json')


class ServoParameterTests(unittest.TestCase):
    def test_ranges_match_robotd(self):
        for good in (('3', '0', '681'), ('1', '0', '200'), ('32', '64', '1000'), (5, 40, 1000)):
            values = runtime.validate_servo(*good)
            self.assertEqual(values, dict(zip(('kp', 'kd', 'torque_limit'), map(int, good))))
        for bad in (('0', '0', '681'), ('33', '0', '681'), ('3', '65', '681'), ('3', '0', '199'),
                    ('3', '0', '1001'), ('3.5', '0', '681'), ('', '0', '681'), ('abc', '0', '681'),
                    ('-1', '0', '681'), ('3', '0', '12345')):
            with self.assertRaises(ValueError):
                runtime.validate_servo(*bad)

    def test_save_clear_and_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'robotd.toml'
            with patch.object(runtime, 'CONFIG', config):
                self.assertIsNone(runtime.servo_parameters())
                result = runtime.set_servo_parameters('3', '0', '700')
                self.assertEqual(result['servo_parameters'], {'kp': 3, 'kd': 0, 'torque_limit': 700})
                path = Path(tmp) / 'servo-test-parameters.json'
                # robotd (serde, deny_unknown_fields) reads exactly these three keys.
                self.assertEqual(json.loads(path.read_text()), {'kp': 3, 'kd': 0, 'torque_limit': 700})
                self.assertEqual(runtime.servo_parameters(), {'kp': 3, 'kd': 0, 'torque_limit': 700})
                with self.assertRaises(ValueError):
                    runtime.set_servo_parameters('3', '0', '50')
                self.assertEqual(json.loads(path.read_text())['torque_limit'], 700)   # refused value never written
                path.write_text('{"kp": 99, "kd": 0, "torque_limit": 700}\n')
                self.assertIn('error', runtime.servo_parameters())               # a bad hand edit is reported, not hidden
                self.assertTrue(runtime.clear_servo_parameters()['removed'])
                self.assertFalse(path.exists())
                self.assertFalse(runtime.clear_servo_parameters()['removed'])

    def test_messages_describe_the_v167_relax_state(self):
        # Relax leaves P restored, D=0 and limit 1000 (as V167); no file = V167 unchanged.
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(runtime, 'CONFIG', Path(tmp) / 'robotd.toml'):
                saved = runtime.set_servo_parameters('3', '0', '700')['message']
                cleared = runtime.clear_servo_parameters()['message']
        self.assertIn('卸力时 P 恢复原值，D=0、转矩 1000（同 V167）', saved)
        self.assertIn('按 V167 原版，P=合同 5、D=0、转矩 1000', cleared)
        for text in (saved, cleared):
            self.assertNotIn('只写模型合同的 P', text)


class JointFilterTests(unittest.TestCase):
    def environment(self, base, walk_id=ID):
        directory = base / 'models' / ID
        directory.mkdir(parents=True)
        for name in NAMES:
            shutil.copyfile(FIXTURE / name, directory / name)
        config = base / 'robotd.toml'
        config.write_text(dumps({'policy': {'walk': str(base / 'models' / walk_id / 'policy.onnx'), 'action_scale': .7}}))
        return config, directory / 'deployment-execution.json'

    def test_only_the_two_filters_change_and_the_first_original_is_kept(self):
        with tempfile.TemporaryDirectory() as tmp:
            config, path = self.environment(Path(tmp))
            original = path.read_bytes(); before = json.loads(original)
            with patch.object(runtime, 'CONFIG', config):
                result = runtime.set_joint_filters(ID, '0.8', '0.9')
                after = json.loads(path.read_text())
                self.assertEqual((after['head_lowpass'], after['legs_lowpass']), (0.8, 0.9))
                self.assertEqual({k: v for k, v in after.items() if k not in ('head_lowpass', 'legs_lowpass')},
                                 {k: v for k, v in before.items() if k not in ('head_lowpass', 'legs_lowpass')})
                self.assertEqual(result['execution_profile'], after)
                orig = path.with_name(path.name + '.orig')
                self.assertEqual(orig.read_bytes(), original)
                runtime.set_joint_filters(ID, '1.0', '0.5')
                self.assertEqual(orig.read_bytes(), original)          # a later edit never replaces the first original
                self.assertEqual(json.loads(path.read_text())['legs_lowpass'], 0.5)

    def test_invalid_values_or_other_models_change_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            config, path = self.environment(Path(tmp))
            before = path.read_bytes()
            with patch.object(runtime, 'CONFIG', config):
                for head, legs in [('0.09', '.5'), ('1.01', '.5'), ('.5', '0'), ('0.555', '.5'), ('nan', '.5'), ('.5', 'inf')]:
                    with self.assertRaises(ValueError):
                        runtime.set_joint_filters(ID, head, legs)
                for ident in ('0' * 32, 'not-an-id'):
                    with self.assertRaises(ValueError):
                        runtime.set_joint_filters(ident, '.5', '.7')
            self.assertEqual(path.read_bytes(), before)
            self.assertFalse(path.with_name(path.name + '.orig').exists())

    def test_the_check_before_relax_reads_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            config, path = self.environment(Path(tmp))
            before = path.read_bytes()
            with patch.object(runtime, 'CONFIG', config):
                _, updated, _ = runtime.check_joint_filters(ID, '0.8', '0.9')
                self.assertEqual((updated['head_lowpass'], updated['legs_lowpass']), (0.8, 0.9))
                for ident in ('0' * 32, 'not-an-id'):
                    with self.assertRaises(ValueError):
                        runtime.check_joint_filters(ident, '.5', '.7')
            self.assertEqual(path.read_bytes(), before)
            self.assertFalse(path.with_name(path.name + '.orig').exists())

    def test_scale_slider_and_filters_do_not_overwrite_each_other(self):
        with tempfile.TemporaryDirectory() as tmp:
            config, path = self.environment(Path(tmp))
            profile = runtime.SLOTS  # noqa: F841  (module import guard)
            with patch.object(runtime, 'CONFIG', config), \
                 patch.object(runtime, 'PROFILE', {**runtime.PROFILE, 'builtin_models': [{'id': ID}]}):
                runtime.set_joint_filters(ID, '0.8', '0.9')
                runtime.set_action_scale(ID, .85)
                after = json.loads(path.read_text())
                self.assertEqual((after['action_scale'], after['head_lowpass'], after['legs_lowpass']), (0.85, 0.8, 0.9))
                runtime.set_joint_filters(ID, '0.6', '0.7')
                self.assertEqual(json.loads(path.read_text())['action_scale'], 0.85)


class ServiceScriptTests(unittest.TestCase):
    def test_service_offers_the_three_commands_and_confirms_restore_first(self):
        text = (ROOT / 'packaging/service.sh').read_text()
        for command in ('servo-parameters)', 'servo-parameters-clear)', 'joint-filters)'):
            self.assertIn(command, text)
        # Commissioning never reads the file: save only, no readback timeout.
        self.assertIn("if [[ $running == *--commissioning* ]]; then\n        echo '标定模式不读取舵机参数", text)
        self.assertIn('verify_servo_loaded none', text)
        # The three tunable commands; netpad) follows them and is tested in test_netpad.py.
        block = text[text.index('servo-parameters)\n'):text.index('\nnetpad)\n')]
        self.assertIn('verify_servo_loaded "$servo_json"', block)
        self.assertEqual(block.count("systemctl stop robotd.service || { restart_service || true;"), 3)
        # robotd is only stopped after the relax/restore check succeeds.
        self.assertEqual(block.count('runtime.py" relax-and-restore'), 3)
        self.assertNotIn('request_relax_service', block)
        # P above 10 or a limit below 500 is warned about, never asked about (no prompt).
        self.assertIn('if (( 10#$2 > 10 || 10#$4 < 500 )); then', block)
        self.assertNotIn('read ', block)
        self.assertIn('卸力时 P 恢复原值，D=0、转矩 1000（同 V167）', block)
        self.assertIn('按 V167 原版，P=合同 5、D=0、转矩 1000', block)
        # The installer still retires the file on install/upgrade/reset-models (kissqy behaviour).
        self.assertIn('rm -f -- "$config_dir/servo-test-parameters.json"', text)
        self.assertEqual(text.splitlines()[1][:46], '# Modified 2026-10 by fanhao375 (microduck-rep')
        # A failed restart after the file changed says so: the value is in effect once robotd is up.
        self.assertEqual(block.count('exit 78; }'), 3)
        self.assertIn('参数文件已写入；robotd 起来后下次 HOME 上力时生效', block)
        self.assertIn('参数文件已删除；robotd 起来后下次 HOME 按 V167 原版上力', block)
        # joint-filters checks the model id and the active Walk before it relaxes anything.
        filters = block[block.index('joint-filters)\n'):]
        self.assertLess(filters.index('validate-filters "$2" "$3" "$4"'), filters.index('relax-and-restore'))


class RelaxRestoreTests(unittest.TestCase):
    """relax-and-restore stops robotd only after torque OFF and the HD1910 restore finished."""
    def run_with(self, reports, relax_raises=None):
        calls = []
        def rpc(method, params=None, timeout=15):
            calls.append(method)
            if method == 'robot.relax':
                if relax_raises: raise relax_raises
                return {}
            value = reports.pop(0) if len(reports) > 1 else reports[0]
            if isinstance(value, Exception): raise value
            return value
        with patch.object(runtime, 'rpc', rpc), patch.object(runtime.time, 'sleep', lambda s: None):
            return runtime.relax_and_restore(), calls

    @staticmethod
    def motion(torque_confirmed, pending, applied):
        return {'mode': 'motion', 'phase': 'control',
                'native': {'torque_state_confirmed': torque_confirmed, 'hd1910': {'restore_pending': pending, 'temporary_gains_applied': applied}}}

    def test_restored_motion_proceeds(self):
        result, calls = self.run_with([self.motion(True, False, True), self.motion(False, False, False)])
        self.assertTrue(result['restored']); self.assertIn('robot.relax', calls)

    def test_pending_restore_refuses_and_keeps_robotd(self):
        with patch.object(runtime.time, 'monotonic', side_effect=[0, 1, 2, 9, 10, 11, 12]):
            with self.assertRaisesRegex(RuntimeError, 'robotd 保持运行'):
                self.run_with([self.motion(True, False, True), self.motion(False, True, False)])

    def test_one_slow_reply_is_retried(self):
        result, _ = self.run_with([self.motion(True, False, True), OSError('timeout'), self.motion(False, False, False)])
        self.assertTrue(result['restored'])

    def test_not_in_control_or_unreachable_proceeds_without_waiting(self):
        for report in ({'mode': 'motion', 'phase': 'waiting_for_bus', 'error': 'servo test parameters: bad'},
                       {'mode': 'commissioning', 'phase': 'control'}):
            with patch.object(runtime, 'request_relax', return_value={}):
                result, calls = self.run_with([report])
            self.assertIsNone(result['restored']); self.assertNotIn('robot.relax', calls)
        for missing in (FileNotFoundError('no socket'), ConnectionRefusedError('refused')):
            with patch.object(runtime, 'request_relax') as relax:
                result, calls = self.run_with([missing])
            self.assertIsNone(result['restored']); self.assertIn('不在运行', result['message'])
            relax.assert_not_called(); self.assertNotIn('robot.relax', calls)

    def test_a_slow_first_answer_is_asked_again(self):
        result, calls = self.run_with([TimeoutError('timed out'), self.motion(True, False, True), self.motion(False, False, False)])
        self.assertTrue(result['restored']); self.assertIn('robot.relax', calls)

    def test_no_answer_for_eight_seconds_requests_a_relax_and_says_the_torque_is_unknown(self):
        clock = iter(range(0, 100))
        with patch.object(runtime.time, 'monotonic', lambda: next(clock)), \
             patch.object(runtime, 'request_relax', return_value={}) as relax:
            result, calls = self.run_with([TimeoutError('timed out')])
        relax.assert_called_once()
        self.assertIsNone(result['restored'])
        self.assertIn('扭矩状态未确认', result['message']); self.assertIn('扶住或放倒鸭子', result['message'])
        self.assertNotIn('robot.relax', calls)

    def test_a_restore_that_never_finishes_says_how_to_get_out(self):
        with patch.object(runtime.time, 'monotonic', side_effect=[0, 0, 1, 2, 9, 10, 11]):
            with self.assertRaisesRegex(RuntimeError, '先断舵机电，再 sudo systemctl stop robotd'):
                self.run_with([self.motion(True, False, True), self.motion(False, True, False)])


class VerifyServoTests(unittest.TestCase):
    def verify(self, reports, expected):
        it = iter(reports)
        def rpc(method, params=None, timeout=15):
            value = next(it, reports[-1])
            if isinstance(value, Exception): raise value
            return value
        clock = iter(range(0, 100))
        with patch.object(runtime, 'rpc', rpc), patch.object(runtime.time, 'sleep', lambda s: None), \
             patch.object(runtime.time, 'monotonic', lambda: next(clock)):
            return runtime.verify_servo_parameters(expected)

    @staticmethod
    def bus(parameters, field=True):
        hd = {'servo_parameters': parameters} if field else {}
        return {'native': {'gain_mode': 'x', 'hd1910': hd}}

    def test_matching_parameters_and_cleared_file(self):
        values = {'kp': 3, 'kd': 0, 'torque_limit': 700}
        self.assertTrue(self.verify([OSError('starting'), self.bus(values)], json.dumps(values))['servo_parameters_verified'])
        self.assertTrue(self.verify([self.bus(None)], 'none')['servo_parameters_verified'])

    def test_official_robotd_or_wrong_values_fail(self):
        with self.assertRaisesRegex(ValueError, '不是可调版本'):
            self.verify([self.bus(None, field=False)], 'none')
        with self.assertRaisesRegex(ValueError, '控制器加载的是'):
            self.verify([self.bus({'kp': 5, 'kd': 40, 'torque_limit': 1000})], json.dumps({'kp': 3, 'kd': 0, 'torque_limit': 700}))

    def test_saved_file_must_hold_json_integers(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'robotd.toml'; path = Path(tmp) / 'servo-test-parameters.json'
            with patch.object(runtime, 'CONFIG', config):
                for text in ('{"kp": "3", "kd": 0, "torque_limit": 700}', '{"kp": true, "kd": 0, "torque_limit": 700}',
                             '{"kp": 3.0, "kd": 0, "torque_limit": 700}', '[3, 0, 700]'):
                    path.write_text(text)
                    self.assertIn('error', runtime.servo_parameters(), text)


if __name__ == '__main__':
    unittest.main()
