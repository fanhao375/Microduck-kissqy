# Added 2026-10 by fanhao375 (microduck-replica): UDP pad (service.sh netpad); Apache-2.0 like the rest of this workspace.
"""service.sh netpad on/off: [netpad] in robotd.toml and robotd.saved.toml, robotd-params' rules."""
import re
import subprocess
import tempfile
import tomllib
import unittest
from pathlib import Path
from unittest.mock import patch

import runtime

ROOT = Path(__file__).resolve().parents[1]
BOARD = (ROOT / 'packaging/fixtures/robotd-board-v167.toml').read_bytes()
# What configure.dumps writes for `netpad on` with the defaults; robotd-params loads exactly this
# block appended to the same fixture (lib.rs, board_robotd_toml_with_the_netpad_block_loads).
BLOCK = '["netpad"]\n"enabled" = true\n"port" = 4210\n"timeout_ms" = 250\n'
SERVICE = (ROOT / 'packaging/service.sh').read_text(encoding='utf-8')


class Board:
    """robotd.toml and robotd.saved.toml in a temporary /etc/robot/feetech-ft5."""
    def __init__(self, toml=BOARD, saved=None):
        self.tmp = tempfile.TemporaryDirectory()
        self.config = Path(self.tmp.name) / 'robotd.toml'
        self.saved = Path(self.tmp.name) / 'robotd.saved.toml'
        self.config.write_bytes(toml)
        self.saved.write_bytes(toml if saved is None else saved)
        self.patch = patch.object(runtime, 'CONFIG', self.config)

    def __enter__(self):
        self.patch.start()
        return self

    def __exit__(self, *exc):
        self.patch.stop()
        self.tmp.cleanup()

    def netpad(self, path):
        return tomllib.loads(path.read_text()).get('netpad')

    def both(self):
        return self.netpad(self.config), self.netpad(self.saved)


class NetpadRulesTests(unittest.TestCase):
    """The three robotd-params rules (lib.rs netpad_* tests), plus types and ranges."""

    def test_timeout_must_be_below_the_deadman_and_port_above_zero(self):
        with Board() as board:
            for args in (('4210', '500'), ('4210', '0'), ('0', '250'), ('4210', '501')):
                with self.assertRaises(ValueError, msg=args):
                    runtime.set_netpad('on', *args)
            self.assertEqual(board.config.read_bytes(), BOARD)
            self.assertEqual(board.saved.read_bytes(), BOARD)
            self.assertFalse(board.config.with_name('robotd.toml.previous').exists())

    def test_off_is_never_refused_and_keeps_the_numbers(self):
        enabled_500 = BOARD + b'\n["netpad"]\n"enabled" = true\n"port" = 4300\n"timeout_ms" = 500\n'
        with Board(enabled_500) as board:
            runtime.set_netpad('off')
            for netpad in board.both():
                self.assertEqual(netpad, {'enabled': False, 'port': 4300, 'timeout_ms': 500})

    def test_a_raised_deadman_allows_a_longer_timeout(self):
        raised = BOARD.replace(b'"limp_fall" = false', b'"limp_fall" = false\n"deadman_ms" = 800')
        with Board(raised) as board:
            runtime.set_netpad('on', '4210', '500')
            self.assertEqual(board.both(), ({'enabled': True, 'port': 4210, 'timeout_ms': 500},) * 2)

    def test_arguments_are_integers_in_range(self):
        with Board() as board:
            for args in (('65536',), ('-1',), ('abc',), ('3.5',), ('4210', '1e3'), ('', '250'),
                         ('4210', '250', 'extra')):
                with self.assertRaises(ValueError, msg=args):
                    runtime.set_netpad('on', *args)
            with self.assertRaises(ValueError):
                runtime.set_netpad('off', '4210')
            with self.assertRaises(ValueError):
                runtime.set_netpad('toggle')
            self.assertEqual(board.config.read_bytes(), BOARD)

    def test_each_file_is_checked_against_its_own_deadman(self):
        raised = BOARD.replace(b'"limp_fall" = false', b'"limp_fall" = false\n"deadman_ms" = 800')
        with Board(raised, saved=BOARD) as board:
            with self.assertRaisesRegex(ValueError, 'robotd.saved.toml'):
                runtime.set_netpad('on', '4210', '600')
            self.assertEqual(board.config.read_bytes(), raised)   # nothing written to either
            self.assertEqual(board.saved.read_bytes(), BOARD)
            runtime.set_netpad('on', '4210', '400')
            self.assertEqual(board.both(), ({'enabled': True, 'port': 4210, 'timeout_ms': 400},) * 2)

    def test_stored_values_robotd_cannot_parse_are_reset_by_off_and_refused_by_on(self):
        bad = BOARD + b'\n["netpad"]\n"enabled" = true\n"port" = "x"\n"timeout_ms" = -5\n'
        with Board(bad) as board:
            with self.assertRaises(ValueError):
                runtime.set_netpad('on')                # stored timeout -5 is not a valid number
            runtime.set_netpad('off')
            self.assertEqual(board.netpad(board.config), {'enabled': False, 'port': 4210, 'timeout_ms': 250})


class NetpadFileTests(unittest.TestCase):
    """The board's real robotd.toml: quoted table names, inline skill array, [trunk_imu]."""

    def test_on_writes_one_section_in_both_files_and_keeps_everything_else(self):
        with Board() as board:
            original = tomllib.loads(BOARD.decode())
            result = runtime.set_netpad('on')
            self.assertEqual(result['port'], 4210)
            for path in (board.config, board.saved):
                text = path.read_text()
                self.assertEqual(text.count('["netpad"]'), 1)
                self.assertNotIn('\r', text)
                self.assertIn(BLOCK, text)
                data = tomllib.loads(text)
                self.assertEqual(data.pop('netpad'), {'enabled': True, 'port': 4210, 'timeout_ms': 250})
                self.assertEqual(data, original)           # skills, pad, trunk_imu all as they were
                backup = path.with_name(path.name + '.netpad-previous')
                self.assertEqual(backup.read_bytes(), BOARD)
            runtime.netpad_commit()                        # the change stays: no copy left behind
            self.assertEqual(runtime.netpad_backups(), [])

    def test_on_twice_and_off_never_duplicate_the_section(self):
        with Board() as board:
            runtime.set_netpad('on', '4300', '200')
            runtime.set_netpad('on')                       # keeps 4300 / 200 already in the file
            self.assertEqual(board.both(), ({'enabled': True, 'port': 4300, 'timeout_ms': 200},) * 2)
            runtime.set_netpad('off')
            for path in (board.config, board.saved):
                self.assertEqual(path.read_text().count('netpad'), 1)
                self.assertEqual(board.netpad(path), {'enabled': False, 'port': 4300, 'timeout_ms': 200})

    def test_off_without_a_section_leaves_the_files_alone(self):
        with Board() as board:
            result = runtime.set_netpad('off')
            self.assertEqual(result['written'], [])
            self.assertEqual(board.config.read_bytes(), BOARD)
            self.assertEqual(board.saved.read_bytes(), BOARD)

    def test_restore_puts_both_files_back_once(self):
        with Board() as board:
            runtime.set_netpad('on')
            runtime.restore_netpad()
            self.assertEqual(board.config.read_bytes(), BOARD)
            self.assertEqual(board.saved.read_bytes(), BOARD)
            self.assertEqual(runtime.netpad_backups(), [])
            runtime.restore_netpad()                       # a second restore never flips it back
            self.assertEqual(board.config.read_bytes(), BOARD)

    def test_restore_after_writing_nothing_changes_nothing(self):
        with Board() as board:
            # kissqy's own atomic_write leaves robotd.toml.previous on every write: not ours to use.
            from configure import atomic_write, dumps
            data = tomllib.loads(BOARD.decode()); data['policy']['action_scale'] = 0.55
            atomic_write(board.config, dumps(data))
            changed = board.config.read_bytes()
            self.assertTrue(board.config.with_name('robotd.toml.previous').exists())
            self.assertEqual(runtime.set_netpad('off')['written'], [])   # no section: nothing written
            self.assertEqual(runtime.restore_netpad()['restored'], [])
            self.assertEqual(board.config.read_bytes(), changed)
            self.assertEqual(board.saved.read_bytes(), BOARD)

    def test_restore_puts_back_only_what_this_run_wrote(self):
        with Board() as board:
            runtime.set_netpad('on'); runtime.netpad_commit()
            on = board.config.read_bytes()
            runtime.set_netpad('off')
            board.saved.write_bytes(BOARD)                 # changed by someone else after the write
            runtime.restore_netpad()
            self.assertEqual(board.config.read_bytes(), on)

    def test_validate_reads_without_writing(self):
        with Board() as board:
            plans = runtime.validate_netpad('on', '4211')
            self.assertEqual([p['netpad']['port'] for p in plans], [4211, 4211])
            self.assertEqual([p['deadman_ms'] for p in plans], [500, 500])
            self.assertEqual(board.config.read_bytes(), BOARD)
            self.assertFalse(board.config.with_name('robotd.toml.previous').exists())
            self.assertEqual(runtime.netpad_backups(), [])


def function(name):
    """One bash function from service.sh, to run on its own with stubbed commands."""
    match = re.search(r'^' + name + r'\(\) \{\n.*?\n\}\n', SERVICE, re.S | re.M)
    assert match, name
    return match.group(0)


def bash(script, **env):
    return subprocess.run(['bash', '-c', script], env={'PATH': '/usr/bin:/bin', **env},
                          capture_output=True, text=True, timeout=30)


class ServiceNetpadTests(unittest.TestCase):
    def block(self):
        return SERVICE[SERVICE.index('\nnetpad)\n'):SERVICE.index('\n*)\n')]

    def test_order_checks_relax_write_verify_and_restore(self):
        block = self.block()
        order = ['validate-netpad', "grep -aq 'netpad on'", 'padd_allows_inet', 'robotd_ready 10',
                 'relax-and-restore', 'set-netpad']
        positions = [block.index(word) for word in order]
        self.assertEqual(positions, sorted(positions), 'checks (robotd ready for on), relax, write')
        # then: off with robotd not ready keeps the change and leaves padd to wait for it; otherwise
        # padd is checked, a failure goes to netpad_failed, success drops the copies.
        verify = block.index('restart_padd_checked "$2"')
        self.assertLess(block.index('if [[ $2 == off ]] && ! robotd_ready 10'), verify)
        self.assertLess(verify, block.index('|| netpad_failed "$2"'))
        self.assertLess(verify, block.rindex('netpad-commit'))
        self.assertIn('[[ $2 == off && $# -eq 2 || $2 == on && $# -le 4 ]]', block)   # off takes no numbers
        self.assertNotIn('systemctl stop robotd', block)                             # only padd restarts
        self.assertNotIn('enable', block.replace('enabled', ''))

    def test_every_command_that_restarts_robotd_brings_padd_back(self):
        start = SERVICE.index('\nservo-parameters)\n'); end = SERVICE.index('\nnetpad)\n')
        block = SERVICE[start:end]
        self.assertEqual(block.count('start_input_services || true'), 12)  # stop / write / restart / verify failed
        self.assertEqual(block.count('    start_input_services\n'), 3)      # the three success paths
        reconnect = SERVICE[SERVICE.index('\nreconnect)\n'):SERVICE.index('\npad-settings)\n')]
        self.assertIn('start_input_services', reconnect)
        self.assertNotIn('gamesir-xboxd', reconnect)       # the bridge only through start_input_services

    def test_a_failed_check_rolls_back_on_but_never_off(self):
        stub = ('python3() { echo "python3 $*" >> "$LOG"; }\n'
                'systemctl() { echo "systemctl $*" >> "$LOG"; return 0; }\n'
                'robotd_state_hint() { echo "robotd 已就绪"; }\n'
                'fail() { echo "错误：$*" >&2; return 1; }\n'
                'install_dir=/opt/robot/feetech-ft5-r5\n')
        script = stub + function('start_input_services') + function('netpad_failed')
        for want, rolled_back in (('on', True), ('off', False)):
            with tempfile.TemporaryDirectory() as tmp:
                log = Path(tmp) / 'calls'
                result = bash(script + f'netpad_failed {want}', LOG=str(log))
                calls = log.read_text()
                self.assertNotEqual(result.returncode, 0, want)
                self.assertEqual('restore-netpad' in calls, rolled_back, want)
                self.assertEqual('netpad-commit' in calls, not rolled_back, want)
                self.assertIn('systemctl start padd.service', calls)
                self.assertIn('robotd 已就绪', result.stderr)
                if want == 'off':
                    self.assertIn('off 不回滚', result.stderr)

    def test_the_bridge_is_started_only_where_it_is_enabled(self):
        script = function('start_input_services') + 'start_input_services'
        stub = 'systemctl() { echo "systemctl $*" >> "$LOG"; [[ $1 == is-enabled ]] && echo "$STATE"; return 0; }\n'
        for state, bridge in (('enabled', True), ('disabled', False), ('masked', False)):
            with tempfile.TemporaryDirectory() as tmp:
                log = Path(tmp) / 'calls'
                result = bash(stub + script, LOG=str(log), STATE=state)
                self.assertEqual(result.returncode, 0, result.stderr)
                calls = log.read_text()
                self.assertIn('systemctl start padd.service', calls)
                self.assertEqual('systemctl start gamesir-xboxd.service' in calls, bridge, state)
                self.assertNotIn('systemctl enable', calls)

    def test_af_inet_is_checked_as_a_set(self):
        script = 'systemctl() { echo "$FAMILIES"; }\n' + function('padd_allows_inet') + 'padd_allows_inet'
        for families, allowed in (('AF_UNIX AF_NETLINK', False), ('AF_UNIX AF_NETLINK AF_INET', True),
                                  ('AF_INET AF_NETLINK AF_UNIX', True), ('', True),
                                  ('~AF_INET', False), ('~AF_PACKET', True), ('AF_INET6', False)):
            self.assertEqual(bash(script, FAMILIES=families).returncode == 0, allowed, families)

    def test_padd_is_judged_by_its_new_process_log(self):
        stub = ('systemctl() { [[ $1 == show ]] && echo 4242; return 0; }\n'
                'journalctl() { [[ " $* " == *" _PID=4242 "* ]] && printf "%b" "$LOG"; return 0; }\n'
                'sleep() { :; }\n')
        on = 'netpad on — reading the pad from UDP\\ndriving — Start once stands up\\n'
        off = 'driving — Start once stands up\\n'
        for want, log, ok in (('on', on, True), ('on', off, False), ('off', off, True), ('off', on, False),
                              ('on', 'netpad on — reading\\n', False)):
            script = stub + function('restart_padd_checked') + f'restart_padd_checked {want} ""'
            self.assertEqual(bash(script, LOG=log).returncode == 0, ok, (want, log))


if __name__ == '__main__':
    unittest.main()
