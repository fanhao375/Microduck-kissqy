#!/usr/bin/env python3
"""Exercise the actual padd process in a Linux environment without input devices.

Every robot connection terminates at this temporary read-only fixture. Refuse a
host with input devices, so this smoke test cannot accidentally read a real pad.
"""
import argparse
import glob
import json
from pathlib import Path
import socket
import stat
import subprocess
import tempfile
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--expect-revision", help="Verify the embedded DUCK_REVISION startup identity")
    args = parser.parse_args()
    assert not glob.glob("/dev/input/event*"), "Use a Linux environment without input devices"
    with tempfile.TemporaryDirectory(prefix="padd-process-smoke-") as name:
        directory = Path(name)
        robot = directory / "robot.sock"
        listener = socket.socket(socket.AF_UNIX)
        listener.bind(str(robot))
        listener.listen()
        listener.settimeout(0.1)
        stop = threading.Event()
        methods = []
        failures = []

        def connection(peer):
            try:
                with peer, peer.makefile("rwb") as stream:
                    for line in stream:
                        request = json.loads(line)
                        method = request.get("method")
                        methods.append(method)
                        assert method in {"robot.mode", "robot.busStatus", "robot.subscribe"}, request
                        if method == "robot.mode":
                            result = {"mode": "walk"}
                        elif method == "robot.busStatus":
                            result = {"homed": True, "policy_enabled": False}
                        else:
                            result = {"accepted": True}
                        stream.write((json.dumps({"jsonrpc": "2.0", "id": request["id"], "result": result}) + "\n").encode())
                        stream.flush()
                        if method == "robot.subscribe":
                            for tick in range(2):
                                state = {"policy": "held", "t_ns": 1_000_000_000 + tick * 20_000_000,
                                         "move": {"requested": [0, 0, 0], "applied": [0, 0, 0]},
                                         "safety": {"limp": False}}
                                stream.write((json.dumps({"jsonrpc": "2.0", "method": "robot.state", "params": state}) + "\n").encode())
                                stream.flush()
                                time.sleep(0.02)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except BaseException as error:
                failures.append(repr(error))

        def accept():
            while not stop.is_set():
                try:
                    peer, _ = listener.accept()
                except socket.timeout:
                    continue
                threading.Thread(target=connection, args=(peer,), daemon=True).start()

        thread = threading.Thread(target=accept, daemon=True)
        thread.start()
        config = directory / "robotd.toml"
        config.write_text("")
        mapping = directory / "mapping.sock"
        tap = directory / "pad.sock"
        with (directory / "stderr.log").open("wb") as errors:
            process = subprocess.Popen([str(args.binary), "--socket", str(robot), "--config", str(config),
                                        "--tap-socket", str(tap), "--mapping-socket", str(mapping),
                                        "--mapping-state-dir", str(directory / "profiles")],
                                       stdout=subprocess.DEVNULL, stderr=errors)
            try:
                until = time.monotonic() + 4
                while not mapping.exists():
                    assert process.poll() is None, (directory / "stderr.log").read_text()
                    assert time.monotonic() < until, "padd did not create mapping socket"
                    time.sleep(0.01)

                def request(path, value):
                    with socket.socket(socket.AF_UNIX) as client:
                        client.settimeout(3)
                        client.connect(str(path))
                        client.sendall((json.dumps(value) + "\n").encode())
                        return json.loads(client.makefile("rb").readline())

                status = request(mapping, {"op": "status"})
                assert status["ok"] and status["version"] == 1, status
                assert status["status"]["device"] is None and status["status"]["raw"] is None, status
                assert status["status"]["output_blocked"] is False, status
                refused = request(mapping, {"op": "begin", "device_id": "absent"})
                assert refused["ok"] is False and refused["code"] == "no_device", refused
                assert request(mapping, {"op": "renew", "token": "unknown"})["code"] == "invalid_token"
                assert request(tap, {"jsonrpc": "2.0", "id": 1, "method": "pad.input"})["result"]["accepted"]
                assert "error" in request(tap, {"jsonrpc": "2.0", "id": 1, "method": "pad.mapping.begin"})
                state = json.loads((directory / "mapping-state.json").read_text())
                assert state == {"version": 1, "pid": process.pid, "output_blocked": False, "device": None, "profile": None}, state
                assert stat.S_IMODE(mapping.stat().st_mode) == 0o660
                assert stat.S_IMODE((directory / "mapping-state.json").stat().st_mode) == 0o640
                assert not (directory / "profiles" / "mappings.json").exists()
                assert not failures, failures
                if args.expect_revision:
                    assert "rev " + args.expect_revision in (directory / "stderr.log").read_text(), "embedded source revision missing"
                assert methods and all(method in {"robot.mode", "robot.busStatus", "robot.subscribe"} for method in methods), methods
                print(json.dumps({"ok": True, "actual_padd_process": True, "robot_methods": methods,
                                  "motion_or_skill_requests": 0, "hardware_devices": 0}, sort_keys=True))
            finally:
                process.terminate()
                process.wait(timeout=3)
                stop.set()
                thread.join(timeout=1)
                listener.close()


if __name__ == "__main__":
    main()
