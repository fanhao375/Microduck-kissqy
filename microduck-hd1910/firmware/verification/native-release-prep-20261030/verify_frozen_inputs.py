"""Refuse to export native target pins until both independent builds are verified."""
import argparse
import hashlib
import json
from pathlib import Path

SOURCE = "665613ac628fdc9b1b71f324809f0c16ce2795f2"
ARCHIVE_SHA = "19d477da39233009377f6653dff317ba16457534e9e2436545adf40385b02f38"
OLD_SHA = "85fc8051df2ce496abf32bbb5eba3c94c44d875365a8f33a2f53f90facba90db"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verified(root):
    source = json.loads((root / "native-source.json").read_text(encoding="utf-8"))
    assert source["source_commit"] == SOURCE
    assert sha(root / source["archive"]) == source["archive_sha256"] == ARCHIVE_SHA
    assert (root / source["archive"]).stat().st_size == source["archive_size"]
    binaries = [root / ("padd-arm64-" + slot) for slot in "ab"]
    for binary in binaries:
        assert binary.is_file() and not binary.is_symlink(), "independent ARM64 build missing"
        data = binary.read_bytes()
        assert data[:7] == b"\x7fELF\x02\x01\x01" and data[18:20] == b"\xb7\x00", "not AArch64 ELF"
    target_sha = sha(binaries[0])
    assert target_sha == sha(binaries[1]), "independent builds are not byte-identical"
    assert target_sha not in (OLD_SHA, "63c0f7b486604552ad4c95309462f3f9653396fbbf779e69023c074c40d558ab")
    for slot in "ab":
        assert (root / ("qemu-version-" + slot + ".txt")).read_text().strip() == "padd 0.15.1"
        smoke = json.loads((root / ("qemu-process-smoke-" + slot + ".json")).read_text())
        assert smoke["ok"] and smoke["actual_padd_process"]
        assert smoke["hardware_devices"] == smoke["motion_or_skill_requests"] == 0
    assert "113 passed; 0 failed" in (root / "frozen-padd-tests.log").read_text()
    host = json.loads((root / "host-process-smoke.json").read_text())
    assert host["ok"] and host["hardware_devices"] == host["motion_or_skill_requests"] == 0
    return {"schema": "microduck-controller-native-inputs/1", "source_commit": SOURCE,
            "source_archive_sha256": ARCHIVE_SHA, "official_version": "0.15.1",
            "target": "aarch64-unknown-linux-gnu.2.31", "size": binaries[0].stat().st_size,
            "sha256": target_sha, "embedded_revision": SOURCE,
            "embedded_build_time": source["duck_build_time"], "mapping_protocol": 1,
            "supported_installed_sha256": [OLD_SHA], "byte_reproducible": True,
            "actual_qemu_status_verified": True, "hardware_acceptance": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact_directory", type=Path)
    args = parser.parse_args()
    record = verified(args.artifact_directory)
    output = args.artifact_directory / "native-inputs.json"
    with output.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(record, stream, indent=2)
        stream.write("\n")
    print(json.dumps(record))
