"""Read the preserved 20261027 payload; do not install or extract executables."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

parser = argparse.ArgumentParser()
parser.add_argument("payload", type=Path)
parser.add_argument("--observed", type=Path)
args = parser.parse_args()
with args.payload.open("rb") as stream:
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    payload_sha = digest.hexdigest()
assert payload_sha == "370f64af5c690eeebab429f4c59d9c5d00883b5cf1e7f0d0e2c18d4afe45998d"
result = {"payload_sha256": payload_sha, "members": []}
with tarfile.open(args.payload, "r:gz") as archive:
    for member in archive:
        if member.isfile() and member.name.endswith(("/padd", "/padd.service", "/mediad.service")):
            data = archive.extractfile(member).read()
            row = {"member": member.name, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}
            if member.name.endswith(".service"):
                row["unit"] = data.decode()
            result["members"].append(row)
if args.observed:
    observation = json.loads(args.observed.read_text(encoding="utf-8"))
    recorded = json.loads(observation["stdout"])
    result["prior_offline_observation"] = {name: {key: row[key] for key in ("running_executable", "sha256")}
                                            for name, row in recorded["services"].items()
                                            if name in ("padd", "robotd", "mediad")}
print(json.dumps(result, ensure_ascii=False, indent=2))
