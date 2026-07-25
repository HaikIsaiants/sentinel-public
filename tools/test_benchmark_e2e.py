import argparse
import json
import os
import pathlib
import subprocess
import sys
import tempfile

import benchmark_runtime


ROOT = pathlib.Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", type=pathlib.Path, required=True)
    parser.add_argument("--agent", type=pathlib.Path, required=True)
    parser.add_argument("--supervisor", type=pathlib.Path)
    options = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        output = pathlib.Path(directory) / "benchmark"
        scratch = pathlib.Path(directory) / "scratch"
        command = [
            sys.executable,
            str(ROOT / "tools" / "run_benchmark.py"),
            "--sim",
            str(options.sim),
            "--agent",
            str(options.agent),
        ]
        if options.supervisor:
            command.extend(("--supervisor", str(options.supervisor)))
        command.extend(
            (
                "--corpus",
                "development",
                "--output",
                str(output),
                "--scratch",
                str(scratch),
                "--jobs",
                "2",
                "--limit",
                "1",
            )
        )
        result = subprocess.run(
            command,
            capture_output=True,
            env=os.environ,
            text=True,
        )
        if result.returncode:
            raise RuntimeError(result.stderr or result.stdout)
        rows = [json.loads(line) for line in (output / "results.jsonl").read_text(encoding="utf-8").splitlines()]
        if len(rows) != 2 or {row["allocator"] for row in rows} != {"sentinel_cbba", "nearest_capable"}:
            raise RuntimeError("paired development rows are missing")
        if len({row["seed_id"] for row in rows}) != 1 or len({row["paired_input_sha256"] for row in rows}) != 1:
            raise RuntimeError("paired development inputs differ")
        for row in rows:
            if row["status"] != "terminated" or not row["replay_verified"] or not row["terminal_event_present"]:
                raise RuntimeError("development mission did not terminate and replay")
            paths = {
                "event_log": output / row["run_path"] / "events.pb.gz",
                "replay_summary": output / row["run_path"] / "replay.json",
                "scenario": output / row["scenario_path"],
                "summary": output / row["run_path"] / "summary.json",
            }
            if set(row["artifact_hashes"]) != set(paths) or any(
                benchmark_runtime.artifact(path) != row["artifact_hashes"][name] for name, path in paths.items()
            ):
                raise RuntimeError("development artifacts failed verification")
        if any(path.is_file() for path in scratch.rglob("*")):
            raise RuntimeError("scratch artifacts were not removed")
        print("paired benchmark test passed")


if __name__ == "__main__":
    main()
