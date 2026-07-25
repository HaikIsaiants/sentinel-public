import argparse
import gzip
import json
import os
import pathlib
import struct
import subprocess
import tempfile
import time

from google.protobuf import text_format
from sentinel.v1 import sentinel_pb2

import benchmark_runtime
from generate_scenarios import generate_scenario, load_seed_records


ROOT = pathlib.Path(__file__).resolve().parents[1]


def command(args, scenario, output, simulator=None, agent=None):
    return [
        args.supervisor,
        "--sim",
        simulator or args.sim,
        "--agent",
        agent or args.agent,
        "--scenario",
        scenario,
        "--log",
        output / "events.pb",
        "--summary",
        output / "summary.json",
    ]


def terminal_frame(data):
    if len(data) < 4:
        raise RuntimeError("supervisor returned no terminal frame")
    size = struct.unpack(">I", data[:4])[0]
    if size != len(data) - 4:
        raise RuntimeError("supervisor returned an invalid terminal frame")
    frame = sentinel_pb2.SimulationFrame()
    frame.ParseFromString(data[4:])
    if not frame.HasField("observations") or not frame.observations.finished:
        raise RuntimeError("supervisor returned a nonterminal frame")
    return frame


def compare_case(args, scenario, root, horizon_ticks):
    reference = root / "reference"
    native = root / "native"
    values = benchmark_runtime.run_mission(args.sim, args.agent, scenario, reference, horizon_ticks)
    if not values["replay_verified"]:
        raise RuntimeError(values["error"] or "reference replay failed")
    native.mkdir(parents=True)
    result = subprocess.run(command(args, scenario, native), capture_output=True, timeout=120)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.decode())
    frame = terminal_frame(result.stdout)
    expected = json.loads((reference / "summary.json").read_text(encoding="utf-8"))
    actual = json.loads((native / "summary.json").read_text(encoding="utf-8"))
    if actual != expected:
        raise RuntimeError("native and reference summaries differ")
    if frame.observations.summary.terminal_hash != actual["terminal_hash"]:
        raise RuntimeError("terminal frame and summary artifact differ")
    if list(frame.observations.summary.tick_hashes) != actual["tick_hashes"]:
        raise RuntimeError("terminal frame tick hashes differ")
    with gzip.open(reference / "events.pb.gz", "rb") as stream:
        if stream.read() != (native / "events.pb").read_bytes():
            raise RuntimeError("native and reference event logs differ")


def children(pid):
    path = pathlib.Path(f"/proc/{pid}/task/{pid}/children")
    if not path.is_file():
        return []
    value = path.read_text(encoding="utf-8").strip()
    return [int(item) for item in value.split()] if value else []


def verify_termination(args, root):
    output = root / "termination"
    output.mkdir()
    process = subprocess.Popen(
        command(args, ROOT / "scenarios" / "compound_failure_cbba.textproto", output),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    descendants = []
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and process.poll() is None:
            descendants = children(process.pid)
            if len(descendants) == 6:
                break
            time.sleep(0.005)
        if len(descendants) != 6:
            raise RuntimeError("supervisor did not create the isolated process set")
        process.terminate()
        process.wait(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and any(pathlib.Path(f"/proc/{pid}").exists() for pid in descendants):
        time.sleep(0.01)
    if any(pathlib.Path(f"/proc/{pid}").exists() for pid in descendants):
        raise RuntimeError("terminated supervisor left child processes")
    if process.returncode == 0:
        raise RuntimeError("terminated supervisor reported success")


def verify_parent_death(args, root):
    output = root / "parent-death"
    output.mkdir()
    reader, writer = os.pipe()
    launcher = os.fork()
    if launcher == 0:
        os.close(reader)
        try:
            process = subprocess.Popen(
                command(args, ROOT / "scenarios" / "compound_failure_cbba.textproto", output),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            os.write(writer, f"{process.pid}\n".encode())
            os.close(writer)
            process.wait()
            os._exit(0)
        except Exception:
            os.write(writer, b"0\n")
            os.close(writer)
            os._exit(1)
    os.close(writer)
    supervisor = -1
    descendants = []
    reaped = False
    try:
        with os.fdopen(reader, encoding="utf-8") as stream:
            supervisor = int(stream.readline() or "0")
        if supervisor <= 0:
            raise RuntimeError("launcher failed to start supervisor")
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            descendants = children(supervisor)
            if len(descendants) == 6:
                break
            time.sleep(0.005)
        if len(descendants) != 6:
            raise RuntimeError("launcher did not create the isolated process set")
        os.kill(launcher, 9)
        os.waitpid(launcher, 0)
        reaped = True
        tracked = [supervisor, *descendants]
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and any(pathlib.Path(f"/proc/{pid}").exists() for pid in tracked):
            time.sleep(0.01)
        if any(pathlib.Path(f"/proc/{pid}").exists() for pid in tracked):
            raise RuntimeError("dead launcher left mission processes")
    finally:
        if not reaped:
            try:
                os.kill(launcher, 9)
                os.waitpid(launcher, 0)
            except (ChildProcessError, ProcessLookupError):
                pass
        for pid in [supervisor, *descendants]:
            try:
                if pid > 0 and pathlib.Path(f"/proc/{pid}").exists():
                    os.kill(pid, 9)
            except ProcessLookupError:
                pass


def verify_failures(args, root):
    cases = (
        ("child", args.sim, root / "missing-agent", []),
        ("truncated", pathlib.Path("/usr/bin/printf"), args.agent, []),
        ("timeout", args.fixture, args.agent, ["--timeout-ms", "25", "--shutdown-timeout-ms", "100"]),
    )
    for name, simulator, agent, extra in cases:
        output = root / name
        output.mkdir()
        result = subprocess.run(
            command(args, ROOT / "scenarios" / "nominal.textproto", output, simulator, agent) + extra,
            capture_output=True,
            timeout=5,
        )
        if result.returncode == 0:
            raise RuntimeError(f"supervisor accepted {name} failure")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--supervisor", type=pathlib.Path, required=True)
    parser.add_argument("--sim", type=pathlib.Path, required=True)
    parser.add_argument("--agent", type=pathlib.Path, required=True)
    parser.add_argument("--fixture", type=pathlib.Path, required=True)
    args = parser.parse_args()
    benchmark = json.loads((ROOT / "config" / "benchmark.json").read_text(encoding="utf-8"))
    horizon_ticks = benchmark["clock"]["mission_horizon_ticks"]
    with tempfile.TemporaryDirectory() as directory:
        output = pathlib.Path(directory)
        compare_case(args, ROOT / "scenarios" / "nominal.textproto", output / "nominal", horizon_ticks)
        compare_case(
            args,
            ROOT / "scenarios" / "compound_failure_cbba.textproto",
            output / "compound",
            horizon_ticks,
        )
        record = next(value for value in load_seed_records("development") if value["id"] == "development-00008")
        generated = output / "generated.textproto"
        generated.write_text(
            text_format.MessageToString(generate_scenario(record, "sentinel_cbba")),
            encoding="utf-8",
            newline="\n",
        )
        compare_case(args, generated, output / "generated", horizon_ticks)
        verify_termination(args, output)
        verify_parent_death(args, output)
        verify_failures(args, output)
    print("supervisor tests passed")


if __name__ == "__main__":
    main()
