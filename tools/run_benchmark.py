import argparse
import concurrent.futures
import json
import pathlib
import shutil

import benchmark_runtime
import run_manifest


ROOT = pathlib.Path(__file__).resolve().parents[1]


def key(row):
    return row["seed_id"], row["allocator"]


def result_path(output, shard_index, shard_count):
    if shard_count == 1:
        return output / "results.jsonl"
    return output / f"results-{shard_index:05d}-of-{shard_count:05d}.jsonl"


def read_results(path):
    if not path.is_file():
        return {}
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
    result = {}
    for row in rows:
        identity = key(row)
        if identity in result:
            raise RuntimeError(f"duplicate existing result: {identity}")
        result[identity] = row
    return result


def write_results(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = "".join(
        json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
        for row in sorted(rows, key=key)
    )
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(data)
    tmp.replace(path)


def journal_root(output, shard_index, shard_count):
    if shard_count == 1:
        return output / "journal"
    return output / f"journal-{shard_index:05d}-of-{shard_count:05d}"


def journal_path(root, row):
    return root / row["seed_id"] / f"{row['allocator']}.json"


def write_journal(root, row):
    # One file per run makes interrupted batches cheap to resume.
    path = journal_path(root, row)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
    tmp.replace(path)


def read_journal(root):
    if not root.is_dir():
        return {}
    result = {}
    for path in sorted(root.rglob("*.json")):
        row = json.loads(path.read_text(encoding="utf-8"))
        identity = key(row)
        if identity in result:
            raise RuntimeError(f"duplicate journal result: {identity}")
        result[identity] = row
    return result


def select_records(records, shard_index, shard_count, limit=None):
    if shard_count < 1 or not 0 <= shard_index < shard_count:
        raise RuntimeError("invalid shard")
    selected = [record for idx, record in enumerate(records) if idx % shard_count == shard_index]
    return selected[:limit] if limit is not None else selected


def artifact_paths(output, row):
    run = output / row["run_path"]
    scenario = output / row["scenario_path"]
    return {
        "event_log": run / "events.pb.gz",
        "replay_summary": run / "replay.json",
        "scenario": scenario,
        "summary": run / "summary.json",
    }


def verify_existing(output, row):
    for name, expected in row.get("artifact_hashes", {}).items():
        if expected is None:
            continue
        path = artifact_paths(output, row)[name]
        if not path.is_file() or benchmark_runtime.artifact(path) != expected:
            raise RuntimeError(f"existing artifact changed: {path}")


def failed_record(record, policy, horizon_ticks, run_id, error):
    invariants = {
        "agent_energy_never_drops_below_zero": False,
        "committed_reservations_never_overlap": False,
        "completed_tasks_are_never_reassigned": False,
        "incapable_agents_never_commit_tasks": False,
        "terminal_replay_hash_matches": False,
    }
    return {
        "allocator": policy,
        "artifact_hashes": {},
        "artifacts_valid": False,
        "error": str(error)[:2000],
        "excluded": False,
        "hard_invariant_violations": len(invariants),
        "hard_invariants": invariants,
        "makespan_ticks": horizon_ticks,
        "mission_success": False,
        "run_id": run_id,
        "replay_attempted": False,
        "replay_verified": False,
        "schema_version": 1,
        "seed": record["seed"],
        "seed_id": record["id"],
        "status": "invalid",
        "stratum": record["stratum"],
        "stratum_index": record["stratum_index"],
        "terminal_event_present": False,
    }


def run_one(record, policy, simulator, agent, supervisor, output, scratch, run_id, horizon_ticks):
    scenario_path = output / "scenarios" / record["id"] / f"{policy}.textproto"
    run_path = output / "runs" / record["id"] / policy
    scratch_path = scratch / record["id"] / policy if scratch else None
    relative_scenario = scenario_path.relative_to(output).as_posix()
    relative_run = run_path.relative_to(output).as_posix()
    try:
        from google.protobuf import text_format
        from generate_scenarios import generate_scenario, paired_digest

        scenario = generate_scenario(record, policy)
        scenario_path.parent.mkdir(parents=True, exist_ok=True)
        scenario_path.write_text(text_format.MessageToString(scenario, as_utf8=True), encoding="utf-8", newline="\n")
        values = benchmark_runtime.run_mission(
            simulator, agent, scenario_path, run_path, horizon_ticks, scratch_path, supervisor, len(scenario.vehicles)
        )
        row = {
            **values,
            "allocator": policy,
            "excluded": False,
            "paired_input_sha256": paired_digest(scenario),
            "run_id": run_id,
            "run_path": relative_run,
            "scenario_path": relative_scenario,
            "schema_version": 1,
            "seed": record["seed"],
            "seed_id": record["id"],
            "stratum": record["stratum"],
            "stratum_index": record["stratum_index"],
        }
        return row
    except Exception as error:
        row = failed_record(record, policy, horizon_ticks, run_id, error)
        row["run_path"] = relative_run
        row["scenario_path"] = relative_scenario
        if scenario_path.is_file():
            row["artifact_hashes"] = {"scenario": benchmark_runtime.artifact(scenario_path)}
        return row


def prepare_output(output, resume):
    if resume:
        if not output.is_dir():
            raise RuntimeError("resume output is missing")
    elif output.exists():
        raise RuntimeError("output already exists")
    else:
        output.mkdir(parents=True)


def clear_pending(output, scratch, pending):
    for record, policy in pending:
        for path in (
            output / "runs" / record["id"] / policy,
            scratch / record["id"] / policy if scratch else None,
        ):
            if path and path.exists():
                shutil.rmtree(path)


def validate_output(output):
    root = ROOT.resolve()
    resolved = output.resolve()
    try:
        relative = resolved.relative_to(root)
    except ValueError:
        return resolved
    if not relative.parts or relative.parts[0] != "out":
        raise RuntimeError("benchmark output inside the source tree must be under out")
    return resolved


def validate_manifest_output(manifest, output):
    if manifest.resolve().parent != output.resolve().parent:
        raise RuntimeError("benchmark manifest and output must share a directory")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", type=pathlib.Path, required=True)
    parser.add_argument("--agent", type=pathlib.Path, required=True)
    parser.add_argument("--supervisor", type=pathlib.Path)
    parser.add_argument("--corpus", choices=("development", "holdout"), default="development")
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--scratch", type=pathlib.Path)
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--shard-index", type=int, default=0)
    parser.add_argument("--shard-count", type=int, default=1)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--manifest", type=pathlib.Path)
    parser.add_argument("--container-image-digest")
    opts = parser.parse_args()
    if opts.jobs < 1 or opts.limit is not None and opts.limit < 1:
        raise RuntimeError("invalid run limits")
    if opts.corpus == "holdout":
        if not opts.manifest or not opts.container_image_digest or not opts.supervisor:
            raise RuntimeError("holdout requires a benchmark manifest, container image digest, and supervisor")
        if opts.resume or opts.limit is not None:
            raise RuntimeError("holdout runs use the complete corpus in a new output directory")
        if opts.shard_count != 1 or opts.shard_index != 0:
            raise RuntimeError("holdout runs use one complete shard")
    simulator = opts.sim.resolve(strict=True)
    agent = opts.agent.resolve(strict=True)
    supervisor = opts.supervisor.resolve(strict=True) if opts.supervisor else None
    output = validate_output(opts.output)
    scratch = opts.scratch.resolve() if opts.scratch else None
    run_id = ""
    manifest = None
    if opts.manifest:
        manifest = run_manifest.verify_manifest(
            opts.manifest,
            ROOT,
            simulator,
            agent,
            opts.container_image_digest or "",
            supervisor,
        )
        run_id = manifest["run_id"]
    if opts.corpus == "holdout":
        if output.exists():
            raise RuntimeError("holdout output already exists")
        validate_manifest_output(opts.manifest, output)
        if run_manifest.output_record_path(opts.manifest, run_id).exists():
            raise RuntimeError("benchmark manifest already has an output")
    from generate_scenarios import load_seed_records

    benchmark = json.loads((ROOT / "config" / "benchmark.json").read_text(encoding="utf-8"))
    records = select_records(
        load_seed_records(opts.corpus), opts.shard_index, opts.shard_count, opts.limit
    )
    if opts.corpus == "holdout":
        run_manifest.record_output(opts.manifest, manifest, output)
    prepare_output(output, opts.resume)
    if scratch:
        scratch.mkdir(parents=True, exist_ok=True)
    policies = tuple(benchmark["allocators"])
    records_by_id = {record["id"]: record for record in records}
    work = [(record, policy) for record in records for policy in policies]
    path = result_path(output, opts.shard_index, opts.shard_count)
    journal = journal_root(output, opts.shard_index, opts.shard_count)
    existing = read_results(path) if opts.resume else {}
    if opts.resume:
        for identity, row in read_journal(journal).items():
            if identity in existing and existing[identity] != row:
                raise RuntimeError(f"journal conflicts with existing result: {identity}")
            existing[identity] = row
    expected = {(record["id"], policy) for record, policy in work}
    if set(existing) - expected:
        raise RuntimeError("resume results do not belong to this shard")
    for row in existing.values():
        verify_existing(output, row)
    pending = [(record, policy) for record, policy in work if (record["id"], policy) not in existing]
    if opts.resume:
        clear_pending(output, scratch, pending)
    rows = dict(existing)
    horizon_ticks = benchmark["clock"]["mission_horizon_ticks"]
    with concurrent.futures.ProcessPoolExecutor(max_workers=opts.jobs) as executor:
        futures = {
            executor.submit(
                run_one, record, policy, simulator, agent, supervisor, output, scratch, run_id, horizon_ticks
            ): (record["id"], policy)
            for record, policy in pending
        }
        for future in concurrent.futures.as_completed(futures):
            seed_id, policy = futures[future]
            record = records_by_id[seed_id]
            try:
                row = future.result()
            except Exception as error:
                row = failed_record(record, policy, horizon_ticks, run_id, error)
                row["run_path"] = f"runs/{seed_id}/{policy}"
                row["scenario_path"] = f"scenarios/{seed_id}/{policy}.textproto"
            write_journal(journal, row)
            rows[key(row)] = row
    write_results(path, rows.values())
    print(json.dumps({"result_file": str(path), "rows": len(rows)}, sort_keys=True))


if __name__ == "__main__":
    main()
