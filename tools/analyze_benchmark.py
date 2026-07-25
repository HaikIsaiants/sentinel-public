import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import pathlib

from benchmark_io import canonical_bytes, file_digest, output_record_path, read_output_record, validate_image_digest


ROOT = pathlib.Path(__file__).resolve().parents[1]
REQUIRED_ARTIFACTS = {"event_log", "replay_summary", "scenario", "summary"}
SUMMARY_FIELDS = {
    "active_agents": "active_agents",
    "agent_energy_below_zero_violations": "agent_energy_below_zero_violations",
    "agent_energy_never_drops_below_zero": "agent_energy_never_drops_below_zero",
    "allocation_convergence": "allocation_convergence",
    "communication_bytes": "communication_bytes",
    "communication_messages": "communication_messages",
    "completed_tasks": "completed_tasks",
    "completed_task_reassignment_violations": "completed_task_reassignment_violations",
    "completed_tasks_are_never_reassigned": "completed_tasks_are_never_reassigned",
    "committed_reservation_overlap_violations": "committed_reservation_overlap_violations",
    "committed_reservations_never_overlap": "committed_reservations_never_overlap",
    "delivered_messages": "delivered_messages",
    "dropped_messages": "dropped_messages",
    "energy_consumed_mj": "energy_consumed_mj",
    "failure_detections": "failure_detections",
    "incapable_agent_commit_violations": "incapable_agent_commit_violations",
    "incapable_agents_never_commit_tasks": "incapable_agents_never_commit_tasks",
    "maximum_reassignment_delay_ms": "maximum_reassignment_delay_ms",
    "missing_reassignments": "missing_reassignments",
    "recharge_ticks": "recharge_ticks",
    "rejected_commits": "rejected_commits",
    "reordered_messages": "reordered_messages",
    "replan_count": "replan_count",
    "replanning_samples": "replanning_samples",
    "return_ticks": "return_ticks",
    "route_conflicts": "route_conflicts",
    "summary_success": "success",
    "task_reassignments": "task_reassignments",
    "terminal_hash": "terminal_hash",
    "ticks": "ticks",
    "total_tasks": "total_tasks",
    "travel_distance_mm": "travel_distance_mm",
    "wait_ticks": "wait_ticks",
}
INVARIANT_FIELDS = {
    "agent_energy_never_drops_below_zero": "agent_energy_below_zero_violations",
    "committed_reservations_never_overlap": "committed_reservation_overlap_violations",
    "completed_tasks_are_never_reassigned": "completed_task_reassignment_violations",
    "incapable_agents_never_commit_tasks": "incapable_agent_commit_violations",
}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def wilson_lower(successes, total, z):
    if total == 0:
        return 0.0
    probability = successes / total
    square = z * z
    return (
        probability
        + square / (2 * total)
        - z * math.sqrt(probability * (1 - probability) / total + square / (4 * total * total))
    ) / (1 + square / total)


def nearest_rank_p95(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[math.ceil(0.95 * len(ordered)) - 1]


def describe(values, expected=None):
    count = len(values)
    return {
        "maximum": max(values) if values else None,
        "mean": sum(values) / count if values else None,
        "missing_count": max(0, expected - count) if expected is not None else 0,
        "p95": nearest_rank_p95(values),
        "sample_count": count,
    }


def stratified(values, strata, expected=None):
    result = describe([value for _, value in values], sum(expected.values()) if expected else None)
    result["per_stratum"] = {
        stratum: describe(
            [value for sample_stratum, value in values if sample_stratum == stratum],
            expected[stratum] if expected else None,
        )
        for stratum in strata
    }
    aggregate = {}
    for field in ("maximum", "mean", "p95"):
        samples = [result["per_stratum"][stratum][field] for stratum in strata]
        aggregate[field] = sum(samples) / len(samples) if samples and all(value is not None for value in samples) else None
    aggregate["represented_strata"] = sum(result["per_stratum"][stratum]["sample_count"] > 0 for stratum in strata)
    aggregate["strata_count"] = len(strata)
    result["equal_weighted_stratum_aggregate"] = aggregate
    result["equal_weighted_stratum_mean"] = aggregate["mean"]
    return result


def row_identity(row):
    if not isinstance(row, dict) or not isinstance(row.get("seed_id"), str) or not isinstance(row.get("allocator"), str):
        return None
    return row["seed_id"], row["allocator"]


def invariant_failures(row, invariants):
    evidence = row.get("hard_invariants", {})
    observed = sum(evidence.get(name) is not True for name in invariants)
    reported = row.get("hard_invariant_violations", observed)
    return max(observed, reported) if isinstance(reported, int) and reported >= 0 else observed + 1


def successful(row, invariants):
    return bool(
        row.get("mission_success") is True
        and row.get("status") == "terminated"
        and row.get("summary_success") is True
        and row.get("terminal_event_present") is True
        and row.get("replay_verified") is True
        and invariant_failures(row, invariants) == 0
    )


def normalize(rows, seeds, allocators, invariants, run_id, artifact_failure_ids=()):
    seed_map = {record["id"]: record for record in seeds}
    groups = {}
    invalid = 0
    unknown = 0
    for row in rows:
        identity = row_identity(row)
        if identity is None:
            invalid += 1
            continue
        if identity[0] not in seed_map or identity[1] not in allocators:
            unknown += 1
            continue
        groups.setdefault(identity, []).append(row)
    duplicates = sum(max(0, len(group) - 1) for group in groups.values())
    chosen = {identity: min(group, key=canonical) for identity, group in groups.items()}
    expected = [(record, allocator) for record in seeds for allocator in allocators]
    missing = sum((record["id"], allocator) not in chosen for record, allocator in expected)
    stratum_mismatches = sum(
        row.get("stratum") != seed_map[identity[0]]["stratum"]
        for identity, row in chosen.items()
    )
    seed_mismatches = sum(
        type(row.get("seed")) is not int or row["seed"] != seed_map[identity[0]]["seed"]
        for identity, row in chosen.items()
    )
    stratum_index_mismatches = sum(
        type(row.get("stratum_index")) is not int
        or row["stratum_index"] != seed_map[identity[0]]["stratum_index"]
        for identity, row in chosen.items()
    )
    run_mismatches = sum(
        bool(run_id) and row.get("run_id", row.get("release_id")) != run_id
        for row in chosen.values()
    )
    artifact_failure_ids = set(artifact_failure_ids)
    artifact_failures = sum(
        row.get("artifacts_valid") is not True or identity in artifact_failure_ids
        for identity, row in chosen.items()
    )
    exclusions = sum(bool(row.get("excluded")) for row in chosen.values())
    observed_invariant_violations = sum(
        invariant_failures(row, invariants)
        for row in chosen.values()
        if row.get("terminal_event_present") is True
    )
    unverified_invariant_checks = sum(
        invariant_failures(row, invariants)
        for row in chosen.values()
        if row.get("terminal_event_present") is not True
    )
    paired_mismatches = 0
    for record in seeds:
        pair = [chosen.get((record["id"], allocator)) for allocator in allocators]
        digests = {row.get("paired_input_sha256") for row in pair if row}
        if len(pair) != len(allocators) or any(row is None for row in pair) or len(digests) != 1 or not next(iter(digests), ""):
            paired_mismatches += 1
    effective = {
        (record["id"], allocator): chosen.get((record["id"], allocator))
        for record, allocator in expected
    }
    complete = not any(
        (missing, duplicates, invalid, unknown, stratum_mismatches, seed_mismatches,
         stratum_index_mismatches, run_mismatches, artifact_failures, paired_mismatches)
    )
    accounting = {
        "artifact_failures": artifact_failures,
        "complete_pairs": complete,
        "duplicate_rows": duplicates,
        "expected_rows": len(expected),
        "observed_invariant_violations": observed_invariant_violations,
        "invalid_rows": invalid,
        "missing_rows": missing,
        "observed_rows": len(rows),
        "paired_input_mismatches": paired_mismatches,
        "post_run_exclusions": exclusions,
        "run_mismatches": run_mismatches,
        "seed_mismatches": seed_mismatches,
        "stratum_index_mismatches": stratum_index_mismatches,
        "stratum_mismatches": stratum_mismatches,
        "unique_expected_rows": len(chosen),
        "unknown_rows": unknown,
        "unverified_invariant_checks": unverified_invariant_checks,
    }
    return effective, accounting


def metric_rows(effective, seeds, allocator):
    return [(record, effective[(record["id"], allocator)]) for record in seeds]


def mission_value(row, field, scale):
    if row is None or not isinstance(row.get(field), (int, float)):
        return None
    return row[field] * scale


def event_durations(rows, field, horizon, step_seconds):
    values = []
    for record, row in rows:
        if row is None:
            continue
        for sample in row.get(field, []):
            start = int(sample.get("start_tick", 0))
            end = int(sample.get("end_tick", 0)) if sample.get("complete") else horizon
            values.append((record["stratum"], max(0, end - start) * step_seconds))
    return values


def support_for_allocator(rows, strata, horizon, step_seconds):
    mission_fields = {
        "communication_bytes": ("communication_bytes", 1),
        "communication_messages": ("communication_messages", 1),
        "energy_consumption_joules": ("energy_consumed_mj", 0.001),
        "route_conflicts": ("route_conflicts", 1),
        "travel_distance_meters": ("travel_distance_mm", 0.001),
    }
    result = {}
    expected = {stratum: sum(record["stratum"] == stratum for record, _ in rows) for stratum in strata}
    for label, (field, scale) in mission_fields.items():
        values = [
            (record["stratum"], value)
            for record, row in rows
            if (value := mission_value(row, field, scale)) is not None
        ]
        result[label] = stratified(values, strata, expected)
    result["allocation_convergence_seconds"] = stratified(
        event_durations(rows, "allocation_convergence", horizon, step_seconds), strata
    )
    result["replanning_latency_seconds"] = stratified(
        event_durations(rows, "replanning_samples", horizon, step_seconds), strata
    )
    detection_values = []
    reassignment_values = []
    for record, row in rows:
        if row is None:
            continue
        for sample in row.get("failure_detections", []):
            detection_values.append(
                (record["stratum"], max(0, sample["detection_tick"] - sample["failure_tick"]) * step_seconds)
            )
        for sample in row.get("task_reassignments", []):
            if sample.get("complete"):
                reassignment_values.append(
                    (record["stratum"], max(0, sample["commit_tick"] - sample["detection_tick"]) * step_seconds)
                )
    result["failure_detection_latency_seconds"] = stratified(detection_values, strata)
    result["reassignment_delay_seconds"] = stratified(reassignment_values, strata)
    return result


def analyze(rows, seeds, benchmark, corpus, run_id="", artifact_failure_ids=()):
    allocators = tuple(benchmark["allocators"])
    invariants = tuple(benchmark["hard_invariants"])
    artifact_failure_ids = set(artifact_failure_ids)
    effective, accounting = normalize(rows, seeds, allocators, invariants, run_id, artifact_failure_ids)
    horizon = benchmark["clock"]["mission_horizon_ticks"]
    step_seconds = benchmark["clock"]["tick_seconds"]
    step_ms = round(step_seconds * 1000)
    degraded = set(benchmark["degraded_communications_strata"])
    z = benchmark["wilson_z"]
    strata = tuple(dict.fromkeys(record["stratum"] for record in seeds))
    primary = {"allocators": {}, "by_stratum": {stratum: {} for stratum in strata}}
    supporting = {}
    makespan_sums = {}
    for allocator in allocators:
        selected = metric_rows(effective, seeds, allocator)
        trusted = [
            row is not None
            and row.get("artifacts_valid") is True
            and (record["id"], allocator) not in artifact_failure_ids
            for record, row in selected
        ]
        successes = [successful(row, invariants) if valid else False for (_, row), valid in zip(selected, trusted)]
        makespans = [
            min(horizon, max(0, int(row.get("ticks", horizon)))) if success and row else horizon
            for (_, row), success in zip(selected, successes)
        ]
        degraded_success = sum(
            success for (record, _), success in zip(selected, successes) if record["stratum"] in degraded
        )
        degraded_count = sum(record["stratum"] in degraded for record, _ in selected)
        makespan_sums[allocator] = sum(makespans)
        primary["allocators"][allocator] = {
            "completion_rate": sum(successes) / len(successes) if successes else 0,
            "degraded_completion_rate": degraded_success / degraded_count if degraded_count else 0,
            "degraded_successes": degraded_success,
            "degraded_total": degraded_count,
            "degraded_wilson_lower": wilson_lower(degraded_success, degraded_count, z),
            "makespan_mean_seconds": sum(makespans) * step_seconds / len(makespans) if makespans else 0,
            "makespan_sum_ticks": sum(makespans),
            "mission_count": len(selected),
            "mission_successes": sum(successes),
        }
        for stratum in strata:
            idxs = [idx for idx, (record, _) in enumerate(selected) if record["stratum"] == stratum]
            primary["by_stratum"][stratum][allocator] = {
                "completion_rate": sum(successes[idx] for idx in idxs) / len(idxs) if idxs else 0,
                "makespan_mean_seconds": sum(makespans[idx] for idx in idxs) * step_seconds / len(idxs) if idxs else 0,
                "missions": len(idxs),
            }
        supporting[allocator] = support_for_allocator(
            [(record, row if valid else None) for (record, row), valid in zip(selected, trusted)],
            strata,
            horizon,
            step_seconds,
        )
    sentinel = "sentinel_cbba"
    baseline = "nearest_capable"
    sentinel_sum = makespan_sums[sentinel]
    baseline_sum = makespan_sums[baseline]
    reduction = 1 - sentinel_sum / baseline_sum if baseline_sum else 0
    primary["paired_makespan_reduction"] = reduction
    maximum_delay_ms = 0
    missing_reassignments = 0
    metric_mismatches = 0
    for identity, row in effective.items():
        if row is None:
            continue
        samples = row.get("task_reassignments", [])
        derived_delays = [
            max(0, sample["commit_tick"] - sample["detection_tick"]) * step_ms
            for sample in samples if sample.get("complete")
        ]
        derived_maximum = max(derived_delays, default=0)
        derived_missing = sum(not sample.get("complete") for sample in samples)
        if identity[1] == sentinel:
            maximum_delay_ms = max(maximum_delay_ms, derived_maximum)
            missing_reassignments += derived_missing
        if row.get("status") == "terminated":
            metric_mismatches += row.get("maximum_reassignment_delay_ms") != derived_maximum
            metric_mismatches += row.get("missing_reassignments") != derived_missing
            trusted = row.get("artifacts_valid") is True and identity not in artifact_failure_ids
            expected_makespan = int(row.get("ticks", horizon)) if trusted and successful(row, invariants) else horizon
            metric_mismatches += row.get("makespan_ticks") != expected_makespan
    accounting["metric_mismatches"] = metric_mismatches
    accounting["complete_pairs"] = accounting["complete_pairs"] and metric_mismatches == 0
    run_counts = {
        allocator: sum(effective[(record["id"], allocator)] is not None for record in seeds)
        for allocator in allocators
    }
    data_integrity = {
        "complete_seed_pairs": accounting["complete_pairs"],
        "no_post_run_exclusions": accounting["post_run_exclusions"] == 0,
    }
    data_integrity["valid"] = all(data_integrity.values())
    ordered_rows = sorted((canonical(row) for row in rows))
    results_digest = hashlib.sha256(("\n".join(ordered_rows) + ("\n" if ordered_rows else "")).encode()).hexdigest()
    return {
        "accounting": {**accounting, "run_counts": run_counts},
        "benchmark_id": benchmark["benchmark_id"],
        "benchmark_version": benchmark["version"],
        "corpus": corpus,
        "data_integrity": data_integrity,
        "primary_metrics": primary,
        "reassignment": {
            "maximum_delay_seconds": maximum_delay_ms / 1000,
            "missing": missing_reassignments,
        },
        "run_id": run_id,
        "results_sha256": results_digest,
        "schema_version": 2,
        "supporting_metrics": supporting,
    }


def read_rows(paths):
    rows = []
    roots = []
    for path in paths:
        values = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
        rows.extend(values)
        roots.extend([path.parent.resolve()] * len(values))
    return rows, roots


def contained(root, relative):
    path = (root / relative).resolve()
    path.relative_to(root)
    return path


def artifact_paths(root, row):
    run = contained(root, row["run_path"])
    scenario = contained(root, row["scenario_path"])
    return {
        "event_log": run / "events.pb.gz",
        "replay_summary": run / "replay.json",
        "scenario": scenario,
        "summary": run / "summary.json",
    }


def artifact_matches(path, expected):
    return bool(
        isinstance(expected, dict)
        and path.is_file()
        and expected.get("name") == path.name
        and expected.get("size") == path.stat().st_size
        and expected.get("sha256") == file_digest(path)
    )


def scenario_matches(path, row, record):
    from google.protobuf import text_format
    from sentinel.v1 import sentinel_pb2
    from generate_scenarios import generate_scenario, paired_digest

    scenario = sentinel_pb2.Scenario()
    text_format.Parse(path.read_text(encoding="utf-8"), scenario)
    expected = generate_scenario(record, row["allocator"], False)
    return bool(
        scenario.SerializeToString(deterministic=True) == expected.SerializeToString(deterministic=True)
        and row.get("paired_input_sha256") == paired_digest(scenario)
    )


def evidence_matches(row, summary, replay, invariants):
    if (
        not isinstance(summary, dict)
        or not isinstance(replay, dict)
        or canonical(summary) != canonical(replay)
        or not set(SUMMARY_FIELDS.values()).issubset(summary)
        or not set(SUMMARY_FIELDS).issubset(row)
        or not isinstance(summary.get("terminal_hash"), str)
        or not summary["terminal_hash"]
    ):
        return False
    evidence = row.get("hard_invariants")
    if not isinstance(evidence, dict) or set(evidence) != set(invariants) or any(type(value) is not bool for value in evidence.values()):
        return False
    execution = row.get("terminal_event_present")
    observed = all(
        type(summary.get(counter)) is int
        and summary[counter] >= 0
        and type(summary.get(name)) is bool
        and summary[name] is (summary[counter] == 0)
        and evidence.get(name) is summary[name]
        for name, counter in INVARIANT_FIELDS.items()
    )
    expected_success = summary["success"] is True and all(evidence.values())
    return bool(
        execution is True
        and observed
        and evidence.get("terminal_replay_hash_matches") is True
        and type(row.get("hard_invariant_violations")) is int
        and row["hard_invariant_violations"] == sum(not value for value in evidence.values())
        and row.get("artifacts_valid") is True
        and row.get("mission_success") is expected_success
        and row.get("replay_attempted") is True
        and row.get("replay_verified") is True
        and row.get("status") == "terminated"
        and all(canonical(row[field]) == canonical(summary[source]) for field, source in SUMMARY_FIELDS.items())
    )


def audit_artifacts(rows, roots, invariants=(), records=()):
    """Return run identities whose saved artifacts fail a cross-check."""

    record_map = {record["id"]: record for record in records}

    def check(row, root):
        identity = row_identity(row)
        try:
            expected = row.get("artifact_hashes", {})
            paths = artifact_paths(root, row)
            valid = set(expected) == REQUIRED_ARTIFACTS and all(
                artifact_matches(path, expected[name]) for name, path in paths.items()
            )
            if valid and record_map:
                valid = scenario_matches(paths["scenario"], row, record_map[row["seed_id"]])
            if valid:
                summary = json.loads(paths["summary"].read_text(encoding="utf-8"))
                replay = json.loads(paths["replay_summary"].read_text(encoding="utf-8"))
                valid = evidence_matches(row, summary, replay, invariants or row.get("hard_invariants", ()))
        except (KeyError, OSError, TypeError, ValueError):
            valid = False
        return identity, valid

    failures = set()
    with ThreadPoolExecutor(max_workers=8) as executor:
        for identity, valid in executor.map(check, rows, roots):
            if identity and not valid:
                failures.add(identity)
    return failures


def manifest_metadata(path, container_image_digest="", require_output=False):
    manifest = json.loads(path.read_text(encoding="utf-8"))
    legacy = "run_id" not in manifest and "release_id" in manifest
    id_field = "release_id" if legacy else "run_id"
    run_id = manifest[id_field]
    payload = {name: item for name, item in manifest.items() if name != id_field}
    if hashlib.sha256(canonical_bytes(payload)).hexdigest() != run_id:
        raise RuntimeError("invalid benchmark manifest")
    if container_image_digest and manifest.get("container_image_digest") != validate_image_digest(container_image_digest):
        raise RuntimeError("benchmark manifest container image digest does not match")
    record_digest = ""
    recorded_output = ""
    if require_output:
        record, output_record = read_output_record(path, run_id, container_image_digest, legacy)
        record_digest = file_digest(record)
        recorded_output = output_record["output"]
        if output_record.get("schema_version") != 1:
            raise RuntimeError("invalid benchmark output record")
    return run_id, record_digest, recorded_output


def verify_result_roots(roots, recorded_output):
    expected = pathlib.Path(recorded_output).resolve()
    if not roots or {root.resolve() for root in roots} != {expected}:
        raise RuntimeError("holdout results do not match the recorded output")


def markdown(report):
    sentinel = report["primary_metrics"]["allocators"]["sentinel_cbba"]
    baseline = report["primary_metrics"]["allocators"]["nearest_capable"]
    accounting = report["accounting"]
    lines = [
        "# Sentinel Benchmark Report",
        "",
        f"Corpus: `{report['corpus']}`",
        "## Measurements",
        "",
        f"Sentinel completion: {sentinel['mission_successes']}/{sentinel['mission_count']}",
        f"Baseline completion: {baseline['mission_successes']}/{baseline['mission_count']}",
        f"Degraded Wilson lower bound: {sentinel['degraded_wilson_lower']:.6f}",
        f"Paired makespan reduction: {report['primary_metrics']['paired_makespan_reduction']:.6%}",
        f"Maximum reassignment delay: {report['reassignment']['maximum_delay_seconds']:.3f} seconds",
        f"Missing reassignments: {report['reassignment']['missing']}",
        "",
        "## Run accounting",
        "",
        f"Rows: {accounting['observed_rows']}/{accounting['expected_rows']}",
        f"Complete pairs: {str(report['data_integrity']['complete_seed_pairs']).lower()}",
        f"Invariant violations: {accounting['observed_invariant_violations']}",
    ]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=pathlib.Path, nargs="+", required=True)
    parser.add_argument("--corpus", choices=("development", "holdout"), required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--markdown", type=pathlib.Path)
    parser.add_argument("--manifest", type=pathlib.Path)
    parser.add_argument("--container-image-digest")
    args = parser.parse_args()
    if args.corpus == "holdout" and (not args.manifest or not args.container_image_digest):
        raise RuntimeError("holdout analysis requires a benchmark manifest and container image digest")
    run_id, output_record_digest, recorded_output = manifest_metadata(
        args.manifest,
        args.container_image_digest or "",
        args.corpus == "holdout",
    ) if args.manifest else ("", "", "")
    from generate_scenarios import load_seed_records

    benchmark = json.loads((ROOT / "config" / "benchmark.json").read_text(encoding="utf-8"))
    records = load_seed_records(args.corpus)
    rows, roots = read_rows(args.results)
    if args.corpus == "holdout":
        verify_result_roots(roots, recorded_output)
    report = analyze(
        rows,
        records,
        benchmark,
        args.corpus,
        run_id,
        audit_artifacts(rows, roots, benchmark["hard_invariants"], records),
    )
    report["output_record_sha256"] = output_record_digest
    if args.output.exists() or args.markdown and args.markdown.exists():
        raise RuntimeError("analysis output already exists")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(markdown(report), encoding="utf-8")
    print(json.dumps({"data_integrity_valid": report["data_integrity"]["valid"], "output": str(args.output)}, sort_keys=True))
    if args.corpus == "holdout" and not report["data_integrity"]["valid"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
