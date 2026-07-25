import copy
import gzip
import hashlib
import io
import json
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

from google.protobuf import text_format

import analyze_benchmark
import benchmark_io
import benchmark_runtime
import generate_scenarios
import run_benchmark
import run_manifest


ROOT = pathlib.Path(__file__).resolve().parents[1]
STRATA = (
    "nominal",
    "latency_jitter",
    "packet_loss_reordering",
    "bandwidth_limited",
    "temporary_partition",
    "blocked_route",
    "dynamic_task_arrival",
    "agent_loss_degradation",
    "combined_communications",
    "compound_disruption",
)


def benchmark_config():
    value = json.loads((ROOT / "config" / "benchmark.json").read_text(encoding="utf-8"))
    value["clock"]["mission_horizon_ticks"] = 1000
    value["clock"]["mission_horizon_seconds"] = 100.0
    value["corpora"]["holdout"]["missions"] = 200
    return value


def seeds():
    return [
        {
            "id": f"synthetic-{index:05d}",
            "seed": index + 1,
            "stratum": STRATA[index % len(STRATA)],
            "stratum_index": index // len(STRATA),
        }
        for index in range(200)
    ]


def row(record, allocator, invariants):
    ticks = 91 if allocator == "sentinel_cbba" else 113
    return {
        "active_agents": 4,
        "agent_energy_below_zero_violations": 0,
        "agent_energy_never_drops_below_zero": True,
        "allocation_convergence": [{"complete": True, "end_tick": 4, "epoch": 1, "start_tick": 1}],
        "allocator": allocator,
        "artifact_hashes": {"event_log": {"sha256": "a"}},
        "artifacts_valid": True,
        "communication_bytes": 1000,
        "communication_messages": 10,
        "completed_tasks": 12,
        "completed_task_reassignment_violations": 0,
        "completed_tasks_are_never_reassigned": True,
        "committed_reservation_overlap_violations": 0,
        "committed_reservations_never_overlap": True,
        "delivered_messages": 9,
        "dropped_messages": 1,
        "energy_consumed_mj": 2000,
        "error": "",
        "excluded": False,
        "failure_detections": [{"detection_tick": 10, "failure_tick": 5}],
        "hard_invariant_violations": 0,
        "hard_invariants": {name: True for name in invariants},
        "incapable_agent_commit_violations": 0,
        "incapable_agents_never_commit_tasks": True,
        "makespan_ticks": ticks,
        "maximum_reassignment_delay_ms": 1200,
        "mission_success": True,
        "missing_reassignments": 0,
        "paired_input_sha256": f"pair-{record['id']}",
        "recharge_ticks": 0,
        "rejected_commits": 0,
        "run_id": "run",
        "reordered_messages": 0,
        "replan_count": 1,
        "replanning_samples": [{"complete": False, "end_tick": 0, "start_tick": 990}],
        "replay_attempted": True,
        "replay_verified": True,
        "return_ticks": 3,
        "route_conflicts": 2,
        "seed": record["seed"],
        "seed_id": record["id"],
        "status": "terminated",
        "stratum": record["stratum"],
        "stratum_index": record["stratum_index"],
        "summary_success": True,
        "task_reassignments": [{"commit_tick": 22, "complete": True, "detection_tick": 10}],
        "terminal_event_present": True,
        "terminal_hash": f"terminal-{record['id']}-{allocator}",
        "ticks": ticks,
        "total_tasks": 12,
        "travel_distance_mm": 3000,
        "wait_ticks": 2,
    }


def rows(records, value):
    invariants = value["hard_invariants"]
    return [row(record, allocator, invariants) for record in records for allocator in value["allocators"]]


def summary(value):
    result = {source: copy.deepcopy(value[field]) for field, source in analyze_benchmark.SUMMARY_FIELDS.items()}
    result["tick_hashes"] = [value["terminal_hash"]]
    return result


class AnalyzerTests(unittest.TestCase):
    def setUp(self):
        self.benchmark = benchmark_config()
        self.seeds = seeds()
        self.rows = rows(self.seeds, self.benchmark)

    def analyze(self, values=None, value=None, artifact_failure_ids=()):
        return analyze_benchmark.analyze(
            self.rows if values is None else values,
            self.seeds,
            self.benchmark if value is None else value,
            "holdout",
            "run",
            artifact_failure_ids,
        )

    def test_reports_measurements(self):
        report = self.analyze()
        self.assertTrue(report["data_integrity"]["valid"])
        self.assertAlmostEqual(report["primary_metrics"]["paired_makespan_reduction"], 1 - 91 / 113)
        self.assertEqual(report["reassignment"]["maximum_delay_seconds"], 1.2)
        self.assertEqual(report["supporting_metrics"]["sentinel_cbba"]["travel_distance_meters"]["mean"], 3.0)
        support = report["supporting_metrics"]["sentinel_cbba"]
        self.assertAlmostEqual(support["allocation_convergence_seconds"]["per_stratum"]["nominal"]["mean"], 0.3)
        self.assertEqual(support["replanning_latency_seconds"]["equal_weighted_stratum_aggregate"]["mean"], 1.0)
        self.assertEqual(support["failure_detection_latency_seconds"]["equal_weighted_stratum_mean"], 0.5)
        self.assertAlmostEqual(support["reassignment_delay_seconds"]["equal_weighted_stratum_mean"], 1.2)
    def test_strategy_metrics_change_without_changing_data_integrity(self):
        values = copy.deepcopy(self.rows)
        sentinel = next(value for value in values if value["allocator"] == "sentinel_cbba")
        sentinel["ticks"] += 1
        sentinel["makespan_ticks"] += 1
        report = self.analyze(values)
        self.assertTrue(report["data_integrity"]["valid"])
        self.assertNotEqual(report["primary_metrics"]["paired_makespan_reduction"], 1 - 91 / 113)

    def test_reassignment_values_are_observations(self):
        values = copy.deepcopy(self.rows)
        sentinel = next(value for value in values if value["allocator"] == "sentinel_cbba")
        sentinel["maximum_reassignment_delay_ms"] = 1500
        sentinel["task_reassignments"][0]["commit_tick"] = 25
        report = self.analyze(values)
        self.assertEqual(report["reassignment"]["maximum_delay_seconds"], 1.5)
        self.assertTrue(report["data_integrity"]["valid"])

        values = copy.deepcopy(self.rows)
        sentinel = next(value for value in values if value["allocator"] == "sentinel_cbba")
        sentinel["task_reassignments"][0]["complete"] = False
        sentinel["task_reassignments"][0]["commit_tick"] = 0
        sentinel["maximum_reassignment_delay_ms"] = 0
        sentinel["missing_reassignments"] = 1
        report = self.analyze(values)
        self.assertEqual(report["reassignment"]["missing"], 1)
        self.assertTrue(report["data_integrity"]["valid"])

    def test_baseline_reassignment_is_reported_separately(self):
        values = copy.deepcopy(self.rows)
        baseline = next(value for value in values if value["allocator"] == "nearest_capable")
        baseline["maximum_reassignment_delay_ms"] = 1500
        baseline["task_reassignments"][0]["commit_tick"] = 25
        report = self.analyze(values)
        self.assertEqual(report["reassignment"]["maximum_delay_seconds"], 1.2)
        self.assertEqual(
            report["supporting_metrics"]["nearest_capable"]["reassignment_delay_seconds"]["maximum"],
            1.5,
        )
        self.assertTrue(report["data_integrity"]["valid"])

    def test_wilson_bound_is_reported_as_a_statistic(self):
        z = self.benchmark["wilson_z"]
        lower = analyze_benchmark.wilson_lower(73, 100, z)
        self.assertGreater(lower, 0)
        self.assertLess(lower, 0.73)

    def test_accounting_detects_missing_duplicate_unknown_and_pair_mismatch(self):
        values = copy.deepcopy(self.rows[1:])
        values.append(copy.deepcopy(values[0]))
        unknown = copy.deepcopy(values[0])
        unknown["seed_id"] = "unknown"
        values.append(unknown)
        values[1]["paired_input_sha256"] = "mismatch"
        report = self.analyze(values)
        self.assertEqual(report["accounting"]["missing_rows"], 1)
        self.assertEqual(report["accounting"]["duplicate_rows"], 1)
        self.assertEqual(report["accounting"]["unknown_rows"], 1)
        self.assertGreater(report["accounting"]["paired_input_mismatches"], 0)
        self.assertFalse(report["data_integrity"]["valid"])

    def test_exclusions_affect_integrity_but_mission_outcomes_do_not(self):
        values = copy.deepcopy(self.rows)
        values[0]["excluded"] = True
        self.assertFalse(self.analyze(values)["data_integrity"]["valid"])

        values = copy.deepcopy(self.rows)
        failed = next(value for value in values if value["allocator"] == "sentinel_cbba")
        failed["hard_invariants"]["agent_energy_never_drops_below_zero"] = False
        failed["hard_invariant_violations"] = 1
        failed["mission_success"] = False
        failed["summary_success"] = False
        failed["makespan_ticks"] = self.benchmark["clock"]["mission_horizon_ticks"]
        report = self.analyze(values)
        self.assertEqual(report["accounting"]["observed_invariant_violations"], 1)
        self.assertEqual(report["accounting"]["unverified_invariant_checks"], 0)
        self.assertEqual(report["primary_metrics"]["allocators"]["sentinel_cbba"]["mission_successes"], 199)
        self.assertTrue(report["data_integrity"]["valid"])

    def test_identity_artifact_and_metric_mismatches_affect_integrity(self):
        fields = (
            ("run_id", "other", "run_mismatches"),
            ("stratum", "other", "stratum_mismatches"),
            ("seed", self.rows[0]["seed"] + 1, "seed_mismatches"),
            ("stratum_index", self.rows[0]["stratum_index"] + 1, "stratum_index_mismatches"),
        )
        for field, changed, counter in fields:
            with self.subTest(field=field):
                values = copy.deepcopy(self.rows)
                values[0][field] = changed
                report = self.analyze(values)
                self.assertEqual(report["accounting"][counter], 1)
                self.assertFalse(report["data_integrity"]["valid"])

        identity = (self.rows[0]["seed_id"], self.rows[0]["allocator"])
        report = self.analyze(artifact_failure_ids={identity})
        self.assertEqual(report["accounting"]["artifact_failures"], 1)
        self.assertFalse(report["data_integrity"]["valid"])

        values = copy.deepcopy(self.rows)
        values[0]["maximum_reassignment_delay_ms"] = 1100
        report = self.analyze(values)
        self.assertEqual(report["accounting"]["metric_mismatches"], 1)
        self.assertFalse(report["data_integrity"]["valid"])

    def test_analysis_is_independent_of_input_order(self):
        self.assertEqual(self.analyze(), self.analyze(list(reversed(self.rows))))

    def test_old_result_rows_keep_their_run_identity(self):
        values = copy.deepcopy(self.rows)
        for value in values:
            value["release_id"] = value.pop("run_id")
        report = self.analyze(values)
        self.assertEqual(report["accounting"]["run_mismatches"], 0)
        self.assertTrue(report["data_integrity"]["valid"])

    def test_terminal_violation_is_valid_negative_evidence(self):
        value = copy.deepcopy(self.rows[0])
        value["committed_reservation_overlap_violations"] = 1
        value["committed_reservations_never_overlap"] = False
        value["hard_invariants"]["committed_reservations_never_overlap"] = False
        value["hard_invariant_violations"] = 1
        value["mission_success"] = False
        evidence = summary(value)
        self.assertTrue(analyze_benchmark.evidence_matches(value, evidence, evidence, self.benchmark["hard_invariants"]))

    def test_invalid_holdout_analysis_writes_report_and_exits_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory).resolve()
            output = root / "report.json"
            report = {"data_integrity": {"valid": False}}
            arguments = [
                "analyze_benchmark.py",
                "--results", str(root / "results.jsonl"),
                "--corpus", "holdout",
                "--manifest", str(root / "manifest.json"),
                "--container-image-digest", "sha256:" + "a" * 64,
                "--output", str(output),
            ]
            with (
                mock.patch.object(sys, "argv", arguments),
                mock.patch.object(analyze_benchmark, "manifest_metadata", return_value=("run", "record", str(root))),
                mock.patch.object(generate_scenarios, "load_seed_records", return_value=[]),
                mock.patch.object(analyze_benchmark, "read_rows", return_value=([], [root])),
                mock.patch.object(analyze_benchmark, "verify_result_roots"),
                mock.patch.object(analyze_benchmark, "audit_artifacts", return_value=set()),
                mock.patch.object(analyze_benchmark, "analyze", return_value=report),
            ):
                with self.assertRaises(SystemExit) as raised:
                    analyze_benchmark.main()
            self.assertEqual(raised.exception.code, 1)
            self.assertFalse(json.loads(output.read_text(encoding="utf-8"))["data_integrity"]["valid"])


class ManifestTests(unittest.TestCase):
    def test_content_addressed_manifest_detects_changes_without_git(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "config").mkdir()
            (root / "config" / "benchmark.json").write_text(
                json.dumps({"benchmark_id": "test", "version": 1}), encoding="utf-8"
            )
            (root / "source.cpp").write_text("int value = 1;\n", encoding="utf-8")
            build = root / "build"
            build.mkdir()
            simulator = build / "sim"
            agent = build / "agent"
            supervisor = build / "supervisor"
            simulator.write_bytes(b"sim")
            agent.write_bytes(b"agent")
            supervisor.write_bytes(b"supervisor")
            output = root / "out" / "manifest.json"
            manifest = run_manifest.create_manifest(root, simulator, agent, output, supervisor=supervisor)
            run_manifest.write_manifest(output, manifest)
            self.assertEqual(run_manifest.verify_manifest(output, root, simulator, agent, supervisor=supervisor), manifest)
            self.assertEqual(set(manifest["executables"]), {"agent", "simulator", "supervisor"})
            with self.assertRaises(RuntimeError):
                run_manifest.verify_manifest(output, root, simulator, agent)
            self.assertEqual(
                manifest["run_id"],
                run_manifest.create_manifest(
                    root, simulator, agent, root / "out" / "other.json", supervisor=supervisor
                )["run_id"],
            )
            supervisor.write_bytes(b"changed")
            with self.assertRaises(RuntimeError):
                run_manifest.verify_manifest(output, root, simulator, agent, supervisor=supervisor)
            supervisor.write_bytes(b"supervisor")
            (root / "source.cpp").write_text("int value = 2;\n", encoding="utf-8")
            with self.assertRaises(RuntimeError):
                run_manifest.verify_manifest(output, root, simulator, agent, supervisor=supervisor)

    def test_manifest_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "manifest.json"
            run_manifest.write_manifest(path, {"run_id": "first"})
            with self.assertRaises(RuntimeError):
                run_manifest.write_manifest(path, {"run_id": "second"})

    def test_old_manifest_and_output_record_are_still_readable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            image = "sha256:" + "a" * 64
            payload = {
                "benchmark_id": "test",
                "benchmark_inputs": {},
                "benchmark_version": 1,
                "container_image_digest": image,
                "executables": {},
                "schema_version": 1,
                "source_files": [],
                "source_revision": "old-tree-digest",
            }
            identifier = hashlib.sha256(benchmark_io.canonical_bytes(payload)).hexdigest()
            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps({**payload, "release_id": identifier}, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            recorded_output = str((root / "results").resolve())
            record = benchmark_io.output_record_path(manifest, identifier, legacy=True)
            record.write_text(
                json.dumps(
                    {
                        "container_image_digest": image,
                        "output": recorded_output,
                        "release_id": identifier,
                        "schema_version": 1,
                    },
                    indent=2,
                    sort_keys=True,
                ) + "\n",
                encoding="utf-8",
            )
            self.assertEqual(
                analyze_benchmark.manifest_metadata(manifest, image, True),
                (identifier, benchmark_io.file_digest(record), recorded_output),
            )

    def test_container_digest_is_validated_and_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "config").mkdir()
            (root / "config" / "benchmark.json").write_text(
                json.dumps({"benchmark_id": "test", "version": 1}), encoding="utf-8"
            )
            simulator = root / "sim"
            agent = root / "agent"
            simulator.write_bytes(b"sim")
            agent.write_bytes(b"agent")
            output = root / "out" / "manifest.json"
            image = "sha256:" + "a" * 64
            manifest = run_manifest.create_manifest(root, simulator, agent, output, image)
            run_manifest.write_manifest(output, manifest)
            self.assertEqual(
                run_manifest.verify_manifest(output, root, simulator, agent, image)["container_image_digest"], image
            )
            with self.assertRaises(RuntimeError):
                run_manifest.verify_manifest(output, root, simulator, agent, "sha256:" + "b" * 64)
            record = run_manifest.record_output(output, manifest, root / "results")
            self.assertEqual(run_manifest.verify_output(output, manifest["run_id"], image)[0], record)
            identifier, record_digest, recorded_output = analyze_benchmark.manifest_metadata(output, image, True)
            self.assertEqual(identifier, manifest["run_id"])
            self.assertEqual(record_digest, benchmark_io.file_digest(record))
            self.assertEqual(recorded_output, str((root / "results").resolve()))
            analyze_benchmark.verify_result_roots([(root / "results").resolve()], recorded_output)
            with self.assertRaises(RuntimeError):
                analyze_benchmark.verify_result_roots([(root / "other").resolve()], recorded_output)
            with self.assertRaises(RuntimeError):
                run_manifest.record_output(output, manifest, root / "second")
            copied = output.with_name("copied.json")
            copied.write_bytes(output.read_bytes())
            with self.assertRaises(RuntimeError):
                run_manifest.record_output(copied, manifest, root / "second")
            self.assertEqual(
                run_manifest.output_record_path(output, manifest["run_id"]),
                run_manifest.output_record_path(copied, manifest["run_id"]),
            )
            alias = output.parent / "alias" / "manifest.json"
            alias.parent.mkdir()
            try:
                alias.symlink_to(output)
            except OSError as error:
                self.skipTest(str(error))
            with self.assertRaises(RuntimeError):
                run_manifest.record_output(alias, manifest, root / "second")
            self.assertEqual(
                run_manifest.output_record_path(output, manifest["run_id"]),
                run_manifest.output_record_path(alias, manifest["run_id"]),
            )
            with self.assertRaises(RuntimeError):
                run_manifest.create_manifest(root, simulator, agent, output, "latest")


class RunnerTests(unittest.TestCase):
    def test_persistent_frame_transport_round_trips_and_stops(self):
        from sentinel.v1 import sentinel_pb2

        first = sentinel_pb2.Envelope(recipient_id="first")
        second = sentinel_pb2.Envelope(recipient_id="second")
        reader = benchmark_runtime.FrameReader(
            io.BytesIO(benchmark_runtime.message_bytes(first) + benchmark_runtime.message_bytes(second)),
            sentinel_pb2.Envelope,
        )
        self.assertEqual(reader.read(), first)
        self.assertEqual(reader.read(), second)
        with self.assertRaises(RuntimeError):
            reader.read()
        stream = io.BytesIO()
        writer = benchmark_runtime.FrameWriter(stream)
        writer.write(first)
        self.assertTrue(writer.close())
        self.assertFalse(writer.thread.is_alive())
        self.assertEqual(stream.getvalue(), benchmark_runtime.message_bytes(first))

    def test_scratch_log_is_compressed_published_and_removed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "output"
            scratch = root / "scratch"
            output.mkdir()
            scratch.mkdir()
            path = scratch / "events.pb"
            path.write_bytes(b"events")
            published = benchmark_runtime.publish_log(path, output, scratch)
            self.assertEqual(published, output / "events.pb.gz")
            self.assertEqual(gzip.decompress(published.read_bytes()), b"events")
            self.assertFalse(scratch.exists())
            self.assertFalse((output / "events.pb.gz.tmp").exists())

    def test_missing_scratch_log_is_removed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "output"
            scratch = root / "scratch"
            output.mkdir()
            scratch.mkdir()
            path = scratch / "events.pb"
            self.assertEqual(benchmark_runtime.publish_log(path, output, scratch), path)
            self.assertFalse(scratch.exists())

    def test_failed_scratch_publication_preserves_compressed_log(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "output"
            scratch = root / "scratch"
            output.mkdir()
            scratch.mkdir()
            path = scratch / "events.pb"
            path.write_bytes(b"events")
            with mock.patch("benchmark_runtime.shutil.copyfile", side_effect=OSError("copy failed")):
                with self.assertRaises(OSError):
                    benchmark_runtime.publish_log(path, output, scratch)
            self.assertTrue((scratch / "events.pb.gz").is_file())
            self.assertFalse((output / "events.pb.gz.tmp").exists())

    def test_results_are_canonical_and_duplicates_are_rejected(self):
        values = [
            {"seed_id": "b", "allocator": "sentinel_cbba"},
            {"seed_id": "a", "allocator": "nearest_capable"},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "results.jsonl"
            run_benchmark.write_results(path, values)
            first = path.read_bytes()
            run_benchmark.write_results(path, reversed(values))
            self.assertEqual(first, path.read_bytes())
            self.assertNotIn(b"\r", first)
            path.write_bytes(first + first.splitlines(keepends=True)[0])
            with self.assertRaises(RuntimeError):
                run_benchmark.read_results(path)

    def test_result_journal_preserves_completed_rows(self):
        values = [
            {"seed_id": "b", "allocator": "sentinel_cbba"},
            {"seed_id": "a", "allocator": "nearest_capable"},
        ]
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory) / "journal"
            for value in values:
                run_benchmark.write_journal(root, value)
            (root / "partial.tmp").write_text("{", encoding="utf-8")
            self.assertEqual(set(run_benchmark.read_journal(root)), {run_benchmark.key(value) for value in values})

    def test_resume_clears_only_pending_run_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "output"
            scratch = root / "scratch"
            completed = output / "runs" / "complete" / "sentinel_cbba"
            pending = output / "runs" / "pending" / "nearest_capable"
            pending_scratch = scratch / "pending" / "nearest_capable"
            for path in (completed, pending, pending_scratch):
                path.mkdir(parents=True)
            run_benchmark.clear_pending(
                output,
                scratch,
                [({"id": "pending"}, "nearest_capable")],
            )
            self.assertTrue(completed.is_dir())
            self.assertFalse(pending.exists())
            self.assertFalse(pending_scratch.exists())

    def test_shards_are_disjoint_and_complete(self):
        values = [{"id": str(index)} for index in range(17)]
        shards = [run_benchmark.select_records(values, index, 4) for index in range(4)]
        self.assertEqual({value["id"] for shard in shards for value in shard}, {value["id"] for value in values})
        self.assertEqual(sum(len(shard) for shard in shards), len(values))

    def test_holdout_output_refuses_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "run"
            run_benchmark.prepare_output(path, False)
            with self.assertRaises(RuntimeError):
                run_benchmark.prepare_output(path, False)

    def test_holdout_manifest_and_output_share_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            run_benchmark.validate_manifest_output(root / "manifest.json", root / "holdout")
            with self.assertRaises(RuntimeError):
                run_benchmark.validate_manifest_output(root / "other" / "manifest.json", root / "holdout")

    def test_output_inside_source_tree_is_restricted_to_out(self):
        self.assertEqual(run_benchmark.validate_output(ROOT / "out" / "benchmark"), (ROOT / "out" / "benchmark").resolve())
        with self.assertRaises(RuntimeError):
            run_benchmark.validate_output(ROOT / "benchmark")

    def test_analysis_rehashes_and_cross_checks_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory).resolve()
            run = root / "runs" / "seed" / "sentinel_cbba"
            scenario = root / "scenarios" / "seed" / "sentinel_cbba.textproto"
            run.mkdir(parents=True)
            scenario.parent.mkdir(parents=True)
            paths = {
                "event_log": run / "events.pb.gz",
                "replay_summary": run / "replay.json",
                "scenario": scenario,
                "summary": run / "summary.json",
            }
            value = benchmark_config()
            result = row(seeds()[0], "sentinel_cbba", value["hard_invariants"])
            evidence = json.dumps(summary(result), sort_keys=True)
            paths["event_log"].write_bytes(b"events")
            paths["scenario"].write_bytes(b"scenario")
            paths["summary"].write_text(evidence, encoding="utf-8")
            paths["replay_summary"].write_text(evidence, encoding="utf-8")
            result["run_path"] = "runs/seed/sentinel_cbba"
            result["scenario_path"] = "scenarios/seed/sentinel_cbba.textproto"
            result["artifact_hashes"] = {name: benchmark_runtime.artifact(path) for name, path in paths.items()}
            identity = (result["seed_id"], result["allocator"])
            self.assertEqual(analyze_benchmark.audit_artifacts([result], [root], value["hard_invariants"]), set())
            record = seeds()[0]
            generated = generate_scenarios.generate_scenario(record, result["allocator"], False)
            paths["scenario"].write_text(text_format.MessageToString(generated, as_utf8=True), encoding="utf-8")
            result["paired_input_sha256"] = generate_scenarios.paired_digest(generated)
            result["artifact_hashes"]["scenario"] = benchmark_runtime.artifact(paths["scenario"])
            self.assertEqual(
                analyze_benchmark.audit_artifacts([result], [root], value["hard_invariants"], [record]), set()
            )
            generated.seed += 1
            paths["scenario"].write_text(text_format.MessageToString(generated, as_utf8=True), encoding="utf-8")
            result["artifact_hashes"]["scenario"] = benchmark_runtime.artifact(paths["scenario"])
            self.assertEqual(
                analyze_benchmark.audit_artifacts([result], [root], value["hard_invariants"], [record]), {identity}
            )
            generated.seed -= 1
            paths["scenario"].write_text(text_format.MessageToString(generated, as_utf8=True), encoding="utf-8")
            result["artifact_hashes"]["scenario"] = benchmark_runtime.artifact(paths["scenario"])
            original = paths["event_log"].read_bytes()
            paths["event_log"].write_bytes(bytes([original[0] ^ 1]) + original[1:])
            self.assertEqual(analyze_benchmark.audit_artifacts([result], [root], value["hard_invariants"]), {identity})
            paths["event_log"].write_bytes(original)
            replay = summary(result)
            replay["ticks"] += 1
            paths["replay_summary"].write_text(json.dumps(replay, sort_keys=True), encoding="utf-8")
            result["artifact_hashes"]["replay_summary"] = benchmark_runtime.artifact(paths["replay_summary"])
            self.assertEqual(analyze_benchmark.audit_artifacts([result], [root], value["hard_invariants"]), {identity})
            paths["replay_summary"].write_text(evidence, encoding="utf-8")
            result["artifact_hashes"]["replay_summary"] = benchmark_runtime.artifact(paths["replay_summary"])
            result["travel_distance_mm"] += 1
            self.assertEqual(analyze_benchmark.audit_artifacts([result], [root], value["hard_invariants"]), {identity})
            result["travel_distance_mm"] -= 1
            result["hard_invariants"]["terminal_replay_hash_matches"] = False
            result["hard_invariant_violations"] = 1
            result["mission_success"] = False
            self.assertEqual(analyze_benchmark.audit_artifacts([result], [root], value["hard_invariants"]), {identity})

    def test_artifact_audit_propagates_unexpected_worker_failure(self):
        result = {"seed_id": "seed", "allocator": "sentinel_cbba"}
        with mock.patch.object(analyze_benchmark, "artifact_paths", side_effect=RuntimeError):
            with self.assertRaises(RuntimeError):
                analyze_benchmark.audit_artifacts([result], [pathlib.Path.cwd()])

    def test_event_log_compression_is_lossless_and_deterministic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            first = root / "first" / "events.pb"
            second = root / "second" / "events.pb"
            first.parent.mkdir()
            second.parent.mkdir()
            payload = b"event-log" * 1000
            first.write_bytes(payload)
            second.write_bytes(payload)
            first_compressed = benchmark_runtime.compress_log(first)
            second_compressed = benchmark_runtime.compress_log(second)
            self.assertEqual(first_compressed.read_bytes(), second_compressed.read_bytes())
            with gzip.open(first_compressed, "rb") as stream:
                self.assertEqual(stream.read(), payload)




if __name__ == "__main__":
    unittest.main()
