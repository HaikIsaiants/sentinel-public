import gzip
import json
import os
import pathlib
import queue
import signal
import shutil
import struct
import subprocess
import threading

from benchmark_io import file_digest


MAX_FRAME_BYTES = 16 * 1024 * 1024
RESPONSE_TIMEOUT_SECONDS = 10
REPLAY_TIMEOUT_SECONDS = 900
MISSION_TIMEOUT_SECONDS = 7200


def artifact(path):
    if not path.is_file():
        return None
    return {"name": path.name, "sha256": file_digest(path), "size": path.stat().st_size}


def compress_log(path):
    compressed = path.with_suffix(path.suffix + ".gz")
    tmp = compressed.with_suffix(compressed.suffix + ".tmp")
    try:
        with path.open("rb") as source, tmp.open("wb") as target:
            with gzip.GzipFile(filename="", mode="wb", fileobj=target, compresslevel=6, mtime=0) as archive:
                shutil.copyfileobj(source, archive, 1024 * 1024)
    except Exception:
        tmp.unlink(missing_ok=True)
        raise
    tmp.replace(compressed)
    path.unlink()
    return compressed


def message_bytes(message):
    payload = message.SerializeToString(deterministic=True)
    if len(payload) > MAX_FRAME_BYTES:
        raise RuntimeError("protocol frame is too large")
    return struct.pack(">I", len(payload)) + payload


class FrameWriter:
    def __init__(self, stream):
        self.stream = stream
        self.requests = queue.Queue(maxsize=1)
        self.thread = threading.Thread(target=self.transmit, daemon=True)
        self.thread.start()

    def transmit(self):
        while True:
            request = self.requests.get()
            if request is None:
                return
            payload, completion = request
            try:
                self.stream.write(payload)
                self.stream.flush()
                completion.put(None)
            except Exception as error:
                completion.put(error)
                return

    def enqueue(self, message):
        completion = queue.Queue(maxsize=1)
        try:
            self.requests.put((message_bytes(message), completion), timeout=RESPONSE_TIMEOUT_SECONDS)
        except queue.Full:
            raise RuntimeError("process request timed out")
        return completion

    def wait(self, completion):
        try:
            error = completion.get(timeout=RESPONSE_TIMEOUT_SECONDS)
        except queue.Empty:
            raise RuntimeError("process request timed out")
        if error:
            raise error

    def write(self, message):
        completion = self.enqueue(message)
        self.wait(completion)

    def close(self):
        try:
            self.requests.put(None, timeout=RESPONSE_TIMEOUT_SECONDS)
        except queue.Full:
            return False
        self.thread.join(timeout=RESPONSE_TIMEOUT_SECONDS)
        return not self.thread.is_alive()


class FrameReader:
    def __init__(self, stream, message_type):
        self.stream = stream
        self.message_type = message_type
        self.messages = queue.Queue(maxsize=1)
        threading.Thread(target=self.receive, daemon=True).start()

    def receive(self):
        try:
            while True:
                header = self.stream.read(4)
                if len(header) != 4:
                    raise RuntimeError("process closed its protocol stream")
                size = struct.unpack(">I", header)[0]
                if size > MAX_FRAME_BYTES:
                    raise RuntimeError("process returned an oversized protocol frame")
                payload = self.stream.read(size)
                if len(payload) != size:
                    raise RuntimeError("process returned a truncated protocol frame")
                message = self.message_type()
                message.ParseFromString(payload)
                self.messages.put(message)
        except Exception as error:
            self.messages.put(error)

    def read(self):
        try:
            msg = self.messages.get(timeout=RESPONSE_TIMEOUT_SECONDS)
        except queue.Empty:
            raise RuntimeError("process response timed out")
        if isinstance(msg, Exception):
            raise msg
        return msg


def stop_process(process, force=False):
    if force:
        if process.poll() is None:
            process.terminate()
    elif process.stdin and not process.stdin.closed:
        process.stdin.close()
    try:
        return_code = process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return_code = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            return_code = process.wait(timeout=5)
    error = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
    if return_code != 0:
        return error.strip() or f"process exited with {return_code}"
    return ""


def convergence(sample):
    return {
        "complete": sample.complete,
        "end_tick": sample.end_tick,
        "epoch": sample.epoch,
        "start_tick": sample.start_tick,
    }


def detection(sample):
    return {
        "detection_tick": sample.detection_tick,
        "detector_agent_id": sample.detector_agent_id,
        "failed_agent_id": sample.failed_agent_id,
        "failure_tick": sample.failure_tick,
    }


def reassignment(sample):
    return {
        "commit_tick": sample.commit_tick,
        "complete": sample.complete,
        "detection_tick": sample.detection_tick,
        "detector_agent_id": sample.detector_agent_id,
        "failed_agent_id": sample.failed_agent_id,
        "failure_tick": sample.failure_tick,
        "new_agent_id": sample.new_agent_id,
        "new_epoch": sample.new_epoch,
        "new_version": sample.new_version,
        "previous_epoch": sample.previous_epoch,
        "previous_version": sample.previous_version,
        "task_id": sample.task_id,
    }


def replanning(sample):
    return {
        "agent_id": sample.agent_id,
        "complete": sample.complete,
        "end_tick": sample.end_tick,
        "reason": sample.reason,
        "start_tick": sample.start_tick,
        "wait_plan": sample.wait_plan,
    }


def summary_values(summary):
    return {
        "active_agents": summary.active_agents,
        "agent_energy_below_zero_violations": summary.agent_energy_below_zero_violations,
        "agent_energy_never_drops_below_zero": summary.agent_energy_never_drops_below_zero,
        "allocation_convergence": [convergence(value) for value in summary.allocation_convergence],
        "communication_bytes": summary.communication_bytes,
        "communication_messages": summary.communication_messages,
        "completed_tasks": summary.completed_tasks,
        "completed_task_reassignment_violations": summary.completed_task_reassignment_violations,
        "completed_tasks_are_never_reassigned": summary.completed_tasks_are_never_reassigned,
        "committed_reservation_overlap_violations": summary.committed_reservation_overlap_violations,
        "committed_reservations_never_overlap": summary.committed_reservations_never_overlap,
        "delivered_messages": summary.delivered_messages,
        "dropped_messages": summary.dropped_messages,
        "energy_consumed_mj": summary.energy_consumed_mj,
        "failure_detections": [detection(value) for value in summary.failure_detections],
        "maximum_reassignment_delay_ms": summary.maximum_reassignment_delay_ms,
        "missing_reassignments": summary.missing_reassignments,
        "incapable_agent_commit_violations": summary.incapable_agent_commit_violations,
        "incapable_agents_never_commit_tasks": summary.incapable_agents_never_commit_tasks,
        "recharge_ticks": summary.recharge_ticks,
        "rejected_commits": summary.rejected_commits,
        "reordered_messages": summary.reordered_messages,
        "replan_count": summary.replan_count,
        "replanning_samples": [replanning(value) for value in summary.replanning_samples],
        "return_ticks": summary.return_ticks,
        "route_conflicts": summary.route_conflicts,
        "summary_success": summary.success,
        "task_reassignments": [reassignment(value) for value in summary.task_reassignments],
        "terminal_hash": summary.terminal_hash,
        "ticks": summary.ticks,
        "total_tasks": summary.total_tasks,
        "travel_distance_mm": summary.travel_distance_mm,
        "wait_ticks": summary.wait_ticks,
    }


def publish_log(path, output, scratch):
    if not path.is_file():
        if scratch:
            shutil.rmtree(scratch)
        return path
    compressed = compress_log(path)
    if not scratch:
        return compressed
    destination = output / compressed.name
    tmp = destination.with_suffix(destination.suffix + ".tmp")
    try:
        shutil.copyfile(compressed, tmp)
        tmp.replace(destination)
    except Exception:
        tmp.unlink(missing_ok=True)
        raise
    shutil.rmtree(scratch)
    return destination


def stop_group(process):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def run_supervisor(supervisor, simulator, agent, scenario, log_path, summary_path, message_type):
    process = subprocess.Popen(
        [
            str(supervisor),
            "--sim",
            str(simulator),
            "--agent",
            str(agent),
            "--scenario",
            str(scenario),
            "--log",
            str(log_path),
            "--summary",
            str(summary_path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    complete = False
    try:
        try:
            stdout, stderr = process.communicate(timeout=MISSION_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("supervisor timed out") from error
        if process.returncode:
            raise RuntimeError(
                stderr.decode("utf-8", errors="replace").strip() or f"supervisor exited with {process.returncode}"
            )
        if len(stdout) < 4:
            raise RuntimeError("supervisor returned no terminal frame")
        size = struct.unpack(">I", stdout[:4])[0]
        if size > MAX_FRAME_BYTES or size != len(stdout) - 4:
            raise RuntimeError("supervisor returned an invalid terminal frame")
        frame = message_type()
        frame.ParseFromString(stdout[4:])
        if not frame.HasField("observations") or not frame.observations.finished:
            raise RuntimeError("supervisor returned a nonterminal frame")
        summary = frame.observations.summary.__class__()
        summary.CopyFrom(frame.observations.summary)
        complete = True
        return summary
    finally:
        if not complete:
            stop_group(process)


def run_mission(simulator, agent, scenario, output, horizon_ticks, scratch=None, supervisor=None, agent_count=0):
    """Run one mission and collect replay-checked result artifacts."""

    from sentinel.v1 import sentinel_pb2

    output.mkdir(parents=True, exist_ok=False)
    if scratch:
        scratch.mkdir(parents=True, exist_ok=False)
    log_path = (scratch or output) / "events.pb"
    summary_path = output / "summary.json"
    replay_path = output / "replay.json"
    simulation = None
    simulation_reader = None
    simulation_writer = None
    agents = {}
    agent_readers = {}
    agent_writers = {}
    summary = None
    terminal = False
    errors = []
    try:
        if supervisor:
            summary = run_supervisor(
                supervisor, simulator, agent, scenario, log_path, summary_path, sentinel_pb2.SimulationFrame
            )
            terminal = True
        else:
            simulation = subprocess.Popen(
                [str(simulator), "run", "--scenario", str(scenario), "--log", str(log_path), "--summary", str(summary_path)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            simulation_reader = FrameReader(simulation.stdout, sentinel_pb2.SimulationFrame)
            simulation_writer = FrameWriter(simulation.stdin)
            while True:
                frame = simulation_reader.read()
                if not frame.HasField("observations"):
                    raise RuntimeError("simulator returned an invalid frame")
                observations = frame.observations
                if observations.finished:
                    summary = sentinel_pb2.SimulationSummary()
                    summary.CopyFrom(observations.summary)
                    terminal = True
                    break
                if not 3 <= len(observations.observations) <= 5:
                    raise RuntimeError("mission does not contain three to five agents")
                batch = sentinel_pb2.ActionBatch(tick=observations.tick)
                envelopes = sorted(observations.observations, key=lambda value: value.recipient_id)
                completions = {}
                for envelope in envelopes:
                    agent_id = envelope.recipient_id
                    if envelope.observation.self.id != agent_id:
                        raise RuntimeError("observation crossed an agent boundary")
                    if any(task.assigned_agent_id != agent_id for task in envelope.observation.assigned_tasks):
                        raise RuntimeError("agent received another agent's task")
                    if agent_id not in agents:
                        agents[agent_id] = subprocess.Popen(
                            [str(agent), "--id", agent_id],
                            stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                        )
                        agent_readers[agent_id] = FrameReader(agents[agent_id].stdout, sentinel_pb2.Envelope)
                        agent_writers[agent_id] = FrameWriter(agents[agent_id].stdin)
                    completions[agent_id] = agent_writers[agent_id].enqueue(envelope)
                for envelope in envelopes:
                    agent_writers[envelope.recipient_id].wait(completions[envelope.recipient_id])
                    batch.actions.add().CopyFrom(agent_readers[envelope.recipient_id].read())
                request = sentinel_pb2.SimulationFrame()
                request.actions.CopyFrom(batch)
                simulation_writer.write(request)
    except Exception as error:
        errors.append(str(error))
    finally:
        force = bool(errors)
        for agent_id, process in agents.items():
            if force:
                error = stop_process(process, True)
                closed = agent_writers[agent_id].close()
            else:
                closed = agent_writers[agent_id].close()
                error = stop_process(process)
            if not closed:
                errors.append("agent request writer did not stop")
            if error:
                errors.append(error)
        if simulation:
            if force:
                error = stop_process(simulation, True)
                closed = simulation_writer.close()
            else:
                closed = simulation_writer.close()
                error = stop_process(simulation)
            if not closed:
                errors.append("simulator request writer did not stop")
            if error:
                errors.append(error)
    execution_valid = terminal and not errors
    replay_attempted = terminal
    replay_verified = False
    if terminal:
        try:
            replay = subprocess.run(
                [str(simulator), "replay", "--scenario", str(scenario), "--log", str(log_path), "--summary", str(replay_path)],
                capture_output=True,
                text=True,
                timeout=REPLAY_TIMEOUT_SECONDS,
            )
            if replay.returncode == 0 and replay_path.is_file():
                replay_summary = json.loads(replay_path.read_text(encoding="utf-8"))
                replay_verified = replay.stdout.strip() == summary.terminal_hash == replay_summary.get("terminal_hash")
            if not replay_verified:
                errors.append(replay.stderr.strip() or "event log replay failed")
        except (OSError, subprocess.TimeoutExpired, json.JSONDecodeError) as error:
            errors.append(f"event log replay failed: {error}")
    log_path = publish_log(log_path, output, scratch)
    invariants = {
        "agent_energy_never_drops_below_zero": bool(summary and summary.agent_energy_never_drops_below_zero),
        "committed_reservations_never_overlap": bool(summary and summary.committed_reservations_never_overlap),
        "completed_tasks_are_never_reassigned": bool(summary and summary.completed_tasks_are_never_reassigned),
        "incapable_agents_never_commit_tasks": bool(summary and summary.incapable_agents_never_commit_tasks),
        "terminal_replay_hash_matches": replay_verified,
    }
    values = summary_values(summary) if summary else {}
    mission_success = bool(summary and summary.success and execution_valid and all(invariants.values()))
    paths = {
        "event_log": log_path,
        "replay_summary": replay_path,
        "scenario": scenario,
        "summary": summary_path,
    }
    artifacts = {name: artifact(path) for name, path in paths.items()}
    artifacts_valid = all(value is not None for value in artifacts.values())
    return {
        **values,
        "agent_count": agent_count if supervisor else len(agents),
        "artifact_hashes": artifacts,
        "artifacts_valid": artifacts_valid,
        "error": "; ".join(errors)[:2000],
        "hard_invariant_violations": sum(not value for value in invariants.values()),
        "hard_invariants": invariants,
        "makespan_ticks": values.get("ticks", horizon_ticks) if mission_success else horizon_ticks,
        "mission_success": mission_success,
        "replay_attempted": replay_attempted,
        "replay_verified": replay_verified,
        "status": "terminated" if terminal else "crashed",
        "terminal_event_present": execution_valid and artifacts["event_log"] is not None,
    }
