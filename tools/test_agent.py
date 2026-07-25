import argparse
import subprocess

from run_scenario import read_message, stop_process, write_message
from sentinel.v1 import sentinel_pb2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--agent", required=True)
    cli = parser.parse_args()
    process = subprocess.Popen(
        [cli.agent, "--id", "uav-alpha"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        envelope = sentinel_pb2.Envelope(
            schema_version=1,
            sequence=1,
            sender_id="sim",
            recipient_id="uav-alpha",
        )
        observation = envelope.observation
        observation.step_ms = 100
        observation.world.width_mm = 100
        observation.world.height_mm = 100
        observation.world.grid_cell_mm = 10
        observation.world.map_version = 1
        observation.peer_ids.append("uav-alpha")
        observation.self.id = "uav-alpha"
        observation.self.active = True
        observation.self.energy_mj = 100000
        observation.self.initial_energy_mj = 100000
        observation.self.energy_cost_mj_per_meter = 1000
        observation.self.max_speed_mm_s = 1000
        observation.self.capabilities.append(sentinel_pb2.CAPABILITY_SEARCH)
        task = observation.assigned_tasks.add(
            id="radius-edge",
            required_capability=sentinel_pb2.CAPABILITY_SEARCH,
            assigned_agent_id="uav-alpha",
            completion_radius_mm=50,
            deadline_tick=100,
            status=sentinel_pb2.TASK_STATUS_PENDING,
        )
        task.target.x_mm = 40
        task.target.y_mm = 40
        write_message(process.stdin, envelope)
        action = read_message(process.stdout, sentinel_pb2.Envelope)
        if not action.HasField("action") or not (action.action.velocity_x_mm_s or action.action.velocity_y_mm_s):
            raise RuntimeError("agent stalled outside the task completion radius")
    finally:
        error = stop_process(process)
    if error:
        raise RuntimeError(error)
    print("agent smoke test passed")


if __name__ == "__main__":
    main()
