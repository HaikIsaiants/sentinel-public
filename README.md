# Sentinel

Sentinel is a deterministic mission simulator for small teams of UAVs and UGVs. Each vehicle runs in its own process and coordinates search, inspection, relay, and delivery tasks while links fail, routes close, and vehicles drop out.

A fixed-step simulator maintains authoritative mission state, while each agent receives only local observations and delivered peer messages. Agents use decentralized CBBA allocation, BehaviorTree.CPP for execution, capability-aware A* routing, and time-window reservations at shared chokepoints. Runs produce event logs that can be replayed byte for byte. See [Architecture](docs/architecture.md) for a detailed description.

## Build

You'll need CMake, Ninja, a C++20 compiler, Protobuf (including its Python module), Eigen, GoogleTest, and Python 3. The first CMake configure also downloads the pinned BehaviorTree.CPP revision.

Windows with vcpkg:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Linux with system packages:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Or build and test the Docker image:

```bash
docker build -t sentinel .
docker run --rm sentinel
```

## Try a mission

This launches the simulator plus one process per vehicle, runs the scenario three times, compares the tick hashes and event logs, and replays each log:

```bash
PYTHONPATH=build/python python3 tools/run_scenario.py \
  --sim build/bin/sentinel_sim \
  --agent build/bin/sentinel_agent \
  --scenario scenarios/compound_failure_cbba.textproto \
  --output out/demo \
  --repeat 3 \
  --require-complete-reassignments
```

There are smaller scenarios for charging, rerouting, reservations, allocation under a clean network, and individual communication faults. The paired fixtures run CBBA and nearest-capable allocation on the same mission.

## Repository map

| Path | What's there |
| --- | --- |
| `sim/`, `core/` | World state, virtual time, failures, network emulation, reservations, logs, and replay |
| `agent/` | Behavior-tree execution and decentralized allocation |
| `planning/` | A*, energy checks, bundle scoring, and chokepoint scheduling |
| `proto/` | Scenarios, process messages, and trace records |
| `runner/` | Native process supervisor used by benchmark workers |
| `tools/` | Scenario generation, orchestration, benchmark runs, and analysis |
| `ros2/sentinel_gazebo/` | Offline playback in Gazebo Harmonic |

## Benchmark result

The recorded comparison used 10,000 paired seeds, so 20,000 allocator runs in total. Fleets started with three to five mixed UAVs and UGVs. Sentinel completed 9,998 of 10,000 missions (99.980%); across the six degraded-communication groups, 5,999 of 6,000 completed and the one-sided Wilson lower bound was 99.925%.

Mean makespan was 158.173 seconds for Sentinel and 229.161 seconds for nearest-capable allocation, a 30.978% reduction. All 1,500 orphaned tasks were reassigned after local failure detection, with a 0.197-second mean, 0.5-second p95, and 0.7-second maximum.

The full setup, timeout notes, and limitations are in [Benchmarking](docs/benchmarking.md). The saved summary is [benchmark-result.json](docs/benchmark-result.json).

## Gazebo playback

Gazebo is a viewer for an exported run. First make a presentation trace from a replayed event log:

```bash
build/bin/sentinel_sim export \
  --scenario scenarios/compound_failure_cbba.textproto \
  --log out/demo/run-0.events.pb \
  --trace out/demo/demo.trace.pb
```

Then, on a ROS 2 Jazzy system with Gazebo Harmonic and `ros_gz` installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select sentinel_gazebo
source install/setup.bash
ros2 launch sentinel_gazebo demo.launch.py trace:=$(pwd)/out/demo/demo.trace.pb
```

The adapter reads the recorded observations and moves simple Gazebo models to match them.
