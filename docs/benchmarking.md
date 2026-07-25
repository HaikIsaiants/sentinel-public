# Benchmarking

I use the benchmark to compare Sentinel's CBBA allocator with a nearest-capable policy on exactly the same generated missions. Allocation is the variable; routing, failures, task execution, reservations, and network simulation are shared.

## Corpus

There are 2,000 development seeds and 10,000 holdout seeds checked into the repository. Each set is balanced across these ten scenario groups:

- nominal operation
- latency and jitter
- packet loss and reordering
- bandwidth limits
- temporary partitions
- blocked routes
- dynamic task arrival
- agent loss or degradation
- combined communication faults
- compound communication, map, and agent disruption

Fleets start with three to five mixed UAVs and UGVs. Missions combine search, inspection, relay, and delivery tasks, with different capability, payload, service-time, deadline, terrain, and energy requirements.

The nearest-capable policy first removes vehicles that can't satisfy those requirements. It ranks the remaining choices by canonical A* distance and stable vehicle ID.

## What gets measured

A completed mission has all active tasks finished before their deadlines and the 600-second horizon. It also needs a terminal event, a matching replay hash, and clean safety invariants. A missing, crashed, corrupt, or incomplete run gets a 600-second makespan.

The main numbers I look at are completion rate, the one-sided 95% Wilson lower bound across the six degraded-communication groups, mean makespan, and the time from local failure detection to a new commitment for an orphaned task. Missing reassignments are counted separately.

The JSON report has more detail: allocation convergence, travel distance, energy use, route conflicts, replanning latency, message count, and serialized bytes. Durations come from simulated time.

Every result row names its seed and allocator. During analysis, the tool checks that all expected pairs are present, rehashes the saved scenario and run files, compares the simulator and replay summaries, and then calculates the aggregate values.

## Running a small comparison

Build the C++ targets first. This example generates and runs ten development missions per allocator:

```bash
PYTHONPATH=build/python python3 tools/generate_scenarios.py \
  --output out/generated \
  --count 10

PYTHONPATH=build/python python3 tools/run_benchmark.py \
  --sim build/bin/sentinel_sim \
  --agent build/bin/sentinel_agent \
  --supervisor build/bin/sentinel_supervisor \
  --corpus development \
  --output out/development \
  --scratch /tmp/sentinel-development \
  --jobs 8 \
  --limit 10

PYTHONPATH=build/python python3 tools/analyze_benchmark.py \
  --results out/development/results.jsonl \
  --corpus development \
  --output out/development-report.json \
  --markdown out/development-report.md
```

Random draws are derived from the mission seed and configuration path. For a full run I save a `tools/run_manifest.py` manifest beside the results so I can trace it back to the inputs and binaries.

## Recorded run: 2026-07-19

The holdout comparison covered 10,000 unique seeds and ran both allocators, giving 20,000 result rows.

| Measurement | Sentinel CBBA | Nearest capable | Difference |
| --- | ---: | ---: | ---: |
| Completed missions | 9,998 / 10,000 (99.980%) | 9,893 / 10,000 (98.930%) | +105 |
| Degraded-communication completion | 5,999 / 6,000 (99.983%) | 5,988 / 6,000 (99.800%) | Sentinel lower bound: 99.925% |
| Mean makespan | 158.173 s | 229.161 s | 30.978% lower |
| Orphan-task reassignment after local detection | mean 0.197 s; p95 0.5 s; max 0.7 s | n/a | 1,500 / 1,500 reassigned |

Two Sentinel rows hit the supervisor's original 10-second response timeout. Those rows remain in the results as unsuccessful missions with 600-second makespans. I later ran both cases with the same simulator and agent binaries and a 60-second watchdog; both completed. The table above still uses the original rows.

The aggregate values and result hash are in [benchmark-result.json](benchmark-result.json).

## Scope and limitations

This is a synthetic grid workload with simplified vehicle motion and network behavior. Failure-detection timers pause during link partitions. Reservations apply to named chokepoints, while normal grid movement relies on the simulator's motion checks. Gazebo displays recorded traces after a mission; it isn't part of the timed run.
