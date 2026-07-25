# Architecture

The important boundary in Sentinel is between the simulator and the vehicles. The simulator owns the map, clock, task state, failures, and final decision on whether an action is valid. An agent owns its local plan and only sees serialized observations addressed to that vehicle.

That split is why the agents run as separate processes. A convenient in-process pointer to world state would make allocation much easier, but it would also hide the communication and stale-state problems I wanted to exercise.

## Layout

| Module | Main job | Dependencies |
| --- | --- | --- |
| `proto/` | Scenarios, observations, actions, peer messages, events, and traces | Protobuf |
| `planning/` | Integer geometry, A*, energy checks, task scoring, and reservation windows | Protobuf, Eigen |
| `core/` | World transitions, network and failure models, event logs, hashes, and replay | `proto/`, `planning/` |
| `sim/` | Scenario execution and the framed process protocol | `core/` |
| `agent/` | Behavior trees, CBBA, routing, recovery, and peer coordination | `proto/`, `planning/`, BehaviorTree.CPP |
| `runner/` | Supervisor used by parallel benchmark workers | `proto/` |
| `tools/` | Python orchestration, corpus generation, and result analysis | generated Protobuf bindings |
| `ros2/sentinel_gazebo/` | Recorded-trace playback | ROS 2 Jazzy, Gazebo Harmonic, `ros_gz` |

## One tick

A mission begins with three to five UAV and UGV processes plus the simulator. Every 100 ms of virtual time:

1. The simulator sends each active vehicle its own observation envelope.
2. Agents return commands and any peer messages they want to send.
3. The simulator validates and applies commands, scheduled events, and network events.
4. It advances the world, records the state hash, and queues peer traffic for later delivery.

Each agent keeps its own view of peers, bids, reservations, route, and task progress. Messages can arrive late, out of order, or never, depending on the link profile. The simulator still checks every action against the current authoritative state.

## Repeatable runs

The simulator uses integer ticks and integer units for distance, energy, rates, and probabilities. Stable IDs settle ties. Random values are keyed by the root seed, configuration path, draw index, and retry index, which keeps an extra draw in one subsystem from shifting the streams used elsewhere.

At each tick, commands are put in canonical order before mutation. The simulator then applies failures and map changes, handles reservation requests, advances vehicles and tasks, resolves network deliveries, and hashes the resulting state. The same binaries and scenario should produce the same tick hashes and event-log bytes.

The rejection sampler deserves a mention because small probability biases add up over a large corpus. It draws across the source integer range and discards the short tail before reducing into the requested range.

## Allocation

CBBA is decentralized here. An agent builds an ordered bundle from the tasks it currently knows about, exchanges versioned bids, and resolves competing claims with deterministic ranks. Feasibility checks include vehicle capability, payload, path, deadline, service cost, and enough energy to finish with a reserve.

The comparison policy is deliberately plain: filter to feasible vehicles, then choose by canonical A* distance and stable vehicle ID. After assignment, both policies share the same routing, reservations, network, failure recovery, and task execution.

BehaviorTree.CPP handles the local loop. It reconciles allocation state, recovers interrupted work, gets a reservation when a route needs one, follows the route, performs service, and returns to charge when required.

One detail that caused enough edge cases to be worth calling out: losing a CBBA bundle item also releases the dependent suffix. Keeping later bids after an earlier route assumption changes can leave an agent claiming work it can no longer perform in that order.

## Movement and chokepoints

Vehicles move on a four-connected integer grid. A* uses the full integer score and coordinate tuple to order equal candidates. Terrain changes speed and energy use; a region closure increments the map version and invalidates routes planned against the older map.

Shared chokepoints use inclusive time-window reservations. A proposal includes its route, map version, allocation epoch, and reservation version. The simulator rejects stale or overlapping commitments and checks the grant again when the vehicle tries to enter. A vehicle that loses contention waits at a deterministic grid location and replans on the next observation.

## Failures and links

There is one queue and one byte budget per directed link. A packet captures the active link profile when it enters the queue; keyed draws decide jitter, loss, and reordering. Partitions change both directions of a link. Work that was already committed stays local until another agent has enough information to supersede it.

Heartbeat timers advance only while the peer link is available. Once a surviving agent detects that an owner has failed, it releases the orphaned task under a newer allocation epoch. A capable survivor can then bid and commit it. Interrupted task service starts over from zero progress.

## Logs and the Gazebo adapter

The event log contains accepted actions, resolved events, network outcomes, the state hash for each tick, and the terminal summary. Replay applies those records directly and compares the stored hashes. Truncated logs and hash mismatches make the run unsuccessful.

For visualization, the simulator exports recipient-scoped observations from a replay into a smaller presentation trace. The ROS 2 adapter reads that trace and updates simple models in Gazebo. It stays outside the timed mission path.
