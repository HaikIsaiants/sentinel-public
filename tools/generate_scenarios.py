import argparse
import hashlib
import json
import math
from collections import Counter, deque
from decimal import Decimal, ROUND_CEILING
from pathlib import Path

from google.protobuf import text_format
from sentinel.v1 import sentinel_pb2


ROOT = Path(__file__).resolve().parents[1]
STRATA_CONFIG = json.loads((ROOT / "config" / "scenario_strata.json").read_text(encoding="utf-8"))
GENERATION_CONFIG = json.loads((ROOT / "config" / "scenario_generation.json").read_text(encoding="utf-8"))
SEED_ROOT = ROOT / "benchmarks" / "seeds"
POLICIES = {
    "sentinel_cbba": sentinel_pb2.ALLOCATION_POLICY_SENTINEL_CBBA,
    "nearest_capable": sentinel_pb2.ALLOCATION_POLICY_NEAREST_CAPABLE,
}
CAPABILITIES = {
    "search": sentinel_pb2.CAPABILITY_SEARCH,
    "inspection": sentinel_pb2.CAPABILITY_INSPECTION,
    "relay": sentinel_pb2.CAPABILITY_RELAY,
    "delivery": sentinel_pb2.CAPABILITY_DELIVERY,
}
REGION_KINDS = {
    "obstacle": sentinel_pb2.REGION_KIND_OBSTACLE,
    "restricted": sentinel_pb2.REGION_KIND_RESTRICTED,
    "terrain": sentinel_pb2.REGION_KIND_TERRAIN,
    "chokepoint": sentinel_pb2.REGION_KIND_CHOKEPOINT,
}


class Draws:
    def __init__(self, seed):
        self.seed = seed
        self.namespace = GENERATION_CONFIG["namespace"]

    def value(self, path, draw=0, attempt=0):
        key = f"{self.namespace}|{self.seed}|{path}|{draw}|{attempt}".encode()
        return int.from_bytes(hashlib.sha256(key).digest(), "big")

    def integer(self, path, minimum, maximum, draw=0):
        if minimum > maximum:
            raise ValueError("invalid integer range")
        span = maximum - minimum + 1
        limit = (1 << 256) // span * span
        attempt = 0
        while True:
            value = self.value(path, draw, attempt)
            if value < limit:
                return minimum + value % span
            attempt += 1

    def rank(self, path, value):
        return self.value(f"{path}/{value}")


def load_seed_records(corpus):
    if corpus not in {"development", "holdout"}:
        raise ValueError("unknown seed corpus")
    records = [json.loads(line) for line in (SEED_ROOT / f"{corpus}.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(records) != len({row["id"] for row in records}):
        raise ValueError("duplicate seed record")
    return records


def _stratum(record):
    matches = [item for item in STRATA_CONFIG["strata"] if item["id"] == record["stratum"]]
    if len(matches) != 1:
        raise ValueError("unknown scenario stratum")
    return matches[0]


def _integer_range(draws, path, values, draw=0):
    return draws.integer(path, int(values[0]), int(values[1]), draw)


def _fixed_range(draws, path, values, scale, draw=0):
    minimum = Decimal(str(values[0])) * scale
    maximum = Decimal(str(values[1])) * scale
    if minimum == maximum:
        return int(minimum)
    low = int(minimum.to_integral_value(rounding=ROUND_CEILING))
    high = int(maximum.to_integral_value(rounding=ROUND_CEILING)) - 1
    return draws.integer(path, low, high, draw)


def _point(message, cells):
    message.x_mm = int(cells[0]) * 1000
    message.y_mm = int(cells[1]) * 1000


def _region(identifier, kind, minimum, maximum, multiplier=1000, terrain=""):
    value = sentinel_pb2.Region(id=identifier, kind=REGION_KINDS[kind], energy_multiplier_permille=multiplier)
    _point(value.minimum, minimum)
    _point(value.maximum, maximum)
    value.terrain = terrain
    return value


def _world(draws):
    source = STRATA_CONFIG["mission"]["world"]
    config = GENERATION_CONFIG["world"]
    width = source["width_meters"]
    height = source["height_meters"]
    world = sentinel_pb2.World(
        width_mm=width * 1000,
        height_mm=height * 1000,
        grid_cell_mm=source["grid_cell_meters"] * 1000,
        map_version=config["map_version"],
    )
    obstacle_low = math.ceil(Decimal(str(source["obstacle_fraction"][0])) * height)
    obstacle_high = math.ceil(Decimal(str(source["obstacle_fraction"][1])) * height) - 1
    restricted_low = math.ceil(Decimal(str(source["restricted_fraction"][0])) * height)
    restricted_high = math.ceil(Decimal(str(source["restricted_fraction"][1])) * height) - 1
    obstacle_cells = draws.integer("world/obstacle_band_cells", obstacle_low, obstacle_high)
    restricted_cells = draws.integer("world/restricted_band_cells", restricted_low, restricted_high)
    regions = [
        _region("obstacle-band", "obstacle", [0, height - obstacle_cells], [width, height]),
        _region("restricted-band", "restricted", [0, 0], [width, restricted_cells]),
    ]
    terrain_count = _integer_range(draws, "world/terrain_count", source["terrain_region_count"])
    terrain_candidates = config["terrain_candidates_cells"]
    terrain_order = sorted(range(len(terrain_candidates)), key=lambda index: draws.rank("world/terrain_order", index))
    for ordinal, candidate_index in enumerate(terrain_order[:terrain_count]):
        size = _integer_range(draws, f"world/terrain/{ordinal}/size", config["terrain_size_cells"])
        x, y = terrain_candidates[candidate_index]
        terrain = config["terrain_labels"][draws.integer(f"world/terrain/{ordinal}/label", 0, 2)]
        multiplier = _integer_range(
            draws, f"world/terrain/{ordinal}/energy", config["terrain_energy_multiplier_permille"]
        )
        regions.append(_region(f"terrain-{ordinal:02d}", "terrain", [x, y], [x + size, y + size], multiplier, terrain))
    chokepoint_count = _integer_range(draws, "world/chokepoint_count", source["chokepoint_count"])
    choke_width, choke_height = config["chokepoint_size_cells"]
    choke_candidates = config["chokepoint_candidates_cells"]
    choke_order = sorted(range(len(choke_candidates)), key=lambda index: draws.rank("world/chokepoint_order", index))
    for ordinal, candidate_index in enumerate(choke_order[:chokepoint_count]):
        x, y = choke_candidates[candidate_index]
        regions.append(_region(f"choke-{ordinal:02d}", "chokepoint", [x, y], [x + choke_width, y + choke_height]))
    for region in sorted(regions, key=lambda value: value.id):
        world.regions.add().CopyFrom(region)
    location_count = _integer_range(
        draws, "world/location_count", source["charging_or_return_location_count"]
    )
    locations = []
    return_location = sentinel_pb2.ServiceLocation(
        id="return-00", kind=sentinel_pb2.LOCATION_KIND_RETURN, radius_mm=config["location_radius_mm"]
    )
    _point(return_location.position, config["return_location_cells"])
    locations.append(return_location)
    for index, cells in enumerate(config["charging_location_cells"][: location_count - 1]):
        location = sentinel_pb2.ServiceLocation(
            id=f"charging-{index:02d}",
            kind=sentinel_pb2.LOCATION_KIND_CHARGING,
            radius_mm=config["location_radius_mm"],
            charge_mj_per_tick=_integer_range(
                draws, f"world/location/{index}/charge", config["charging_rate_mj_per_tick"]
            ),
        )
        _point(location.position, cells)
        locations.append(location)
    for location in sorted(locations, key=lambda value: value.id):
        world.locations.add().CopyFrom(location)
    return world


def _network_profile(draws, profile_id):
    source = STRATA_CONFIG["network_profiles"][profile_id]
    step_ms = GENERATION_CONFIG["clock"]["step_ms"]
    latency_ms = _integer_range(draws, f"network/{profile_id}/latency_ms", source["latency_ms"])
    jitter_ms = _integer_range(draws, f"network/{profile_id}/jitter_ms", source["jitter_ms"])
    bandwidth = _integer_range(
        draws, f"network/{profile_id}/bandwidth_bytes_per_second", source["bandwidth_bytes_per_second"]
    )
    loss = _fixed_range(
        draws,
        f"network/{profile_id}/packet_loss_probability",
        source["packet_loss_probability"],
        GENERATION_CONFIG["quantization"]["probability_scale"],
    )
    reorder = _fixed_range(
        draws,
        f"network/{profile_id}/reorder_probability",
        source["reorder_probability"],
        GENERATION_CONFIG["quantization"]["probability_scale"],
    )
    window = 0
    if reorder:
        window = _integer_range(
            draws, f"network/{profile_id}/reorder_window_messages", source["reorder_window_messages"]
        )
    return sentinel_pb2.NetworkProfile(
        id=profile_id,
        latency_ticks=(latency_ms + step_ms - 1) // step_ms,
        jitter_ticks=(jitter_ms + step_ms - 1) // step_ms,
        packet_loss_permyriad=loss,
        bandwidth_bytes_per_tick=bandwidth * step_ms // 1000,
        reorder_permyriad=reorder,
        reorder_window_ticks=window,
    )


def _vehicles(draws, count, anchor_speed):
    config = GENERATION_CONFIG["vehicles"]
    vehicles = []
    for index in range(count):
        kind = config["class_order"][index % len(config["class_order"])]
        values = config[kind]
        speed = _integer_range(draws, f"vehicles/{index}/speed", values["speed_mm_s"])
        if anchor_speed and index == config["anchor_agent_index"]:
            speed = anchor_speed
        vehicle = sentinel_pb2.VehicleSpec(
            id=f"agent-{index:02d}",
            kind=kind,
            max_speed_mm_s=speed,
            initial_energy_mj=_integer_range(draws, f"vehicles/{index}/energy", values["initial_energy_mj"]),
            energy_cost_mj_per_meter=_integer_range(
                draws, f"vehicles/{index}/energy_cost", values["energy_cost_mj_per_meter"]
            ),
            payload_grams=_integer_range(draws, f"vehicles/{index}/payload", values["payload_grams"]),
            return_location_id="return-00",
        )
        _point(vehicle.initial_position, config["initial_positions_cells"][index])
        capabilities = sorted(CAPABILITIES[name] for name in config["capability_templates"][index])
        vehicle.capabilities.extend(capabilities)
        vehicle.terrain_access.extend(sorted(values["terrain_access"]))
        vehicles.append(vehicle)
    return vehicles


def _event_seconds(draws, path, values):
    return _integer_range(draws, path, values) * 10


def _failure_kind(draws, profile):
    if not profile["agent_failures"]:
        return None
    types = profile["agent_failures"][0]["types"]
    value = draws.integer("events/failure/type", 0, 9999)
    total = 0
    for item in types:
        total += int(Decimal(str(item["weight"])) * 10000)
        if value < total:
            return item
    raise ValueError("invalid failure weights")


def _event_plan(record, draws, vehicle_ids, choke_ids, initial_task_count):
    stratum = _stratum(record)
    profile = STRATA_CONFIG["event_profiles"][stratum["event_profile"]]
    events = []
    arrivals = []
    horizon = GENERATION_CONFIG["clock"]["max_ticks"]
    partition_specs = profile["partitions"]
    if partition_specs:
        spec = partition_specs[0]
        count = _integer_range(draws, "events/partition/count", spec["count"])
        starts = []
        durations = []
        if count == 1:
            starts.append(_event_seconds(draws, "events/partition/0/start", spec["start_seconds"]))
            durations.append(_event_seconds(draws, "events/partition/0/duration", spec["duration_seconds"]))
        else:
            duration = _event_seconds(draws, "events/partition/0/duration", spec["duration_seconds"])
            maximum_first = spec["start_seconds"][1] * 10 - spec["duration_seconds"][1] * 10 - 10
            first = draws.integer("events/partition/0/start", spec["start_seconds"][0] * 10, maximum_first)
            second = draws.integer(
                "events/partition/1/start", first + duration + 10, spec["start_seconds"][1] * 10
            )
            starts.extend([first, second])
            durations.extend(
                [duration, _event_seconds(draws, "events/partition/1/duration", spec["duration_seconds"])]
            )
        for index in range(count):
            if record["stratum"] == "compound_disruption":
                first = vehicle_ids[:2]
                second = vehicle_ids[2:]
            else:
                order = sorted(vehicle_ids, key=lambda value: draws.rank(f"events/partition/{index}/order", value))
                split = draws.integer(f"events/partition/{index}/split", 1, len(order) - 1)
                first = order[:split]
                second = order[split:]
            end = min(starts[index] + durations[index], horizon)
            for left in sorted(first):
                for right in sorted(second):
                    pair = sorted([left, right])
                    event = sentinel_pb2.TapeEvent(
                        id=f"partition-{index}-{pair[0]}-{pair[1]}-start",
                        tick=starts[index],
                        kind=sentinel_pb2.TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                        target_id=pair[0],
                        text_value=pair[1],
                        bool_value=True,
                    )
                    events.append(event)
                    if end < horizon:
                        events.append(
                            sentinel_pb2.TapeEvent(
                                id=f"partition-{index}-{pair[0]}-{pair[1]}-end",
                                tick=end,
                                kind=sentinel_pb2.TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                                target_id=pair[0],
                                text_value=pair[1],
                            )
                        )
    blockage_specs = profile["route_blockages"]
    if blockage_specs:
        spec = blockage_specs[0]
        count = _integer_range(draws, "events/blockage/count", spec["count"])
        order = sorted(choke_ids, key=lambda value: draws.rank("events/blockage/order", value))
        for index in range(count):
            start = _event_seconds(draws, f"events/blockage/{index}/start", spec["start_seconds"])
            duration = _event_seconds(draws, f"events/blockage/{index}/duration", spec["duration_seconds"])
            end = min(start + duration, horizon)
            target = order[index]
            events.append(
                sentinel_pb2.TapeEvent(
                    id=f"blockage-{index}-start",
                    tick=start,
                    kind=sentinel_pb2.TAPE_EVENT_KIND_SET_REGION_CLOSED,
                    target_id=target,
                    bool_value=True,
                )
            )
            if end < horizon:
                events.append(
                    sentinel_pb2.TapeEvent(
                        id=f"blockage-{index}-end",
                        tick=end,
                        kind=sentinel_pb2.TAPE_EVENT_KIND_SET_REGION_CLOSED,
                        target_id=target,
                    )
                )
    arrival_specs = profile["task_arrivals"]
    if arrival_specs:
        spec = arrival_specs[0]
        count = _integer_range(draws, "events/arrival/count", spec["count"])
        spacing = int(spec.get("minimum_spacing_seconds", 0)) * 10
        start_min = spec["start_seconds"][0] * 10
        start_max = spec["start_seconds"][1] * 10 - spacing * (count - 1)
        base = draws.integer("events/arrival/base", start_min, start_max)
        for index in range(count):
            tick = base + spacing * index
            task_id = f"task-{initial_task_count + index:03d}"
            events.append(
                sentinel_pb2.TapeEvent(
                    id=f"arrival-{index}",
                    tick=tick,
                    kind=sentinel_pb2.TAPE_EVENT_KIND_RELEASE_TASK,
                    target_id=task_id,
                )
            )
            arrivals.append((task_id, tick, spec, index))
    failure_specs = profile["agent_failures"]
    failure = None
    if failure_specs:
        spec = failure_specs[0]
        kind = _failure_kind(draws, profile)
        tick = _event_seconds(draws, "events/failure/start", spec["start_seconds"])
        removal = kind["type"] == "removal"
        target = vehicle_ids[GENERATION_CONFIG["vehicles"]["anchor_agent_index"] if removal else GENERATION_CONFIG["vehicles"]["degradation_agent_index"]]
        event_kind = {
            "removal": sentinel_pb2.TAPE_EVENT_KIND_DISABLE_VEHICLE,
            "speed_degradation": sentinel_pb2.TAPE_EVENT_KIND_SET_SPEED_PERMILLE,
            "endurance_degradation": sentinel_pb2.TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE,
        }[kind["type"]]
        event = sentinel_pb2.TapeEvent(id="failure-0", tick=tick, kind=event_kind, target_id=target)
        if "multiplier" in kind:
            scale = GENERATION_CONFIG["quantization"]["multiplier_scale"]
            event.value_min = int(Decimal(str(kind["multiplier"][0])) * scale)
            event.value_max = int(Decimal(str(kind["multiplier"][1])) * scale) - 1
            event.rng_stream = event.id
        events.append(event)
        failure = (kind["type"], tick, target)
    return sorted(events, key=lambda value: (value.tick, value.id)), arrivals, failure


def _task(draws, identifier, index, task_type, target, released, priority, service_range, assigned=""):
    config = GENERATION_CONFIG["tasks"]
    service_ticks = _integer_range(draws, f"tasks/{identifier}/service_ticks", service_range)
    service_energy = _integer_range(
        draws, f"tasks/{identifier}/service_energy", config["service_energy_mj_per_tick"]
    )
    payload = 0
    if task_type == "delivery":
        payload = _integer_range(draws, f"tasks/{identifier}/payload", config["delivery_payload_grams"])
    value = sentinel_pb2.TaskSpec(
        id=identifier,
        kind=task_type,
        required_capability=CAPABILITIES[task_type],
        payload_required_grams=payload,
        deadline_tick=config["deadline_tick"],
        completion_radius_mm=config["completion_radius_mm"],
        assigned_agent_id=assigned,
        released=released,
        service_ticks=service_ticks,
        service_energy_mj_per_tick=service_energy,
        priority=priority,
    )
    _point(value.target, target)
    return value


def _tasks(record, draws, initial_count, arrivals, anchor):
    config = GENERATION_CONFIG["tasks"]
    types = config["type_order"]
    targets = config["target_candidates_cells"]
    tasks = []
    for index in range(initial_count):
        identifier = f"task-{index:03d}"
        task_type = types[index % len(types)]
        target = targets[draws.integer(f"tasks/{identifier}/target", 0, len(targets) - 1)]
        priority = _integer_range(draws, f"tasks/{identifier}/priority", config["initial_priority"])
        assigned = ""
        service_range = config["service_ticks"]
        if anchor and index == 0:
            task_type = "search"
            target = config["anchor_target_cells"]
            priority = config["anchor_priority"]
            assigned = "agent-00"
            service_range = [config["anchor_service_ticks"], config["anchor_service_ticks"]]
        task = _task(draws, identifier, index, task_type, target, True, priority, service_range, assigned)
        if anchor and index == 0:
            task.service_energy_mj_per_tick = config["anchor_service_energy_mj_per_tick"]
        tasks.append(task)
    for identifier, tick, spec, index in arrivals:
        if spec["task_type"] == "uniform":
            task_type = types[draws.integer(f"tasks/{identifier}/type", 0, len(types) - 1)]
        else:
            raise ValueError("unknown arrival task type rule")
        target = targets[draws.integer(f"tasks/{identifier}/target", 0, len(targets) - 1)]
        priority = _integer_range(draws, f"tasks/{identifier}/priority", spec["priority"])
        task = _task(
            draws,
            identifier,
            initial_count + index,
            task_type,
            target,
            False,
            priority,
            config["arrival_service_ticks"],
        )
        tasks.append(task)
    return sorted(tasks, key=lambda value: value.id)


def _inside(region, x, y):
    return region.minimum.x_mm <= x <= region.maximum.x_mm and region.minimum.y_mm <= y <= region.maximum.y_mm


def _distances(scenario, vehicle):
    cell = scenario.world.grid_cell_mm
    columns = scenario.world.width_mm // cell + 1
    rows = scenario.world.height_mm // cell + 1
    access = set(vehicle.terrain_access)
    blocked = [False] * (columns * rows)
    for y in range(rows):
        py = y * cell
        for x in range(columns):
            px = x * cell
            for region in scenario.world.regions:
                if not _inside(region, px, py):
                    continue
                if region.closed or region.kind in {sentinel_pb2.REGION_KIND_OBSTACLE, sentinel_pb2.REGION_KIND_RESTRICTED}:
                    blocked[y * columns + x] = True
                    break
                if region.kind == sentinel_pb2.REGION_KIND_TERRAIN and region.terrain not in access:
                    blocked[y * columns + x] = True
                    break
    start_x = vehicle.initial_position.x_mm // cell
    start_y = vehicle.initial_position.y_mm // cell
    start = start_y * columns + start_x
    if blocked[start]:
        raise ValueError("vehicle starts in a blocked cell")
    result = [-1] * len(blocked)
    result[start] = 0
    queue = deque([start])
    while queue:
        current = queue.popleft()
        x = current % columns
        y = current // columns
        for next_x, next_y in ((x - 1, y), (x, y - 1), (x, y + 1), (x + 1, y)):
            if next_x < 0 or next_y < 0 or next_x >= columns or next_y >= rows:
                continue
            target = next_y * columns + next_x
            if blocked[target] or result[target] >= 0:
                continue
            result[target] = result[current] + 1
            queue.append(target)
    return result, columns


def _activation_ticks(scenario):
    result = {task.id: 0 for task in scenario.tasks if task.released}
    for event in scenario.events:
        if event.kind == sentinel_pb2.TAPE_EVENT_KIND_RELEASE_TASK:
            result[event.target_id] = event.tick
    return result


def validate_generated(scenario, record, policy=None):
    if record["id"] != scenario.name or int(record["seed"]) != scenario.seed:
        raise ValueError("scenario identity does not match seed record")
    if record["stratum"] != _stratum(record)["id"]:
        raise ValueError("scenario stratum mismatch")
    if policy is not None:
        if policy not in POLICIES or scenario.allocation_policy != POLICIES[policy]:
            raise ValueError("scenario allocation policy mismatch")
    clock = GENERATION_CONFIG["clock"]
    if scenario.schema_version != 1 or scenario.step_ms != clock["step_ms"] or scenario.max_ticks != clock["max_ticks"]:
        raise ValueError("invalid generated clock")
    if scenario.failure_detection_ticks != clock["failure_detection_ticks"]:
        raise ValueError("invalid failure detector interval")
    stratum = _stratum(record)
    if scenario.network_profile != stratum["network_profile"] or len(scenario.network_profiles) != 1:
        raise ValueError("invalid generated network profile")
    profile = scenario.network_profiles[0]
    if profile.id != scenario.network_profile or profile.bandwidth_bytes_per_tick <= 0:
        raise ValueError("invalid generated network values")
    if profile.reorder_permyriad and not profile.reorder_window_ticks:
        raise ValueError("invalid generated reorder window")
    if not 3 <= len(scenario.vehicles) <= 5 or {vehicle.kind for vehicle in scenario.vehicles} != {"uav", "ugv"}:
        raise ValueError("generated fleet is not mixed")
    identifiers = [vehicle.id for vehicle in scenario.vehicles]
    if identifiers != sorted(identifiers) or len(identifiers) != len(set(identifiers)):
        raise ValueError("invalid generated vehicle IDs")
    initial_tasks = [task for task in scenario.tasks if task.released]
    initial_range = STRATA_CONFIG["mission"]["initial_task_count"]
    if not initial_range[0] <= len(initial_tasks) <= initial_range[1]:
        raise ValueError("invalid generated initial task count")
    counts = Counter(task.kind for task in initial_tasks)
    if any(counts[name] < STRATA_CONFIG["mission"]["minimum_each_task_type"] for name in GENERATION_CONFIG["tasks"]["type_order"]):
        raise ValueError("generated task type minimum is missing")
    task_ids = [task.id for task in scenario.tasks]
    event_ids = [event.id for event in scenario.events]
    region_ids = [region.id for region in scenario.world.regions]
    location_ids = [location.id for location in scenario.world.locations]
    if task_ids != sorted(task_ids) or len(task_ids) != len(set(task_ids)):
        raise ValueError("invalid generated task IDs")
    if len(event_ids) != len(set(event_ids)) or len(region_ids) != len(set(region_ids)) or len(location_ids) != len(set(location_ids)):
        raise ValueError("duplicate generated ID")
    if list(scenario.events) != sorted(scenario.events, key=lambda value: (value.tick, value.id)):
        raise ValueError("generated events are not canonical")
    world_source = STRATA_CONFIG["mission"]["world"]
    obstacle = next(region for region in scenario.world.regions if region.id == "obstacle-band")
    restricted = next(region for region in scenario.world.regions if region.id == "restricted-band")
    obstacle_fraction = Decimal(obstacle.maximum.y_mm - obstacle.minimum.y_mm) / scenario.world.height_mm
    restricted_fraction = Decimal(restricted.maximum.y_mm - restricted.minimum.y_mm) / scenario.world.height_mm
    if not Decimal(str(world_source["obstacle_fraction"][0])) <= obstacle_fraction < Decimal(str(world_source["obstacle_fraction"][1])):
        raise ValueError("generated obstacle fraction is out of range")
    if not Decimal(str(world_source["restricted_fraction"][0])) <= restricted_fraction < Decimal(str(world_source["restricted_fraction"][1])):
        raise ValueError("generated restricted fraction is out of range")
    terrain_count = sum(region.kind == sentinel_pb2.REGION_KIND_TERRAIN for region in scenario.world.regions)
    choke_count = sum(region.kind == sentinel_pb2.REGION_KIND_CHOKEPOINT for region in scenario.world.regions)
    if not world_source["terrain_region_count"][0] <= terrain_count <= world_source["terrain_region_count"][1]:
        raise ValueError("invalid generated terrain count")
    if not world_source["chokepoint_count"][0] <= choke_count <= world_source["chokepoint_count"][1]:
        raise ValueError("invalid generated chokepoint count")
    location_range = world_source["charging_or_return_location_count"]
    if not location_range[0] <= len(scenario.world.locations) <= location_range[1]:
        raise ValueError("invalid generated location count")
    distance_maps = {vehicle.id: _distances(scenario, vehicle) for vehicle in scenario.vehicles}
    activation = _activation_ticks(scenario)
    cell = scenario.world.grid_cell_mm
    numerator = GENERATION_CONFIG["tasks"]["deadline_lower_bound_numerator"]
    denominator = GENERATION_CONFIG["tasks"]["deadline_lower_bound_denominator"]
    vehicles = {vehicle.id: vehicle for vehicle in scenario.vehicles}
    for task in scenario.tasks:
        if task.id not in activation:
            raise ValueError("generated task has no activation")
        feasible = []
        target_x = task.target.x_mm // cell
        target_y = task.target.y_mm // cell
        for vehicle in scenario.vehicles:
            if task.required_capability not in vehicle.capabilities or vehicle.payload_grams < task.payload_required_grams:
                continue
            distances, columns = distance_maps[vehicle.id]
            steps = distances[target_y * columns + target_x]
            if steps < 0:
                continue
            edge_ticks = (cell * 1000 + vehicle.max_speed_mm_s * scenario.step_ms - 1) // (
                vehicle.max_speed_mm_s * scenario.step_ms
            )
            duration = steps * edge_ticks + task.service_ticks
            energy = steps * vehicle.energy_cost_mj_per_meter * 1450 // 1000
            energy += task.service_ticks * task.service_energy_mj_per_tick
            reserve = (vehicle.initial_energy_mj + 9) // 10
            if activation[task.id] + duration <= task.deadline_tick and energy + reserve <= vehicle.initial_energy_mj:
                feasible.append((duration, vehicle.id))
        if not feasible:
            raise ValueError("generated task is infeasible")
        earliest = min(value[0] for value in feasible)
        required = (earliest * numerator + denominator - 1) // denominator
        if task.deadline_tick - activation[task.id] < required:
            raise ValueError("generated task deadline is too short")
        if task.assigned_agent_id:
            assigned = vehicles[task.assigned_agent_id]
            distances, columns = distance_maps[assigned.id]
            steps = distances[target_y * columns + target_x]
            edge_ticks = (cell * 1000 + assigned.max_speed_mm_s * scenario.step_ms - 1) // (
                assigned.max_speed_mm_s * scenario.step_ms
            )
            if steps < 0 or activation[task.id] + steps * edge_ticks + task.service_ticks > task.deadline_tick:
                raise ValueError("generated fixed assignment is infeasible")
    failures = [event for event in scenario.events if event.kind == sentinel_pb2.TAPE_EVENT_KIND_DISABLE_VEHICLE]
    if failures:
        if len(failures) != 1 or failures[0].target_id != "agent-00":
            raise ValueError("invalid generated removal")
        anchors = [task for task in initial_tasks if task.assigned_agent_id == "agent-00"]
        if len(anchors) != 1:
            raise ValueError("removal has no unique orphan anchor")
        anchor = anchors[0]
        assigned = vehicles["agent-00"]
        distances, columns = distance_maps[assigned.id]
        target = anchor.target.y_mm // cell * columns + anchor.target.x_mm // cell
        edge_ticks = (cell * 1000 + assigned.max_speed_mm_s * scenario.step_ms - 1) // (
            assigned.max_speed_mm_s * scenario.step_ms
        )
        if distances[target] * edge_ticks + anchor.service_ticks <= failures[0].tick:
            raise ValueError("removal anchor completes before failure")
        recovery = vehicles["agent-01"]
        if anchor.required_capability not in recovery.capabilities or recovery.payload_grams < anchor.payload_required_grams:
            raise ValueError("removal anchor has no recovery agent")
        recovery_distances, recovery_columns = distance_maps[recovery.id]
        recovery_target = anchor.target.y_mm // cell * recovery_columns + anchor.target.x_mm // cell
        recovery_edge = (cell * 1000 + recovery.max_speed_mm_s * scenario.step_ms - 1) // (
            recovery.max_speed_mm_s * scenario.step_ms
        )
        recovery_ticks = recovery_distances[recovery_target] * recovery_edge + anchor.service_ticks
        if failures[0].tick + scenario.failure_detection_ticks + 100 + recovery_ticks > scenario.max_ticks:
            raise ValueError("removal anchor cannot finish after recovery")
    return True


def generate_scenario(record, policy, validate=True):
    if policy not in POLICIES:
        raise ValueError("unknown allocation policy")
    draws = Draws(int(record["seed"]))
    stratum = _stratum(record)
    mission = STRATA_CONFIG["mission"]
    vehicle_count = _integer_range(draws, "mission/agent_count", mission["agent_count"])
    initial_task_count = _integer_range(draws, "mission/initial_task_count", mission["initial_task_count"])
    anchor = record["stratum"] in GENERATION_CONFIG["tasks"]["anchor_strata"]
    anchor_speed = GENERATION_CONFIG["vehicles"]["anchor_speed_mm_s"].get(record["stratum"])
    world = _world(draws)
    vehicles = _vehicles(draws, vehicle_count, anchor_speed)
    vehicle_ids = [vehicle.id for vehicle in vehicles]
    choke_ids = [region.id for region in world.regions if region.kind == sentinel_pb2.REGION_KIND_CHOKEPOINT]
    events, arrivals, failure = _event_plan(record, draws, vehicle_ids, choke_ids, initial_task_count)
    tasks = _tasks(record, draws, initial_task_count, arrivals, anchor)
    scenario = sentinel_pb2.Scenario(
        schema_version=1,
        name=record["id"],
        seed=int(record["seed"]),
        step_ms=GENERATION_CONFIG["clock"]["step_ms"],
        max_ticks=GENERATION_CONFIG["clock"]["max_ticks"],
        network_profile=stratum["network_profile"],
        allocation_policy=POLICIES[policy],
        failure_detection_ticks=GENERATION_CONFIG["clock"]["failure_detection_ticks"],
    )
    scenario.world.CopyFrom(world)
    scenario.vehicles.extend(vehicles)
    scenario.tasks.extend(tasks)
    scenario.events.extend(events)
    scenario.network_profiles.add().CopyFrom(_network_profile(draws, stratum["network_profile"]))
    if validate:
        validate_generated(scenario, record, policy)
    return scenario


def paired_digest(scenario):
    common = sentinel_pb2.Scenario()
    common.CopyFrom(scenario)
    common.ClearField("allocation_policy")
    return hashlib.sha256(common.SerializeToString(deterministic=True)).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--id", action="append", default=[])
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=1)
    arguments = parser.parse_args()
    records = load_seed_records("development")
    if arguments.id:
        selected = {record["id"]: record for record in records}
        records = [selected[identifier] for identifier in arguments.id]
    else:
        if arguments.start < 0 or arguments.count <= 0:
            raise ValueError("invalid development selection")
        records = records[arguments.start : arguments.start + arguments.count]
    if not records:
        raise ValueError("development selection is empty")
    arguments.output.mkdir(parents=True, exist_ok=True)
    manifest = []
    for record in records:
        digests = set()
        for policy in POLICIES:
            scenario = generate_scenario(record, policy)
            digests.add(paired_digest(scenario))
            path = arguments.output / f"{record['id']}-{policy}.textproto"
            path.write_text(text_format.MessageToString(scenario), encoding="utf-8", newline="\n")
        if len(digests) != 1:
            raise ValueError("paired scenario generation diverged")
        manifest.append({"id": record["id"], "seed": record["seed"], "stratum": record["stratum"], "paired_sha256": digests.pop()})
    (arguments.output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"generated_pairs": len(records), "output": str(arguments.output)}, sort_keys=True))


if __name__ == "__main__":
    main()
