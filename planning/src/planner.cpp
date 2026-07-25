#include <sentinel/planning/planner.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace sentinel::planning {
namespace {

Coordinate coordinate(std::int64_t x, std::int64_t y) {
    Coordinate result;
    result << x, y;
    return result;
}

bool contains(const v1::Region& region, const Coordinate& point) {
    return point.x() >= region.minimum().x_mm() && point.x() <= region.maximum().x_mm()
           && point.y() >= region.minimum().y_mm() && point.y() <= region.maximum().y_mm();
}

int orientation(const Coordinate& first, const Coordinate& second, const Coordinate& third) {
    const auto cross = (second.x() - first.x()) * (third.y() - first.y())
                       - (second.y() - first.y()) * (third.x() - first.x());
    return (cross > 0) - (cross < 0);
}

bool on_segment(const Coordinate& first, const Coordinate& second, const Coordinate& point) {
    return point.x() >= std::min(first.x(), second.x()) && point.x() <= std::max(first.x(), second.x())
           && point.y() >= std::min(first.y(), second.y()) && point.y() <= std::max(first.y(), second.y());
}

bool segments_intersect(const Coordinate& first_start, const Coordinate& first_end,
                        const Coordinate& second_start, const Coordinate& second_end) {
    const auto first = orientation(first_start, first_end, second_start);
    const auto second = orientation(first_start, first_end, second_end);
    const auto third = orientation(second_start, second_end, first_start);
    const auto fourth = orientation(second_start, second_end, first_end);
    if ((first == 0 && on_segment(first_start, first_end, second_start))
        || (second == 0 && on_segment(first_start, first_end, second_end))
        || (third == 0 && on_segment(second_start, second_end, first_start))
        || (fourth == 0 && on_segment(second_start, second_end, first_end))) {
        return true;
    }
    return ((first < 0 && second > 0) || (first > 0 && second < 0))
           && ((third < 0 && fourth > 0) || (third > 0 && fourth < 0));
}

bool intersects(const v1::Region& region, const Coordinate& from, const Coordinate& to) {
    if (contains(region, from) || contains(region, to)) {
        return true;
    }
    const auto lower_left = coordinate(region.minimum().x_mm(), region.minimum().y_mm());
    const auto lower_right = coordinate(region.maximum().x_mm(), region.minimum().y_mm());
    const auto upper_left = coordinate(region.minimum().x_mm(), region.maximum().y_mm());
    const auto upper_right = coordinate(region.maximum().x_mm(), region.maximum().y_mm());
    return segments_intersect(from, to, lower_left, lower_right)
           || segments_intersect(from, to, lower_right, upper_right)
           || segments_intersect(from, to, upper_right, upper_left)
           || segments_intersect(from, to, upper_left, lower_left);
}

bool has_capability(const v1::VehicleState& vehicle, v1::Capability capability) {
    return std::find(vehicle.capabilities().begin(), vehicle.capabilities().end(), capability)
           != vehicle.capabilities().end();
}

bool has_terrain(const v1::VehicleState& vehicle, const std::string& terrain) {
    return std::find(vehicle.terrain_access().begin(), vehicle.terrain_access().end(), terrain)
           != vehicle.terrain_access().end();
}

std::uint64_t edge_ticks(std::int64_t distance_mm, const v1::VehicleState& vehicle, std::uint64_t step_ms) {
    const auto numerator = static_cast<std::uint64_t>(distance_mm) * 1000U;
    const auto denominator = static_cast<std::uint64_t>(vehicle.max_speed_mm_s()) * step_ms;
    // Round partial travel steps up to a full tick.
    return numerator / denominator + (numerator % denominator != 0U);
}

struct OpenEntry {
    std::int64_t estimate;
    std::int64_t cost;
    std::int64_t x;
    std::int64_t y;
    std::size_t index;
};

struct OpenEntryOrder {
    bool operator()(const OpenEntry& lhs, const OpenEntry& rhs) const {
        // stable A* tie-breaks
        return std::tie(lhs.estimate, lhs.cost, lhs.x, lhs.y, lhs.index)
               > std::tie(rhs.estimate, rhs.cost, rhs.x, rhs.y, rhs.index);
    }
};

bool same_reservation(const Reservation& left, const Reservation& right) {
    return left.resource_id == right.resource_id && left.agent_id == right.agent_id
           && left.start_tick == right.start_tick && left.end_tick == right.end_tick
           && left.version == right.version && left.route_version == right.route_version
           && left.map_version == right.map_version;
}

bool overlaps(const Reservation& left, const Reservation& right) {
    return left.resource_id == right.resource_id && left.start_tick <= right.end_tick
           && right.start_tick <= left.end_tick;
}

bool higher_priority(const Reservation& left, const Reservation& right) {
    if (left.start_tick != right.start_tick) {
        return left.start_tick < right.start_tick;
    }
    if (left.agent_id != right.agent_id) {
        return left.agent_id < right.agent_id;
    }
    return left.version > right.version;
}

bool reservation_order(const Reservation& left, const Reservation& right) {
    return std::tie(left.resource_id, left.start_tick, left.end_tick, left.agent_id, left.version,
                    left.route_version, left.map_version)
           < std::tie(right.resource_id, right.start_tick, right.end_tick, right.agent_id, right.version,
                      right.route_version, right.map_version);
}

}

bool segment_allowed(const v1::World& world, const v1::VehicleState& vehicle, const Coordinate& from,
                     const Coordinate& to) {
    if (from.x() < 0 || from.y() < 0 || to.x() < 0 || to.y() < 0 || from.x() > world.width_mm()
        || to.x() > world.width_mm() || from.y() > world.height_mm() || to.y() > world.height_mm()) {
        return false;
    }
    for (const auto& region : world.regions()) {
        if (!intersects(region, from, to)) {
            continue;
        }
        if (region.closed() || region.kind() == v1::REGION_KIND_OBSTACLE
            || region.kind() == v1::REGION_KIND_RESTRICTED) {
            return false;
        }
        if (region.kind() == v1::REGION_KIND_TERRAIN && !has_terrain(vehicle, region.terrain())) {
            return false;
        }
    }
    return true;
}

std::uint32_t terrain_multiplier(const v1::World& world, const Coordinate& from, const Coordinate& to) {
    std::uint32_t multiplier = 1000;
    for (const auto& region : world.regions()) {
        if (region.kind() == v1::REGION_KIND_TERRAIN && intersects(region, from, to)) {
            multiplier = std::max(multiplier, region.energy_multiplier_permille());
        }
    }
    return multiplier;
}

std::int64_t motion_energy(const v1::VehicleState& vehicle, std::int64_t distance_mm,
                           std::uint32_t multiplier_permille) {
    if (distance_mm < 0 || vehicle.energy_cost_mj_per_meter() < 0) {
        throw std::invalid_argument("invalid motion energy input");
    }
    const auto base = distance_mm * vehicle.energy_cost_mj_per_meter() / 1000;
    return base * static_cast<std::int64_t>(multiplier_permille) / 1000;
}

std::optional<Route> astar(const v1::World& world, const v1::VehicleState& vehicle, const Coordinate& start,
                           const Coordinate& goal, std::uint64_t step_ms) {
    const auto cell = world.grid_cell_mm();
    if (cell <= 0 || step_ms == 0 || vehicle.max_speed_mm_s() <= 0 || start.x() % cell != 0
        || start.y() % cell != 0 || goal.x() % cell != 0 || goal.y() % cell != 0
        || !segment_allowed(world, vehicle, start, start) || !segment_allowed(world, vehicle, goal, goal)) {
        return std::nullopt;
    }
    const auto columns = static_cast<std::size_t>(world.width_mm() / cell + 1);
    const auto rows = static_cast<std::size_t>(world.height_mm() / cell + 1);
    const auto node_count = columns * rows;
    const auto start_x = static_cast<std::size_t>(start.x() / cell);
    const auto start_y = static_cast<std::size_t>(start.y() / cell);
    const auto goal_x = static_cast<std::size_t>(goal.x() / cell);
    const auto goal_y = static_cast<std::size_t>(goal.y() / cell);
    if (start_x >= columns || start_y >= rows || goal_x >= columns || goal_y >= rows) {
        return std::nullopt;
    }
    const auto start_index = start_y * columns + start_x;
    const auto goal_index = goal_y * columns + goal_x;
    const auto missing = std::numeric_limits<std::size_t>::max();
    const auto infinity = std::numeric_limits<std::int64_t>::max();
    std::vector<std::int64_t> costs(node_count, infinity);
    std::vector<std::size_t> parents(node_count, missing);
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryOrder> open;
    const auto heuristic = [cell, goal_x, goal_y](std::size_t x, std::size_t y) {
        return (std::abs(static_cast<std::int64_t>(goal_x) - static_cast<std::int64_t>(x))
                + std::abs(static_cast<std::int64_t>(goal_y) - static_cast<std::int64_t>(y)))
               * cell;
    };
    costs[start_index] = 0;
    open.push(OpenEntry{heuristic(start_x, start_y), 0, start.x(), start.y(), start_index});
    constexpr std::array<std::array<std::int64_t, 2>, 4> directions{{{-1, 0}, {0, -1}, {0, 1}, {1, 0}}};
    while (!open.empty()) {
        const auto current = open.top();
        open.pop();
        if (current.cost != costs[current.index]) {
            continue;
        }
        if (current.index == goal_index) {
            break;
        }
        const auto x = current.index % columns;
        const auto y = current.index / columns;
        const auto from = coordinate(static_cast<std::int64_t>(x) * cell, static_cast<std::int64_t>(y) * cell);
        for (const auto& direction : directions) {
            const auto next_x = static_cast<std::int64_t>(x) + direction[0];
            const auto next_y = static_cast<std::int64_t>(y) + direction[1];
            if (next_x < 0 || next_y < 0 || next_x >= static_cast<std::int64_t>(columns)
                || next_y >= static_cast<std::int64_t>(rows)) {
                continue;
            }
            const auto to = coordinate(next_x * cell, next_y * cell);
            if (!segment_allowed(world, vehicle, from, to)) {
                continue;
            }
            const auto next_index = static_cast<std::size_t>(next_y) * columns + static_cast<std::size_t>(next_x);
            const auto step_cost = cell * static_cast<std::int64_t>(terrain_multiplier(world, from, to)) / 1000;
            const auto next_cost = current.cost + step_cost;
            if (next_cost > costs[next_index]
                || (next_cost == costs[next_index] && current.index >= parents[next_index])) {
                continue;
            }
            costs[next_index] = next_cost;
            parents[next_index] = current.index;
            open.push(OpenEntry{next_cost + heuristic(static_cast<std::size_t>(next_x),
                                                      static_cast<std::size_t>(next_y)),
                                next_cost, to.x(), to.y(), next_index});
        }
    }
    if (costs[goal_index] == infinity) {
        return std::nullopt;
    }
    Route route;
    for (auto idx = goal_index;; idx = parents[idx]) {
        route.points.push_back(coordinate(static_cast<std::int64_t>(idx % columns) * cell,
                                          static_cast<std::int64_t>(idx / columns) * cell));
        if (idx == start_index) {
            break;
        }
    }
    std::reverse(route.points.begin(), route.points.end());
    for (std::size_t i = 1; i < route.points.size(); ++i) {
        const auto distance = std::abs(route.points[i].x() - route.points[i - 1].x())
                              + std::abs(route.points[i].y() - route.points[i - 1].y());
        route.distance_mm += distance;
        route.energy_mj += motion_energy(vehicle, distance,
                                         terrain_multiplier(world, route.points[i - 1], route.points[i]));
        route.travel_ticks += edge_ticks(distance, vehicle, step_ms);
    }
    return route;
}

std::optional<Route> route_from(const v1::World& world, const v1::VehicleState& vehicle,
                                const Coordinate& start, const Coordinate& goal, std::uint64_t step_ms) {
    const auto cell = world.grid_cell_mm();
    if (cell <= 0) {
        return std::nullopt;
    }
    if (start.x() % cell == 0 && start.y() % cell == 0) {
        return astar(world, vehicle, start, goal, step_ms);
    }
    std::vector<Coordinate> anchors;
    if (start.x() % cell != 0 && start.y() % cell == 0) {
        anchors.push_back(coordinate(start.x() / cell * cell, start.y()));
        anchors.push_back(coordinate((start.x() / cell + 1) * cell, start.y()));
    } else if (start.y() % cell != 0 && start.x() % cell == 0) {
        anchors.push_back(coordinate(start.x(), start.y() / cell * cell));
        anchors.push_back(coordinate(start.x(), (start.y() / cell + 1) * cell));
    }
    std::optional<Route> best;
    Coordinate best_anchor = coordinate(0, 0);
    for (const auto& anchor : anchors) {
        if (!segment_allowed(world, vehicle, start, anchor)) {
            continue;
        }
        auto route = astar(world, vehicle, anchor, goal, step_ms);
        if (!route) {
            continue;
        }
        const auto partial = std::abs(start.x() - anchor.x()) + std::abs(start.y() - anchor.y());
        route->distance_mm += partial;
        route->energy_mj += motion_energy(vehicle, partial, terrain_multiplier(world, start, anchor));
        route->travel_ticks += edge_ticks(partial, vehicle, step_ms);
        route->points.insert(route->points.begin(), start);
        if (!best || std::tuple{route->distance_mm, anchor.x(), anchor.y()}
                         < std::tuple{best->distance_mm, best_anchor.x(), best_anchor.y()}) {
            best = std::move(route);
            best_anchor = anchor;
        }
    }
    return best;
}

std::int64_t reserve_energy(const v1::VehicleState& vehicle) {
    if (vehicle.initial_energy_mj() <= 0) {
        return 0;
    }
    return vehicle.initial_energy_mj() / 10 + (vehicle.initial_energy_mj() % 10 != 0);
}

std::optional<Route> feasible_route(const v1::World& world, const v1::VehicleState& vehicle,
                                    const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms) {
    if (!vehicle.active() || task.status() != v1::TASK_STATUS_PENDING
        || !has_capability(vehicle, task.required_capability())
        || vehicle.payload_grams() < task.payload_required_grams() || tick > task.deadline_tick()) {
        return std::nullopt;
    }
    const auto start = coordinate(vehicle.position().x_mm(), vehicle.position().y_mm());
    const auto goal = coordinate(task.target().x_mm(), task.target().y_mm());
    auto route = route_from(world, vehicle, start, goal, step_ms);
    if (!route) {
        return std::nullopt;
    }
    const auto remaining_service = task.service_ticks() > task.progress_ticks()
                                       ? task.service_ticks() - task.progress_ticks()
                                       : 0;
    const auto remaining_deadline = task.deadline_tick() - tick;
    if (route->travel_ticks > remaining_deadline
        || remaining_service > remaining_deadline - route->travel_ticks) {
        return std::nullopt;
    }
    const auto available = vehicle.energy_mj() - reserve_energy(vehicle);
    if (available < 0 || route->energy_mj > available) {
        return std::nullopt;
    }
    const auto service_energy = static_cast<std::int64_t>(remaining_service) * task.service_energy_mj_per_tick();
    if (service_energy < 0 || service_energy > available - route->energy_mj) {
        return std::nullopt;
    }
    return route;
}

v1::AllocationClaim make_claim(const v1::World& world, const v1::VehicleState& vehicle,
                               const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms,
                               std::uint64_t epoch, std::uint64_t version) {
    v1::AllocationClaim claim;
    claim.set_epoch(epoch);
    claim.set_version(version);
    claim.set_task_id(task.id());
    claim.set_agent_id(vehicle.id());
    const auto route = feasible_route(world, vehicle, task, tick, step_ms);
    claim.set_feasible(route.has_value());
    if (route) {
        claim.set_distance_mm(route->distance_mm);
    }
    return claim;
}

std::optional<v1::AllocationClaim> nearest_capable(const v1::World& world,
                                                   const std::vector<v1::VehicleState>& vehicles,
                                                   const v1::TaskState& task, std::uint64_t tick,
                                                   std::uint64_t step_ms, std::uint64_t epoch,
                                                   std::uint64_t version) {
    std::optional<v1::AllocationClaim> winner;
    for (const auto& vehicle : vehicles) {
        auto claim = make_claim(world, vehicle, task, tick, step_ms, epoch, version);
        if (!claim.feasible()) {
            continue;
        }
        if (!winner || std::tuple{claim.distance_mm(), claim.agent_id()}
                           < std::tuple{winner->distance_mm(), winner->agent_id()}) {
            winner = std::move(claim);
        }
    }
    return winner;
}

class BundleEvaluator::Impl {
public:
    Impl(const v1::World& world, const v1::VehicleState& vehicle, std::uint64_t tick,
         std::uint64_t step_ms)
        : world_(world), vehicle_(vehicle), tick_(tick), step_ms_(step_ms) {}

    std::optional<BundleEvaluation> evaluate(const std::vector<v1::TaskState>& ordered_tasks) {
        if (!vehicle_.active() || step_ms_ == 0) {
            return std::nullopt;
        }
        BundleEvaluation result;
        result.completion_tick = tick_;
        result.task_completion_ticks.reserve(ordered_tasks.size());
        auto position = coordinate(vehicle_.position().x_mm(), vehicle_.position().y_mm());
        auto payload = vehicle_.payload_grams();
        const auto available_energy = vehicle_.energy_mj() - reserve_energy(vehicle_);
        if (available_energy < 0) {
            return std::nullopt;
        }
        for (const auto& task : ordered_tasks) {
            if (task.status() != v1::TASK_STATUS_PENDING
                || !has_capability(vehicle_, task.required_capability())
                || payload < task.payload_required_grams() || result.completion_tick > task.deadline_tick()) {
                return std::nullopt;
            }
            const auto target = coordinate(task.target().x_mm(), task.target().y_mm());
            const auto& route = route_to(position, target);
            if (!route) {
                return std::nullopt;
            }
            const auto service_ticks = task.service_ticks() > task.progress_ticks()
                                           ? task.service_ticks() - task.progress_ticks()
                                           : 0;
            const auto service_energy = static_cast<std::int64_t>(service_ticks)
                                        * task.service_energy_mj_per_tick();
            result.distance_mm += route->distance_mm;
            result.energy_mj += route->energy_mj + service_energy;
            result.completion_tick += route->travel_ticks + service_ticks;
            if (service_energy < 0 || result.energy_mj > available_energy
                || result.completion_tick > task.deadline_tick()) {
                return std::nullopt;
            }
            result.task_completion_ticks.push_back(result.completion_tick);
            if (task.required_capability() == v1::CAPABILITY_DELIVERY) {
                payload -= task.payload_required_grams();
            }
            position = target;
        }
        return result;
    }

    std::optional<BundleInsertion> best_insertion(const std::vector<v1::TaskState>& ordered_tasks,
                                                  const v1::TaskState& candidate,
                                                  std::size_t minimum_index) {
        if (minimum_index > ordered_tasks.size()
            || std::any_of(ordered_tasks.begin(), ordered_tasks.end(), [&candidate](const auto& task) {
                return task.id() == candidate.id();
            })) {
            return std::nullopt;
        }
        const auto current = evaluate(ordered_tasks);
        if (!current) {
            return std::nullopt;
        }
        std::optional<BundleInsertion> best;
        for (std::size_t index = minimum_index; index <= ordered_tasks.size(); ++index) {
            auto tasks = ordered_tasks;
            tasks.insert(tasks.begin() + static_cast<std::ptrdiff_t>(index), candidate);
            const auto evaluated = evaluate(tasks);
            if (!evaluated) {
                continue;
            }
            std::uint64_t delay_ticks = 0;
            for (std::size_t task_index = 0; task_index < ordered_tasks.size(); ++task_index) {
                const auto inserted_index = task_index < index ? task_index : task_index + 1;
                if (evaluated->task_completion_ticks[inserted_index]
                    > current->task_completion_ticks[task_index]) {
                    delay_ticks += evaluated->task_completion_ticks[inserted_index]
                                   - current->task_completion_ticks[task_index];
                }
            }
            const auto distance = evaluated->distance_mm - current->distance_mm;
            const auto energy = evaluated->energy_mj - current->energy_mj;
            const auto completion = evaluated->task_completion_ticks[index];
            const auto slack = candidate.deadline_tick() - completion;
            const auto score = static_cast<std::int64_t>(candidate.priority()) * 1'000'000'000'000LL
                               + static_cast<std::int64_t>(slack) * 1'000'000LL
                               - static_cast<std::int64_t>(delay_ticks) * 1'000'000LL - distance - energy;
            BundleInsertion insertion{index, score, distance, energy, completion};
            if (!best || insertion.score > best->score
                || (insertion.score == best->score
                    && std::tuple{insertion.distance_mm, insertion.energy_mj,
                                  insertion.completion_tick, insertion.index}
                           < std::tuple{best->distance_mm, best->energy_mj,
                                       best->completion_tick, best->index})) {
                best = insertion;
            }
        }
        return best;
    }

private:
    const std::optional<Route>& route_to(const Coordinate& start, const Coordinate& goal) {
        const auto key = std::tuple{start.x(), start.y(), goal.x(), goal.y()};
        const auto position = routes_.find(key);
        if (position != routes_.end()) {
            return position->second;
        }
        return routes_.emplace(key, route_from(world_, vehicle_, start, goal, step_ms_)).first->second;
    }

    const v1::World& world_;
    const v1::VehicleState& vehicle_;
    std::uint64_t tick_;
    std::uint64_t step_ms_;
    std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t, std::int64_t>,
             std::optional<Route>> routes_;
};

BundleEvaluator::BundleEvaluator(const v1::World& world, const v1::VehicleState& vehicle,
                                 std::uint64_t tick, std::uint64_t step_ms)
    : impl_(std::make_unique<Impl>(world, vehicle, tick, step_ms)) {}

BundleEvaluator::~BundleEvaluator() = default;

std::optional<BundleEvaluation> BundleEvaluator::evaluate(
    const std::vector<v1::TaskState>& ordered_tasks) {
    return impl_->evaluate(ordered_tasks);
}

std::optional<BundleInsertion> BundleEvaluator::best_insertion(
    const std::vector<v1::TaskState>& ordered_tasks, const v1::TaskState& candidate,
    std::size_t minimum_index) {
    return impl_->best_insertion(ordered_tasks, candidate, minimum_index);
}

std::optional<BundleEvaluation> evaluate_bundle(const v1::World& world,
                                                 const v1::VehicleState& vehicle,
                                                 const std::vector<v1::TaskState>& ordered_tasks,
                                                 std::uint64_t tick, std::uint64_t step_ms) {
    return BundleEvaluator(world, vehicle, tick, step_ms).evaluate(ordered_tasks);
}

std::optional<BundleInsertion> best_insertion(const v1::World& world,
                                               const v1::VehicleState& vehicle,
                                               const std::vector<v1::TaskState>& ordered_tasks,
                                               const v1::TaskState& candidate, std::uint64_t tick,
                                               std::uint64_t step_ms, std::size_t minimum_index) {
    return BundleEvaluator(world, vehicle, tick, step_ms)
        .best_insertion(ordered_tasks, candidate, minimum_index);
}

std::optional<std::string> contested_resource(const v1::World& world, const Coordinate& from,
                                              const Coordinate& to) {
    std::optional<std::string> result;
    for (const auto& region : world.regions()) {
        if (region.kind() == v1::REGION_KIND_CHOKEPOINT && intersects(region, from, to)
            && (!result || region.id() < *result)) {
            result = region.id();
        }
    }
    return result;
}

ReservationResult ReservationTable::reserve(const Reservation& proposal) {
    if (proposal.resource_id.empty() || proposal.agent_id.empty() || proposal.version == 0
        || proposal.route_version == 0 || proposal.map_version == 0 || proposal.start_tick > proposal.end_tick) {
        throw std::invalid_argument("invalid reservation");
    }
    if (std::any_of(seen_.begin(), seen_.end(), [&proposal](const auto& current) {
            return same_reservation(current, proposal);
        })) {
        return ReservationResult::unchanged;
    }
    if (std::any_of(seen_.begin(), seen_.end(), [&proposal](const auto& current) {
            return current.agent_id == proposal.agent_id && current.version >= proposal.version;
        })) {
        return ReservationResult::stale;
    }
    seen_.push_back(proposal);
    std::sort(seen_.begin(), seen_.end(), reservation_order);
    std::vector<std::size_t> conflicts;
    for (std::size_t idx = 0; idx < committed_.size(); ++idx) {
        if (overlaps(committed_[idx], proposal)) {
            conflicts.push_back(idx);
        }
    }
    if (conflicts.empty()) {
        committed_.push_back(proposal);
        std::sort(committed_.begin(), committed_.end(), reservation_order);
        return ReservationResult::committed;
    }
    if (std::any_of(conflicts.begin(), conflicts.end(), [this, &proposal](std::size_t index) {
            // the proposal has to beat the entire overlap set.
            return !higher_priority(proposal, committed_[index]);
        })) {
        ++conflicts_;
        return ReservationResult::rejected;
    }
    conflicts_ += conflicts.size();
    for (auto position = conflicts.rbegin(); position != conflicts.rend(); ++position) {
        committed_.erase(committed_.begin() + static_cast<std::ptrdiff_t>(*position));
    }
    committed_.push_back(proposal);
    std::sort(committed_.begin(), committed_.end(), reservation_order);
    return ReservationResult::committed;
}

void ReservationTable::release_before(std::uint64_t tick) {
    std::erase_if(committed_, [tick](const auto& current) { return current.end_tick < tick; });
}

void ReservationTable::release_agent(std::string_view agent_id) {
    std::erase_if(committed_, [agent_id](const auto& current) { return current.agent_id == agent_id; });
}

void ReservationTable::release_route(std::string_view agent_id, std::uint64_t route_version) {
    std::erase_if(committed_, [agent_id, route_version](const auto& current) {
        return current.agent_id == agent_id && current.route_version == route_version;
    });
}

void ReservationTable::release_map(std::uint64_t map_version) {
    std::erase_if(committed_, [map_version](const auto& current) { return current.map_version != map_version; });
}

const std::vector<Reservation>& ReservationTable::committed() const {
    return committed_;
}

const std::vector<Reservation>& ReservationTable::seen() const {
    return seen_;
}

std::uint64_t ReservationTable::conflicts() const {
    return conflicts_;
}

}
