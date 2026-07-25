#include <sentinel/agent/autonomy.hpp>
#include <sentinel/agent/allocation.hpp>

#include <sentinel/planning/planner.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace sentinel::agent {
namespace {

using sentinel::planning::Coordinate;
using sentinel::planning::Route;

Coordinate coordinate(std::int64_t x, std::int64_t y) {
    Coordinate result;
    result << x, y;
    return result;
}

std::int64_t distance(std::int64_t first_x, std::int64_t first_y, std::int64_t second_x,
                      std::int64_t second_y) {
    return std::abs(first_x - second_x) + std::abs(first_y - second_y);
}

bool capable(const v1::VehicleState& vehicle, v1::Capability capability) {
    return std::find(vehicle.capabilities().begin(), vehicle.capabilities().end(), capability)
           != vehicle.capabilities().end();
}

std::int64_t velocity(std::int64_t delta, std::int64_t maximum, std::uint64_t step_ms) {
    const auto magnitude = std::abs(delta);
    if (magnitude == 0) {
        return 0;
    }
    const auto required = (magnitude * 1000 + static_cast<std::int64_t>(step_ms) - 1)
                          / static_cast<std::int64_t>(step_ms);
    const auto speed = std::min(maximum, required);
    return delta < 0 ? -speed : speed;
}

std::uint64_t travel_ticks(std::int64_t distance_mm, const v1::VehicleState& vehicle,
                           std::uint64_t step_ms) {
    const auto numerator = static_cast<std::uint64_t>(distance_mm) * 1000U;
    const auto denominator = static_cast<std::uint64_t>(vehicle.max_speed_mm_s()) * step_ms;
    return numerator / denominator + (numerator % denominator != 0U);
}

}

class Controller::Impl {
public:
    explicit Impl(std::string agent_id) : agent_id_(std::move(agent_id)), allocator_(agent_id_) {
        factory_.registerSimpleAction("Synchronize", [this](BT::TreeNode&) { return synchronize(); });
        factory_.registerSimpleAction("Allocate", [this](BT::TreeNode&) { return allocate(); });
        factory_.registerSimpleAction("Validate", [this](BT::TreeNode&) { return validate(); });
        factory_.registerSimpleAction("Recover", [this](BT::TreeNode&) { return recover(); });
        factory_.registerSimpleAction("Plan", [this](BT::TreeNode&) { return plan(); });
        factory_.registerSimpleAction("Navigate", [this](BT::TreeNode&) { return navigate(); });
        factory_.registerSimpleAction("Execute", [this](BT::TreeNode&) { return execute(); });
        factory_.registerSimpleAction("Report", [this](BT::TreeNode&) { return report(); });
        tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
            R"(<root BTCPP_format="4"><BehaviorTree ID="Autonomy"><Sequence><Synchronize/><Allocate/><Validate/><Fallback><Recover/><Sequence><Plan/><Fallback><Sequence><Execute/><Report/></Sequence><Navigate/></Fallback></Sequence></Fallback></Sequence></BehaviorTree></root>)"));
    }

    v1::Envelope act(const v1::Envelope& input) {
        if (input.schema_version() != 1 || input.sender_id() != "sim" || input.recipient_id() != agent_id_
            || !input.has_observation() || input.observation().self().id() != agent_id_) {
            throw std::invalid_argument("invalid observation envelope");
        }
        v1::Envelope output;
        output.set_schema_version(1);
        output.set_sequence(input.sequence());
        output.set_simulation_time_ms(input.simulation_time_ms());
        output.set_sender_id(agent_id_);
        output.set_recipient_id("sim");
        output.mutable_action()->set_tick(input.observation().tick());
        input_ = &input;
        output_ = &output;
        tree_->tickOnce();
        output.mutable_action()->set_route_version(route_version_);
        if (replan_reason_ != v1::REPLAN_REASON_UNSPECIFIED) {
            auto* sample = output.mutable_action()->add_replanning_samples();
            sample->set_agent_id(agent_id_);
            sample->set_reason(replan_reason_);
            sample->set_start_tick(input.observation().tick());
            sample->set_end_tick(input.observation().tick());
            sample->set_wait_plan(output.action().behavior_mode() == v1::BEHAVIOR_MODE_WAITING);
            sample->set_complete(true);
            output.mutable_action()->set_replanned(true);
        }
        input_ = nullptr;
        output_ = nullptr;
        observation_ = nullptr;
        task_ = nullptr;
        return output;
    }

private:
    BT::NodeStatus synchronize() {
        observation_ = &input_->observation();
        task_ = nullptr;
        handled_ = false;
        energy_short_ = false;
        allocation_pending_ = false;
        candidate_.reset();
        auto* action = output_->mutable_action();
        action->set_behavior_mode(v1::BEHAVIOR_MODE_IDLE);
        replan_reason_ = v1::REPLAN_REASON_UNSPECIFIED;
        for (const auto& detection : observation_->failure_detections()) {
            const auto key = detection.failed_agent_id() + ":" + std::to_string(detection.detection_tick());
            if (!reported_failures_.contains(key)) {
                action->add_failure_detections()->CopyFrom(detection);
                reported_failures_[key] = true;
            }
        }
        map_changed_ = map_version_ != 0 && map_version_ != observation_->world().map_version();
        const std::vector<std::string> terrain_access(observation_->self().terrain_access().begin(),
                                                      observation_->self().terrain_access().end());
        const auto dynamics_changed = max_speed_mm_s_ != 0
                                      && (max_speed_mm_s_ != observation_->self().max_speed_mm_s()
                                          || energy_capacity_mj_ != observation_->self().energy_capacity_mj()
                                          || energy_cost_mj_per_meter_ != observation_->self().energy_cost_mj_per_meter()
                                           || step_ms_ != observation_->step_ms()
                                           || terrain_access_ != terrain_access);
        // route cache key includes geometry and vehicle dynamics
        if (map_changed_ || dynamics_changed) {
            route_cache_.clear();
        }
        max_speed_mm_s_ = observation_->self().max_speed_mm_s();
        energy_capacity_mj_ = observation_->self().energy_capacity_mj();
        energy_cost_mj_per_meter_ = observation_->self().energy_cost_mj_per_meter();
        step_ms_ = observation_->step_ms();
        terrain_access_ = terrain_access;
        map_version_ = observation_->world().map_version();
        std::vector<const v1::TaskState*> tasks;
        for (const auto& task : observation_->assigned_tasks()) {
            if (task.status() == v1::TASK_STATUS_PENDING && task.assigned_agent_id() == agent_id_) {
                tasks.push_back(&task);
            }
        }
        std::sort(tasks.begin(), tasks.end(), [](const auto* left, const auto* right) {
            return std::tuple{left->allocation_epoch(), left->bundle_position(), left->deadline_tick(), left->id()}
                   < std::tuple{right->allocation_epoch(), right->bundle_position(), right->deadline_tick(),
                                right->id()};
        });
        if (!tasks.empty()) {
            task_ = tasks.front();
        }
        if (route_) {
            auto reason = v1::REPLAN_REASON_UNSPECIFIED;
            if (map_changed_) {
                reason = v1::REPLAN_REASON_BLOCKAGE;
            } else if (dynamics_changed) {
                reason = v1::REPLAN_REASON_ENDURANCE;
            } else if (route_goal_.starts_with("task:")) {
                const auto id = route_goal_.substr(5);
                if (!task_ || task_->id() != id) {
                    const auto known = std::find_if(observation_->known_tasks().begin(),
                                                    observation_->known_tasks().end(), [&id](const auto& current) {
                                                        return current.id() == id;
                                                    });
                    reason = known != observation_->known_tasks().end()
                                     && !known->assigned_agent_id().empty()
                                 ? v1::REPLAN_REASON_OWNER
                                 : v1::REPLAN_REASON_TASK;
                }
            }
            if (reason != v1::REPLAN_REASON_UNSPECIFIED) {
                invalidate(reason, true);
            }
        }
        // A reservation request expires at its proposed start tick.
        if (pending_reservation_ && pending_reservation_->start_tick() <= observation_->tick()) {
            const auto committed = std::any_of(observation_->committed_reservations().begin(),
                                               observation_->committed_reservations().end(),
                                               [this](const auto& current) {
                                                   return current.agent_id() == agent_id_
                                                          && current.version() == pending_reservation_->version();
                                               });
            if (!committed) {
                invalidate(v1::REPLAN_REASON_RESERVATION, false);
            }
            pending_reservation_.reset();
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus allocate() {
        if (observation_->allocation_policy() != v1::ALLOCATION_POLICY_NEAREST_CAPABLE
            && observation_->allocation_policy() != v1::ALLOCATION_POLICY_SENTINEL_CBBA) {
            return BT::NodeStatus::SUCCESS;
        }
        auto result = allocator_.update(*observation_);
        coordinated_ = result.coordinated;
        output_->mutable_action()->mutable_allocation_state()->CopyFrom(result.state);
        for (auto& message : result.outgoing_messages) {
            output_->mutable_action()->add_outgoing_messages()->CopyFrom(message);
        }
        for (auto& commit : result.commits) {
            output_->mutable_action()->add_allocation_commits()->CopyFrom(commit);
        }
        allocation_pending_ = result.pending && task_ == nullptr;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus validate() {
        if (allocation_pending_) {
            handled_ = true;
            output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
            return BT::NodeStatus::SUCCESS;
        }
        if (!observation_->self().active()) {
            clear_route();
            handled_ = true;
            return BT::NodeStatus::SUCCESS;
        }
        if (!task_) {
            return BT::NodeStatus::SUCCESS;
        }
        const auto& self = observation_->self();
        if (task_->status() != v1::TASK_STATUS_PENDING || task_->assigned_agent_id() != agent_id_
            || !capable(self, task_->required_capability())
            || self.payload_grams() < task_->payload_required_grams()
            || observation_->tick() > task_->deadline_tick()) {
            reject();
            return BT::NodeStatus::SUCCESS;
        }
        candidate_ = route_to(coordinate(task_->target().x_mm(), task_->target().y_mm()));
        if (!candidate_) {
            reject();
            return BT::NodeStatus::SUCCESS;
        }
        const auto remaining_service = task_->service_ticks() > task_->progress_ticks()
                                           ? task_->service_ticks() - task_->progress_ticks()
                                           : 0;
        if (candidate_->travel_ticks + remaining_service > task_->deadline_tick() - observation_->tick()) {
            reject();
            return BT::NodeStatus::SUCCESS;
        }
        const auto service_energy = static_cast<std::int64_t>(remaining_service)
                                    * task_->service_energy_mj_per_tick();
        const auto required_energy = candidate_->energy_mj + service_energy + planning::reserve_energy(self);
        energy_short_ = self.energy_mj() < required_energy;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus recover() {
        if (handled_) {
            return BT::NodeStatus::SUCCESS;
        }
        if (!task_) {
            return return_home();
        }
        if (!energy_short_) {
            return BT::NodeStatus::FAILURE;
        }
        const auto* charger = select_charger();
        if (!charger) {
            reject();
            return BT::NodeStatus::SUCCESS;
        }
        const auto current_distance = distance(observation_->self().position().x_mm(),
                                               observation_->self().position().y_mm(),
                                               charger->position().x_mm(), charger->position().y_mm());
        if (current_distance <= charger->radius_mm()) {
            clear_route();
            output_->mutable_action()->set_charge_location_id(charger->id());
            output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_CHARGING);
            return BT::NodeStatus::SUCCESS;
        }
        const auto goal = std::string("charge:") + charger->id();
        if (!route_ || route_goal_ != goal || route_map_version_ != map_version_) {
            auto next = route_to(coordinate(charger->position().x_mm(), charger->position().y_mm()));
            if (!next || next->energy_mj > observation_->self().energy_mj()) {
                reject();
                return BT::NodeStatus::SUCCESS;
            }
            set_route(std::move(*next), goal);
        }
        follow(v1::BEHAVIOR_MODE_NAVIGATING);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus plan() {
        const auto goal = std::string("task:") + task_->id();
        if (!route_ || route_goal_ != goal || route_map_version_ != map_version_) {
            if (!candidate_) {
                reject();
                return BT::NodeStatus::FAILURE;
            }
            set_route(std::move(*candidate_), goal);
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus execute() {
        const auto current_distance = distance(observation_->self().position().x_mm(),
                                               observation_->self().position().y_mm(), task_->target().x_mm(),
                                               task_->target().y_mm());
        if (current_distance > task_->completion_radius_mm()) {
            return BT::NodeStatus::FAILURE;
        }
        output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_EXECUTING);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus report() {
        auto* report = output_->mutable_action()->add_task_reports();
        report->set_task_id(task_->id());
        report->set_kind(v1::TASK_REPORT_KIND_WORKING);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus navigate() {
        if (!follow(v1::BEHAVIOR_MODE_NAVIGATING)) {
            output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus return_home() {
        const auto& id = observation_->self().return_location_id();
        const auto position = std::find_if(observation_->world().locations().begin(),
                                           observation_->world().locations().end(), [&id](const auto& location) {
                                               return location.id() == id
                                                      && location.kind() == v1::LOCATION_KIND_RETURN;
                                           });
        if (id.empty() || position == observation_->world().locations().end()) {
            clear_route();
            output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_IDLE);
            return BT::NodeStatus::SUCCESS;
        }
        if (distance(observation_->self().position().x_mm(), observation_->self().position().y_mm(),
                     position->position().x_mm(), position->position().y_mm()) <= position->radius_mm()) {
            clear_route();
            output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_IDLE);
            return BT::NodeStatus::SUCCESS;
        }
        const auto goal = std::string("return:") + id;
        if (!route_ || route_goal_ != goal || route_map_version_ != map_version_) {
            auto next = route_to(coordinate(position->position().x_mm(), position->position().y_mm()));
            if (!next) {
                clear_route();
                output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
                return BT::NodeStatus::SUCCESS;
            }
            set_route(std::move(*next), goal);
        }
        if (!follow(v1::BEHAVIOR_MODE_RETURNING)) {
            output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
        }
        return BT::NodeStatus::SUCCESS;
    }

    const v1::ServiceLocation* select_charger() {
        const v1::ServiceLocation* selected = nullptr;
        std::optional<Route> selected_route;
        const auto remaining_service = task_->service_ticks() > task_->progress_ticks()
                                           ? task_->service_ticks() - task_->progress_ticks()
                                           : 0;
        const auto remaining_ticks = task_->deadline_tick() - observation_->tick();
        for (const auto& location : observation_->world().locations()) {
            if (location.kind() != v1::LOCATION_KIND_CHARGING || location.charge_mj_per_tick() <= 0) {
                continue;
            }
            auto route = route_to(coordinate(location.position().x_mm(), location.position().y_mm()));
            if (!route || route->energy_mj > observation_->self().energy_mj()) {
                continue;
            }
            auto charged = observation_->self();
            charged.mutable_position()->CopyFrom(location.position());
            charged.set_energy_mj(charged.energy_capacity_mj());
            const auto& task_route = aligned_route(
                coordinate(location.position().x_mm(), location.position().y_mm()),
                coordinate(task_->target().x_mm(), task_->target().y_mm()));
            if (!task_route) {
                continue;
            }
            const auto task_energy = task_route->energy_mj + planning::reserve_energy(charged)
                                     + static_cast<std::int64_t>(remaining_service)
                                           * task_->service_energy_mj_per_tick();
            if (task_energy > charged.energy_capacity_mj()) {
                continue;
            }
            const auto arrival_energy = observation_->self().energy_mj() - route->energy_mj;
            const auto charge_needed = std::max<std::int64_t>(0, task_energy - arrival_energy);
            const auto charge_ticks = static_cast<std::uint64_t>(
                charge_needed / location.charge_mj_per_tick()
                + (charge_needed % location.charge_mj_per_tick() != 0));
            if (route->travel_ticks + charge_ticks + task_route->travel_ticks + remaining_service
                > remaining_ticks) {
                continue;
            }
            if (!selected || std::tuple{route->distance_mm, location.id()}
                                 < std::tuple{selected_route->distance_mm, selected->id()}) {
                selected = &location;
                selected_route = std::move(route);
            }
        }
        return selected;
    }

    const std::optional<Route>& aligned_route(const Coordinate& start, const Coordinate& goal) {
        const auto key = std::tuple{start.x(), start.y(), goal.x(), goal.y()};
        const auto found = route_cache_.find(key);
        if (found != route_cache_.end()) {
            return found->second;
        }
        return route_cache_.emplace(
            key, planning::astar(observation_->world(), observation_->self(), start, goal,
                                 observation_->step_ms())).first->second;
    }

    std::optional<Route> route_to(const Coordinate& goal) {
        const auto& self = observation_->self();
        const auto start = coordinate(self.position().x_mm(), self.position().y_mm());
        const auto cell = observation_->world().grid_cell_mm();
        if (cell <= 0) {
            return std::nullopt;
        }
        if (start.x() % cell == 0 && start.y() % cell == 0) {
            return aligned_route(start, goal);
        }
        // Try each reachable grid anchor.
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
            if (!planning::segment_allowed(observation_->world(), self, start, anchor)) {
                continue;
            }
            const auto& planned = aligned_route(anchor, goal);
            if (!planned) {
                continue;
            }
            auto route = *planned;
            const auto partial = std::abs(start.x() - anchor.x()) + std::abs(start.y() - anchor.y());
            route.distance_mm += partial;
            route.energy_mj += planning::motion_energy(
                self, partial, planning::terrain_multiplier(observation_->world(), start, anchor));
            route.travel_ticks += travel_ticks(partial, self, observation_->step_ms());
            route.points.insert(route.points.begin(), start);
            if (!best || std::tuple{route.distance_mm, anchor.x(), anchor.y()}
                             < std::tuple{best->distance_mm, best_anchor.x(), best_anchor.y()}) {
                best = std::move(route);
                best_anchor = anchor;
            }
        }
        return best;
    }

    void set_route(Route route, std::string goal) {
        const auto replacing = route_.has_value();
        route_ = std::move(route);
        route_goal_ = std::move(goal);
        route_map_version_ = map_version_;
        waypoint_ = 0;
        ++route_version_;
        auto* plan = output_->mutable_action()->mutable_route_plan();
        plan->set_version(route_version_);
        plan->set_map_version(map_version_);
        plan->set_goal(route_goal_);
        for (const auto& point : route_->points) {
            auto* waypoint = plan->add_waypoints();
            waypoint->set_x_mm(point.x());
            waypoint->set_y_mm(point.y());
        }
        if (replacing || map_changed_) {
            output_->mutable_action()->set_replanned(true);
        }
    }

    bool follow(v1::BehaviorMode mode) {
        if (!route_) {
            return false;
        }
        const auto current = coordinate(observation_->self().position().x_mm(),
                                        observation_->self().position().y_mm());
        while (waypoint_ < route_->points.size() && route_->points[waypoint_] == current) {
            ++waypoint_;
        }
        if (waypoint_ >= route_->points.size()) {
            return false;
        }
        const auto& next = route_->points[waypoint_];
        if (!reserve(current, next)) {
            output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
            return false;
        }
        const auto delta_x = next.x() - current.x();
        const auto delta_y = next.y() - current.y();
        auto* action = output_->mutable_action();
        if (delta_x != 0) {
            action->set_velocity_x_mm_s(velocity(delta_x, observation_->self().max_speed_mm_s(),
                                                 observation_->step_ms()));
        } else {
            action->set_velocity_y_mm_s(velocity(delta_y, observation_->self().max_speed_mm_s(),
                                                 observation_->step_ms()));
        }
        action->set_behavior_mode(mode);
        return true;
    }

    std::uint64_t passage_ticks(std::string_view resource, Coordinate current) const {
        std::uint64_t result{};
        const auto denominator = static_cast<std::uint64_t>(observation_->self().max_speed_mm_s())
                                 * observation_->step_ms();
        for (auto index = waypoint_; index < route_->points.size(); ++index) {
            const auto& next = route_->points[index];
            const auto contested = planning::contested_resource(observation_->world(), current, next);
            if (!contested || *contested != resource) {
                break;
            }
            const auto length = static_cast<std::uint64_t>(std::abs(next.x() - current.x())
                                                           + std::abs(next.y() - current.y()));
            const auto numerator = length * 1000;
            result += numerator / denominator + (numerator % denominator != 0);
            current = next;
        }
        return std::max<std::uint64_t>(1, result);
    }

    bool coordinated() const {
        if (observation_->allocation_policy() != v1::ALLOCATION_POLICY_NEAREST_CAPABLE
            && observation_->allocation_policy() != v1::ALLOCATION_POLICY_SENTINEL_CBBA) {
            return true;
        }
        return coordinated_;
    }

    bool reserve(const Coordinate& current, const Coordinate& next) {
        const auto resource = planning::contested_resource(observation_->world(), current, next);
        if (!resource) {
            return true;
        }
        const auto valid = [this, &resource](const auto& value) {
            return value.resource_id() == *resource && value.agent_id() == agent_id_
                   && value.route_version() == route_version_ && value.map_version() == map_version_;
        };
        const auto grant = std::find_if(observation_->committed_reservations().begin(),
                                        observation_->committed_reservations().end(), [&](const auto& value) {
                                            return valid(value) && value.start_tick() <= observation_->tick()
                                                   && observation_->tick() <= value.end_tick();
                                        });
        if (grant != observation_->committed_reservations().end()) {
            return true;
        }
        if (!coordinated()) {
            return false;
        }
        const auto future = std::find_if(observation_->committed_reservations().begin(),
                                         observation_->committed_reservations().end(), [&](const auto& value) {
                                             return valid(value) && value.start_tick() > observation_->tick();
                                         });
        if (future != observation_->committed_reservations().end() || pending_reservation_) {
            return false;
        }
        // next slot after the committed queue
        auto start = observation_->tick() + 1;
        for (const auto& value : observation_->committed_reservations()) {
            if (value.resource_id() == *resource && value.end_tick() >= start) {
                start = value.end_tick() + 1;
            }
        }
        auto* proposal = output_->mutable_action()->add_reservation_proposals();
        proposal->set_resource_id(*resource);
        proposal->set_agent_id(agent_id_);
        proposal->set_start_tick(start);
        proposal->set_end_tick(start + passage_ticks(*resource, current) - 1);
        proposal->set_version(++reservation_version_);
        proposal->set_route_version(route_version_);
        proposal->set_map_version(map_version_);
        pending_reservation_ = *proposal;
        return false;
    }

    void reject() {
        handled_ = true;
        clear_route();
        output_->mutable_action()->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
        if (!task_) {
            return;
        }
        const auto key = task_->id() + ":" + std::to_string(task_->allocation_version()) + ":"
                         + std::to_string(map_version_);
        if (rejections_.contains(key)) {
            return;
        }
        auto* report = output_->mutable_action()->add_task_reports();
        report->set_task_id(task_->id());
        report->set_kind(v1::TASK_REPORT_KIND_REJECTED);
        rejections_[key] = true;
    }

    void invalidate(v1::ReplanReason reason, bool clear) {
        if (replan_reason_ == v1::REPLAN_REASON_UNSPECIFIED) {
            replan_reason_ = reason;
        }
        if (clear) {
            clear_route();
        }
    }

    void clear_route() {
        if (route_) {
            ++route_version_;
        }
        route_.reset();
        route_goal_.clear();
        waypoint_ = 0;
        pending_reservation_.reset();
    }

    std::string agent_id_;
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> tree_;
    const v1::Envelope* input_{};
    v1::Envelope* output_{};
    const v1::AgentObservation* observation_{};
    const v1::TaskState* task_{};
    Allocator allocator_;
    std::map<std::string, bool> rejections_;
    std::map<std::string, bool> reported_failures_;
    std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t, std::int64_t>,
             std::optional<Route>> route_cache_;
    std::optional<Route> route_;
    std::optional<Route> candidate_;
    std::optional<v1::SpaceTimeReservation> pending_reservation_;
    std::string route_goal_;
    std::size_t waypoint_{};
    std::uint64_t route_version_{};
    std::uint64_t route_map_version_{};
    std::uint64_t map_version_{};
    std::uint64_t reservation_version_{};
    std::int64_t max_speed_mm_s_{};
    std::int64_t energy_capacity_mj_{};
    std::int64_t energy_cost_mj_per_meter_{};
    std::uint64_t step_ms_{};
    std::vector<std::string> terrain_access_;
    v1::ReplanReason replan_reason_{v1::REPLAN_REASON_UNSPECIFIED};
    bool map_changed_{};
    bool handled_{};
    bool energy_short_{};
    bool allocation_pending_{};
    bool coordinated_{};
};

Controller::Controller(std::string agent_id) : impl_(std::make_unique<Impl>(std::move(agent_id))) {}

Controller::~Controller() = default;

v1::Envelope Controller::act(const v1::Envelope& input) {
    return impl_->act(input);
}

}
