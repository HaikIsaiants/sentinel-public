#include <sentinel/core/simulator.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/core/scenario.hpp>
#include <sentinel/planning/planner.hpp>
#include <sentinel/protocol/framing.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <map>
#include <stdexcept>
#include <tuple>

namespace sentinel::core {
namespace {

std::int64_t axis_velocity(std::int64_t value, std::int64_t limit) {
    return std::clamp(value, -limit, limit);
}

bool valid_behavior(sentinel::v1::BehaviorMode value) {
    return value >= sentinel::v1::BEHAVIOR_MODE_UNSPECIFIED && value <= sentinel::v1::BEHAVIOR_MODE_RETURNING;
}

bool valid_report(sentinel::v1::TaskReportKind value) {
    return value == sentinel::v1::TASK_REPORT_KIND_WORKING || value == sentinel::v1::TASK_REPORT_KIND_REJECTED;
}

bool valid_replan_reason(sentinel::v1::ReplanReason value) {
    return value >= sentinel::v1::REPLAN_REASON_BLOCKAGE && value <= sentinel::v1::REPLAN_REASON_OWNER;
}

planning::Reservation reservation(const sentinel::v1::SpaceTimeReservation& value) {
    return {value.resource_id(), value.agent_id(), value.start_tick(), value.end_tick(), value.version(),
            value.route_version(), value.map_version()};
}

void copy_reservation(const planning::Reservation& source, sentinel::v1::SpaceTimeReservation* target) {
    target->set_resource_id(source.resource_id);
    target->set_agent_id(source.agent_id);
    target->set_start_tick(source.start_tick);
    target->set_end_tick(source.end_tick);
    target->set_version(source.version);
    target->set_route_version(source.route_version);
    target->set_map_version(source.map_version);
}

bool inside(const sentinel::v1::Point& point, const sentinel::v1::ServiceLocation& location) {
    return std::abs(point.x_mm() - location.position().x_mm())
               + std::abs(point.y_mm() - location.position().y_mm())
           <= location.radius_mm();
}

}

Simulator::Simulator(sentinel::v1::Scenario scenario)
    : scenario_(std::move(scenario)), rng_streams_(scenario_.seed()),
      network_(scenario_.seed(), scenario_.step_ms()), network_profile_(scenario_.network_profile()) {
    normalize_scenario(scenario_);
    validate_scenario(scenario_);
    vehicles_.reserve(scenario_.vehicles_size());
    for (const auto& spec : scenario_.vehicles()) {
        Vehicle current;
        current.spec = spec;
        current.x_mm = spec.initial_position().x_mm();
        current.y_mm = spec.initial_position().y_mm();
        current.energy_mj = spec.initial_energy_mj();
        current.energy_capacity_mj = spec.initial_energy_mj();
        current.max_speed_mm_s = spec.max_speed_mm_s();
        current.payload_grams = spec.payload_grams();
        record_energy_invariant(current);
        vehicles_.push_back(std::move(current));
    }
    tasks_.reserve(scenario_.tasks_size());
    for (const auto& spec : scenario_.tasks()) {
        Task current;
        current.spec = spec;
        current.released = spec.released();
        current.assigned_agent_id = spec.assigned_agent_id();
        current.allocation_epoch = current.assigned_agent_id.empty() ? 0 : allocation_epoch_;
        current.allocation_version = current.assigned_agent_id.empty() ? 0 : 1;
        tasks_.push_back(std::move(current));
    }
    allocation_views_.resize(vehicles_.size());
    network_.set_profile(network_profile(network_profile_));
    if (scenario_.allocation_policy() != sentinel::v1::ALLOCATION_POLICY_SCRIPTED) {
        begin_convergence();
    }
    state_hash_ = compute_state_hash();
}

sentinel::v1::VehicleState Simulator::vehicle_state(const Vehicle& current) const {
    sentinel::v1::VehicleState state;
    state.set_id(current.spec.id());
    state.set_kind(current.spec.kind());
    state.mutable_position()->set_x_mm(current.x_mm);
    state.mutable_position()->set_y_mm(current.y_mm);
    state.set_velocity_x_mm_s(current.velocity_x_mm_s);
    state.set_velocity_y_mm_s(current.velocity_y_mm_s);
    state.set_max_speed_mm_s(current.max_speed_mm_s);
    state.set_energy_mj(current.energy_mj);
    state.set_payload_grams(current.payload_grams);
    state.set_active(current.active);
    state.set_initial_energy_mj(current.spec.initial_energy_mj());
    state.set_energy_cost_mj_per_meter(current.spec.energy_cost_mj_per_meter());
    state.set_return_location_id(current.spec.return_location_id());
    state.set_energy_capacity_mj(current.energy_capacity_mj);
    for (const auto capability : current.spec.capabilities()) {
        state.add_capabilities(static_cast<sentinel::v1::Capability>(capability));
    }
    for (const auto& terrain : current.spec.terrain_access()) {
        state.add_terrain_access(terrain);
    }
    return state;
}

sentinel::v1::TaskState Simulator::task_state(const Task& current) const {
    sentinel::v1::TaskState state;
    state.set_id(current.spec.id());
    state.set_kind(current.spec.kind());
    state.mutable_target()->CopyFrom(current.spec.target());
    state.set_required_capability(current.spec.required_capability());
    state.set_payload_required_grams(current.spec.payload_required_grams());
    state.set_deadline_tick(current.spec.deadline_tick());
    state.set_completion_radius_mm(current.spec.completion_radius_mm());
    state.set_assigned_agent_id(current.assigned_agent_id);
    state.set_status(current.status);
    state.set_service_ticks(current.spec.service_ticks());
    state.set_service_energy_mj_per_tick(current.spec.service_energy_mj_per_tick());
    state.set_progress_ticks(current.progress_ticks);
    state.set_allocation_epoch(current.allocation_epoch);
    state.set_allocation_version(current.allocation_version);
    state.set_priority(current.spec.priority());
    state.set_allocation_score(current.allocation_score);
    state.set_bundle_position(current.bundle_position);
    return state;
}

sentinel::v1::ObservationBatch Simulator::observe() const {
    sentinel::v1::ObservationBatch batch;
    batch.set_tick(tick_);
    batch.set_finished(finished());
    for (std::size_t index = 0; index < vehicles_.size(); ++index) {
        const auto& current = vehicles_[index];
        auto* envelope = batch.add_observations();
        envelope->set_schema_version(1);
        envelope->set_sequence(tick_ * vehicles_.size() + index + 1);
        envelope->set_simulation_time_ms(tick_ * scenario_.step_ms());
        envelope->set_sender_id("sim");
        envelope->set_recipient_id(current.spec.id());
        auto* observation = envelope->mutable_observation();
        observation->set_tick(tick_);
        observation->set_step_ms(scenario_.step_ms());
        observation->mutable_self()->CopyFrom(vehicle_state(current));
        observation->mutable_world()->CopyFrom(scenario_.world());
        observation->set_allocation_epoch(allocation_epoch_);
        observation->set_allocation_policy(scenario_.allocation_policy());
        observation->set_network_profile(network_profile_);
        const auto detected = [this, &current](std::string_view agent_id) {
            return std::any_of(failure_monitors_.begin(), failure_monitors_.end(),
                               [&current, agent_id](const auto& monitor) {
                                   return monitor.detector_id == current.spec.id()
                                          && monitor.failed_id == agent_id
                                          && monitor.detection_tick.has_value();
                               });
        };
        for (const auto& peer : vehicles_) {
            observation->add_peer_ids(peer.spec.id());
            if (peer.spec.id() == current.spec.id()
                || (!network_.link_blocked(current.spec.id(), peer.spec.id()) && !detected(peer.spec.id()))) {
                observation->add_reachable_peer_ids(peer.spec.id());
            }
        }
        for (const auto& current_task : tasks_) {
            if (!current_task.released || current_task.status != sentinel::v1::TASK_STATUS_PENDING) {
                continue;
            }
            auto projected = task_state(current_task);
            if (current_task.assigned_agent_id != current.spec.id()) {
                projected.clear_progress_ticks();
            }
            if (scenario_.allocation_policy() != sentinel::v1::ALLOCATION_POLICY_SCRIPTED
                && current_task.assigned_agent_id != current.spec.id()) {
                projected.clear_assigned_agent_id();
                projected.clear_allocation_epoch();
                projected.clear_allocation_version();
                projected.clear_allocation_score();
                projected.clear_bundle_position();
                const auto& owners = allocation_views_[index].owners();
                const auto owner = std::lower_bound(owners.begin(), owners.end(), current_task.spec.id(),
                                                    [](const auto& value, std::string_view id) {
                                                        return value.task_id() < id;
                                                    });
                if (owner != owners.end() && owner->task_id() == current_task.spec.id()
                    && owner->agent_id() != current.spec.id() && !detected(owner->agent_id())) {
                    projected.set_assigned_agent_id(owner->agent_id());
                    projected.set_allocation_epoch(owner->epoch());
                    projected.set_allocation_version(owner->version());
                    projected.set_allocation_score(owner->score());
                    projected.set_bundle_position(owner->bundle_position());
                }
            }
            observation->add_known_tasks()->CopyFrom(projected);
            if (current_task.assigned_agent_id == current.spec.id()) {
                observation->add_assigned_tasks()->CopyFrom(task_state(current_task));
            } else if (projected.assigned_agent_id().empty()) {
                observation->add_available_tasks()->CopyFrom(projected);
            }
        }
        for (const auto& message : delivered_messages_) {
            if (message.recipient_id() == current.spec.id()) {
                observation->add_delivered_messages()->CopyFrom(message);
            }
        }
        if (current.active) {
            for (const auto& monitor : failure_monitors_) {
                if (monitor.detector_id != current.spec.id() || !monitor.detection_tick) {
                    continue;
                }
                auto* detection = observation->add_failure_detections();
                detection->set_failed_agent_id(monitor.failed_id);
                detection->set_detector_agent_id(monitor.detector_id);
                detection->set_failure_tick(monitor.failure_tick);
                detection->set_detection_tick(*monitor.detection_tick);
            }
        }
        for (const auto& committed : reservations_.committed()) {
            if (committed.agent_id == current.spec.id()) {
                copy_reservation(committed, observation->add_committed_reservations());
            }
        }
    }
    if (batch.finished()) {
        batch.mutable_summary()->CopyFrom(summary());
    }
    return batch;
}

StepOutcome Simulator::step(const sentinel::v1::ActionBatch& actions) {
    return step_internal(actions, nullptr);
}

StepOutcome Simulator::replay_step(
    const sentinel::v1::ActionBatch& actions,
    const google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent>& applied_events) {
    return step_internal(actions, &applied_events);
}

StepOutcome Simulator::step_internal(
    const sentinel::v1::ActionBatch& actions,
    const google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent>* recorded_events) {
    if (finished()) {
        throw std::logic_error("simulation has already finished");
    }
    StepOutcome outcome;
    outcome.actions = canonicalize_actions(actions);
    outcome.applied_events = apply_events(recorded_events, outcome.network_outcomes);
    // link changes fence this tick's coordination traffic
    const auto coordination_fence = std::any_of(outcome.applied_events.begin(), outcome.applied_events.end(),
                                                 [](const auto& event) {
                                                     return event.source().kind()
                                                            == sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED;
                                                 });
    apply_actions(outcome.actions, outcome.network_outcomes, coordination_fence);
    update_failure_monitors();
    std::sort(outcome.network_outcomes.begin(), outcome.network_outcomes.end(), [](const auto& left, const auto& right) {
        return std::tuple{left.tick(), left.sequence(), left.disposition()}
               < std::tuple{right.tick(), right.sequence(), right.disposition()};
    });
    network_outcomes_ = outcome.network_outcomes;
    // Progress and deadlines use the post-action tick.
    ++tick_;
    update_tasks(outcome.actions);
    update_convergence();
    state_hash_ = compute_state_hash();
    tick_hashes_.push_back(state_hash_);
    outcome.observations = observe();
    return outcome;
}

sentinel::v1::SimulationSummary Simulator::summary() const {
    sentinel::v1::SimulationSummary result;
    result.set_ticks(tick_);
    result.set_terminal_hash(state_hash());
    std::uint32_t completed{};
    for (const auto& current : tasks_) {
        completed += current.status == sentinel::v1::TASK_STATUS_COMPLETED ? 1U : 0U;
    }
    std::uint32_t active{};
    for (const auto& current : vehicles_) {
        active += current.active ? 1U : 0U;
    }
    result.set_completed_tasks(completed);
    result.set_total_tasks(static_cast<std::uint32_t>(tasks_.size()));
    result.set_active_agents(active);
    result.set_success(completed == tasks_.size());
    result.set_wait_ticks(wait_ticks_);
    result.set_replan_count(replan_count_);
    result.set_recharge_ticks(recharge_ticks_);
    result.set_return_ticks(return_ticks_);
    result.set_route_conflicts(reservations_.conflicts());
    result.set_travel_distance_mm(travel_distance_mm_);
    result.set_energy_consumed_mj(energy_consumed_mj_);
    result.set_rejected_commits(rejected_commits_);
    result.set_communication_bytes(network_.communication_bytes());
    result.set_communication_messages(network_.communication_messages());
    result.set_delivered_messages(network_.delivered_messages());
    result.set_dropped_messages(network_.dropped_messages());
    result.set_reordered_messages(network_.reordered_messages());
    for (const auto& sample : allocation_convergence_) {
        result.add_allocation_convergence()->CopyFrom(sample);
    }
    for (const auto& detection : failure_detections_) {
        result.add_failure_detections()->CopyFrom(detection);
    }
    std::uint64_t maximum_delay{};
    std::uint32_t missing{};
    for (const auto& sample : task_reassignments_) {
        result.add_task_reassignments()->CopyFrom(sample);
        if (sample.complete()) {
            maximum_delay = std::max(maximum_delay,
                                     (sample.commit_tick() - sample.detection_tick()) * scenario_.step_ms());
        } else {
            ++missing;
        }
    }
    result.set_maximum_reassignment_delay_ms(maximum_delay);
    result.set_missing_reassignments(missing);
    result.set_agent_energy_never_drops_below_zero(agent_energy_below_zero_violations_ == 0);
    result.set_agent_energy_below_zero_violations(agent_energy_below_zero_violations_);
    result.set_committed_reservations_never_overlap(committed_reservation_overlap_violations_ == 0);
    result.set_committed_reservation_overlap_violations(committed_reservation_overlap_violations_);
    result.set_completed_tasks_are_never_reassigned(completed_task_reassignment_violations_ == 0);
    result.set_completed_task_reassignment_violations(completed_task_reassignment_violations_);
    result.set_incapable_agents_never_commit_tasks(incapable_agent_commit_violations_ == 0);
    result.set_incapable_agent_commit_violations(incapable_agent_commit_violations_);
    for (const auto& sample : replanning_samples_) {
        result.add_replanning_samples()->CopyFrom(sample);
    }
    if (convergence_open_) {
        auto* sample = result.add_allocation_convergence();
        sample->set_epoch(allocation_epoch_);
        sample->set_start_tick(convergence_start_tick_);
        sample->set_end_tick(tick_);
    }
    for (const auto& hash : tick_hashes_) {
        result.add_tick_hashes(hash);
    }
    return result;
}

std::string Simulator::state_hash() const {
    return state_hash_;
}

std::string Simulator::compute_state_hash() const {
    HashBuilder hash;
    hash.unsigned_integer(tick_);
    hash.text(network_profile_);
    hash.unsigned_integer(allocation_epoch_);
    hash.unsigned_integer(scenario_.world().map_version());
    for (const auto& region : scenario_.world().regions()) {
        hash.text(region.id());
        hash.boolean(region.closed());
    }
    for (const auto& current : vehicles_) {
        hash.text(current.spec.id());
        hash.text(current.spec.kind());
        hash.signed_integer(current.x_mm);
        hash.signed_integer(current.y_mm);
        hash.signed_integer(current.velocity_x_mm_s);
        hash.signed_integer(current.velocity_y_mm_s);
        hash.signed_integer(current.energy_mj);
        hash.signed_integer(current.energy_capacity_mj);
        hash.signed_integer(current.max_speed_mm_s);
        hash.signed_integer(current.payload_grams);
        hash.boolean(current.active);
        hash.boolean(current.failure_tick.has_value());
        hash.unsigned_integer(current.failure_tick.value_or(0));
        hash.boolean(current.failure_announced);
        hash.unsigned_integer(static_cast<std::uint64_t>(current.behavior_mode));
        hash.unsigned_integer(current.route_version);
    }
    for (const auto& current : tasks_) {
        hash.text(current.spec.id());
        hash.boolean(current.released);
        hash.unsigned_integer(static_cast<std::uint64_t>(current.status));
        hash.text(current.assigned_agent_id);
        hash.unsigned_integer(current.allocation_epoch);
        hash.unsigned_integer(current.allocation_version);
        hash.signed_integer(current.allocation_score);
        hash.unsigned_integer(current.bundle_position);
        hash.unsigned_integer(current.progress_ticks);
        hash.unsigned_integer(current.completion_tick);
    }
    hash.unsigned_integer(wait_ticks_);
    hash.unsigned_integer(replan_count_);
    hash.unsigned_integer(recharge_ticks_);
    hash.unsigned_integer(return_ticks_);
    hash.unsigned_integer(travel_distance_mm_);
    hash.unsigned_integer(energy_consumed_mj_);
    hash.unsigned_integer(rejected_commits_);
    hash.unsigned_integer(reservations_.conflicts());
    hash.unsigned_integer(reservations_.committed().size());
    for (const auto& current : reservations_.committed()) {
        hash.text(current.resource_id);
        hash.text(current.agent_id);
        hash.unsigned_integer(current.start_tick);
        hash.unsigned_integer(current.end_tick);
        hash.unsigned_integer(current.version);
        hash.unsigned_integer(current.route_version);
        hash.unsigned_integer(current.map_version);
    }
    hash.unsigned_integer(reservations_.seen().size());
    for (const auto& current : reservations_.seen()) {
        hash.text(current.resource_id);
        hash.text(current.agent_id);
        hash.unsigned_integer(current.start_tick);
        hash.unsigned_integer(current.end_tick);
        hash.unsigned_integer(current.version);
        hash.unsigned_integer(current.route_version);
        hash.unsigned_integer(current.map_version);
    }
    network_.append_hash(hash);
    hash.boolean(convergence_open_);
    hash.unsigned_integer(convergence_start_tick_);
    hash.unsigned_integer(convergence_candidate_tick_);
    hash.unsigned_integer(convergence_stable_ticks_);
    hash.text(convergence_signature_);
    hash.unsigned_integer(allocation_views_.size());
    for (const auto& view : allocation_views_) {
        hash.text(protocol::deterministic_bytes(view));
    }
    hash.unsigned_integer(allocation_convergence_.size());
    for (const auto& sample : allocation_convergence_) {
        hash.unsigned_integer(sample.epoch());
        hash.unsigned_integer(sample.start_tick());
        hash.unsigned_integer(sample.end_tick());
        hash.boolean(sample.complete());
    }
    hash.unsigned_integer(failure_monitors_.size());
    for (const auto& monitor : failure_monitors_) {
        hash.text(monitor.detector_id);
        hash.text(monitor.failed_id);
        hash.unsigned_integer(monitor.failure_tick);
        hash.unsigned_integer(monitor.missed_ticks);
        hash.boolean(monitor.detection_tick.has_value());
        hash.unsigned_integer(monitor.detection_tick.value_or(0));
    }
    hash.unsigned_integer(failure_detections_.size());
    for (const auto& detection : failure_detections_) {
        hash.text(protocol::deterministic_bytes(detection));
    }
    hash.unsigned_integer(task_reassignments_.size());
    for (const auto& sample : task_reassignments_) {
        hash.text(protocol::deterministic_bytes(sample));
    }
    hash.unsigned_integer(replanning_samples_.size());
    for (const auto& sample : replanning_samples_) {
        hash.text(protocol::deterministic_bytes(sample));
    }
    for (const auto& [name, state] : rng_streams_.states()) {
        hash.text(name);
        hash.unsigned_integer(state);
    }
    for (const auto& message : delivered_messages_) {
        hash.text(message.sender_id());
        hash.text(message.recipient_id());
        hash.unsigned_integer(message.version());
        hash.text(message.payload());
    }
    hash.unsigned_integer(network_outcomes_.size());
    for (const auto& outcome : network_outcomes_) {
        hash.text(protocol::deterministic_bytes(outcome));
    }
    return hash.finish();
}

bool Simulator::finished() const {
    if (tick_ >= scenario_.max_ticks()) {
        return true;
    }
    const bool any_active = std::any_of(vehicles_.begin(), vehicles_.end(), [](const auto& current) {
        return current.active;
    });
    const bool all_terminal = std::all_of(tasks_.begin(), tasks_.end(), [](const auto& current) {
        return current.status != sentinel::v1::TASK_STATUS_PENDING;
    });
    return !any_active || all_terminal;
}

std::uint64_t Simulator::tick() const {
    return tick_;
}

const sentinel::v1::Scenario& Simulator::scenario() const {
    return scenario_;
}

sentinel::v1::ActionBatch Simulator::canonicalize_actions(const sentinel::v1::ActionBatch& actions) const {
    if (actions.tick() != tick_ || actions.actions_size() != static_cast<int>(vehicles_.size())) {
        throw std::invalid_argument("invalid action batch");
    }
    auto result = actions;
    std::sort(result.mutable_actions()->begin(), result.mutable_actions()->end(), [](const auto& left, const auto& right) {
        return left.sender_id() < right.sender_id();
    });
    for (int index = 0; index < result.actions_size(); ++index) {
        auto* envelope = result.mutable_actions(index);
        if (envelope->schema_version() != 1 || !envelope->has_action()
            || envelope->sender_id() != vehicles_[static_cast<std::size_t>(index)].spec.id()
            || envelope->recipient_id() != "sim" || envelope->simulation_time_ms() != tick_ * scenario_.step_ms()
            || envelope->sequence() != tick_ * vehicles_.size() + static_cast<std::size_t>(index) + 1
            || envelope->action().tick() != tick_ || !valid_behavior(envelope->action().behavior_mode())) {
            throw std::invalid_argument("invalid agent action envelope");
        }
        auto* action = envelope->mutable_action();
        if (action->has_allocation_state()) {
            auto* state = action->mutable_allocation_state();
            std::sort(state->mutable_bids()->begin(), state->mutable_bids()->end(), [](const auto& left, const auto& right) {
                return std::tuple{left.task_id(), left.bidder_id()} < std::tuple{right.task_id(), right.bidder_id()};
            });
            std::sort(state->mutable_winners()->begin(), state->mutable_winners()->end(), [](const auto& left, const auto& right) {
                return std::tuple{left.task_id(), left.bidder_id()} < std::tuple{right.task_id(), right.bidder_id()};
            });
            std::sort(state->mutable_winner_relays()->begin(), state->mutable_winner_relays()->end(),
                      [](const auto& left, const auto& right) {
                          return std::tuple{left.bid().task_id(), left.bid().bidder_id()}
                                 < std::tuple{right.bid().task_id(), right.bid().bidder_id()};
                      });
            std::sort(state->mutable_owners()->begin(), state->mutable_owners()->end(), [](const auto& left, const auto& right) {
                return left.task_id() < right.task_id();
            });
            if (scenario_.allocation_policy() == sentinel::v1::ALLOCATION_POLICY_SCRIPTED
                || state->sender_id() != envelope->sender_id() || state->epoch() != allocation_epoch_
                || state->version() == 0 || state->map_version() != scenario_.world().map_version()) {
                throw std::invalid_argument("invalid allocation state");
            }
            const auto valid_bid = [this, state](const auto& bid) {
                if (bid.epoch() != state->epoch() || bid.version() == 0 || bid.task_id().empty()
                    || bid.bidder_id().empty() || bid.distance_mm() < 0 || bid.energy_mj() < 0
                    || bid.completion_tick() > scenario_.max_ticks()
                    || bid.bundle_position() >= static_cast<std::uint32_t>(tasks_.size())) {
                    return false;
                }
                task(bid.task_id());
                vehicle(bid.bidder_id());
                return true;
            };
            if (!std::all_of(state->bids().begin(), state->bids().end(), [&](const auto& bid) {
                    return valid_bid(bid) && bid.bidder_id() == envelope->sender_id();
                })
                || !std::all_of(state->winners().begin(), state->winners().end(), valid_bid)
                || state->winner_relays_size() != state->winners_size()
                || std::adjacent_find(state->bids().begin(), state->bids().end(), [](const auto& left, const auto& right) {
                       return left.task_id() == right.task_id();
                   }) != state->bids().end()
                || std::adjacent_find(state->winners().begin(), state->winners().end(), [](const auto& left, const auto& right) {
                       return left.task_id() == right.task_id();
                   }) != state->winners().end()) {
                throw std::invalid_argument("invalid allocation bids");
            }
            for (int index = 0; index < state->winner_relays_size(); ++index) {
                const auto& relay = state->winner_relays(index);
                if (protocol::deterministic_bytes(relay.bid())
                        != protocol::deterministic_bytes(state->winners(index))
                    || relay.path_agent_ids().empty()
                    || relay.path_agent_ids(0) != relay.bid().bidder_id()
                    || relay.path_agent_ids(relay.path_agent_ids_size() - 1) != state->sender_id()) {
                    throw std::invalid_argument("invalid allocation relay");
                }
                std::vector<std::string> path(relay.path_agent_ids().begin(), relay.path_agent_ids().end());
                auto unique = path;
                std::sort(unique.begin(), unique.end());
                if (std::adjacent_find(unique.begin(), unique.end()) != unique.end()
                    || (relay.bid().bidder_id() == state->sender_id() && path.size() != 1)) {
                    throw std::invalid_argument("invalid allocation relay");
                }
                for (const auto& agent : path) {
                    vehicle(agent);
                }
            }
            if (std::adjacent_find(state->owners().begin(), state->owners().end(),
                                   [](const auto& left, const auto& right) {
                                       return left.task_id() == right.task_id();
                                   }) != state->owners().end()) {
                throw std::invalid_argument("invalid allocation owners");
            }
            for (const auto& owner : state->owners()) {
                const auto& current = task(owner.task_id());
                if (!current.released || current.status != sentinel::v1::TASK_STATUS_PENDING
                    || owner.agent_id().empty() || owner.epoch() == 0 || owner.epoch() > allocation_epoch_
                    || owner.version() == 0 || owner.distance_mm() != 0
                    || owner.bundle_position() >= static_cast<std::uint32_t>(tasks_.size())) {
                    throw std::invalid_argument("invalid allocation owners");
                }
                vehicle(owner.agent_id());
            }
            std::vector<std::string> bundle(state->bundle_task_ids().begin(), state->bundle_task_ids().end());
            auto sorted_bundle = bundle;
            std::sort(sorted_bundle.begin(), sorted_bundle.end());
            if (std::adjacent_find(sorted_bundle.begin(), sorted_bundle.end()) != sorted_bundle.end()) {
                throw std::invalid_argument("invalid allocation bundle");
            }
            for (const auto& id : bundle) {
                task(id);
            }
            if (scenario_.allocation_policy() == sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE) {
                const auto invalid = [](const auto& bid) {
                    return bid.score() != 0 || bid.bundle_position() != 0;
                };
                if (!bundle.empty() || std::any_of(state->bids().begin(), state->bids().end(), invalid)
                    || std::any_of(state->winners().begin(), state->winners().end(), invalid)) {
                    throw std::invalid_argument("invalid nearest allocation state");
                }
            } else {
                if (bundle.size() != static_cast<std::size_t>(state->bids_size())) {
                    throw std::invalid_argument("invalid CBBA bundle");
                }
                std::vector<std::uint32_t> positions;
                for (const auto& id : bundle) {
                    const auto bid = std::lower_bound(state->bids().begin(), state->bids().end(),
                                                       id, [](const auto& value, std::string_view task_id) {
                                                          return value.task_id() < task_id;
                                                      });
                    if (bid == state->bids().end() || bid->task_id() != id
                        || bid->bundle_position() >= bundle.size()) {
                        throw std::invalid_argument("invalid CBBA bundle");
                    }
                    positions.push_back(bid->bundle_position());
                }
                std::sort(positions.begin(), positions.end());
                for (std::size_t index = 0; index < positions.size(); ++index) {
                    if (positions[index] != index) {
                        throw std::invalid_argument("invalid CBBA bundle");
                    }
                }
            }
        }
        std::sort(action->mutable_outgoing_messages()->begin(), action->mutable_outgoing_messages()->end(),
                  [](const auto& left, const auto& right) {
                      if (left.recipient_id() != right.recipient_id()) {
                          return left.recipient_id() < right.recipient_id();
                      }
                      if (left.sender_id() != right.sender_id()) {
                          return left.sender_id() < right.sender_id();
                      }
                      if (left.version() != right.version()) {
                          return left.version() < right.version();
                      }
                      return left.payload() < right.payload();
                  });
        std::sort(action->mutable_allocation_commits()->begin(), action->mutable_allocation_commits()->end(),
                  [](const auto& left, const auto& right) {
                      return left.task_id() < right.task_id();
                  });
        std::sort(action->mutable_task_reports()->begin(), action->mutable_task_reports()->end(),
                   [](const auto& left, const auto& right) {
                       return left.task_id() < right.task_id();
                   });
        std::sort(action->mutable_failure_detections()->begin(), action->mutable_failure_detections()->end(),
                  [](const auto& left, const auto& right) {
                      return std::tuple{left.failed_agent_id(), left.detector_agent_id()}
                             < std::tuple{right.failed_agent_id(), right.detector_agent_id()};
                  });
        std::sort(action->mutable_reservation_proposals()->begin(), action->mutable_reservation_proposals()->end(),
                  [](const auto& left, const auto& right) {
                      return std::tuple{left.resource_id(), left.start_tick(), left.end_tick(), left.version()}
                             < std::tuple{right.resource_id(), right.start_tick(), right.end_tick(), right.version()};
                  });
        std::sort(action->mutable_replanning_samples()->begin(), action->mutable_replanning_samples()->end(),
                  [](const auto& left, const auto& right) {
                      return std::tuple{left.start_tick(), left.reason(), left.wait_plan()}
                             < std::tuple{right.start_tick(), right.reason(), right.wait_plan()};
                  });
        for (const auto& message : action->outgoing_messages()) {
            if (message.sender_id() != envelope->sender_id() || message.version() == 0) {
                throw std::invalid_argument("invalid outgoing network message");
            }
            vehicle(message.recipient_id());
        }
        for (const auto& commit : action->allocation_commits()) {
            if (commit.agent_id() != envelope->sender_id() || commit.epoch() == 0 || commit.version() == 0) {
                throw std::invalid_argument("invalid allocation commit");
            }
            task(commit.task_id());
        }
        for (const auto& report : action->task_reports()) {
            if (!valid_report(report.kind())) {
                throw std::invalid_argument("invalid task report");
            }
            task(report.task_id());
        }
        for (const auto& detection : action->failure_detections()) {
            if (!valid_detection(detection, *envelope)) {
                throw std::invalid_argument("invalid failure detection");
            }
        }
        for (const auto& proposal : action->reservation_proposals()) {
            if (!valid_reservation(proposal, *envelope)) {
                throw std::invalid_argument("invalid reservation proposal");
            }
        }
        for (const auto& sample : action->replanning_samples()) {
            if (sample.agent_id() != envelope->sender_id() || !valid_replan_reason(sample.reason())
                || sample.start_tick() > sample.end_tick() || sample.end_tick() != tick_ || !sample.complete()) {
                throw std::invalid_argument("invalid replanning sample");
            }
        }
        if (std::adjacent_find(action->allocation_commits().begin(), action->allocation_commits().end(),
                               [](const auto& left, const auto& right) {
                                   return left.task_id() == right.task_id();
                               }) != action->allocation_commits().end()
            || std::adjacent_find(action->task_reports().begin(), action->task_reports().end(),
                                  [](const auto& left, const auto& right) {
                                      return left.task_id() == right.task_id();
                                  }) != action->task_reports().end()
            || std::adjacent_find(action->failure_detections().begin(), action->failure_detections().end(),
                                  [](const auto& left, const auto& right) {
                                      return left.failed_agent_id() == right.failed_agent_id();
                                  }) != action->failure_detections().end()) {
            throw std::invalid_argument("duplicate agent action");
        }
        if (!action->charge_location_id().empty()) {
            const auto& target = location(action->charge_location_id());
            if (target.kind() != sentinel::v1::LOCATION_KIND_CHARGING) {
                throw std::invalid_argument("invalid charging request");
            }
        }
    }
    return result;
}

std::vector<sentinel::v1::AppliedEvent> Simulator::apply_events(
    const google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent>* recorded_events,
    std::vector<sentinel::v1::NetworkOutcome>& network_outcomes) {
    std::vector<sentinel::v1::AppliedEvent> applied;
    while (next_event_ < static_cast<std::size_t>(scenario_.events_size())
           && scenario_.events(static_cast<int>(next_event_)).tick() == tick_) {
        const auto& source = scenario_.events(static_cast<int>(next_event_));
        sentinel::v1::AppliedEvent event;
        if (recorded_events) {
            if (applied.size() >= static_cast<std::size_t>(recorded_events->size())) {
                throw std::runtime_error("event log is missing a recorded event");
            }
            event.CopyFrom(recorded_events->Get(static_cast<int>(applied.size())));
            if (sentinel::protocol::deterministic_bytes(event.source())
                != sentinel::protocol::deterministic_bytes(source)) {
                throw std::runtime_error("event log source event mismatch");
            }
        } else {
            event.mutable_source()->CopyFrom(source);
        }
        if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK) {
            auto& target = task(source.target_id());
            if (!target.released) {
                target.released = true;
                if (target.assigned_agent_id.empty()) {
                    ++allocation_epoch_;
                    begin_convergence();
                }
            }
        } else if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE) {
            auto& target = vehicle(source.target_id());
            if (target.active) {
                target.active = false;
                target.velocity_x_mm_s = 0;
                target.velocity_y_mm_s = 0;
                target.failure_tick = tick_;
                reservations_.release_agent(target.spec.id());
                for (const auto& detector : vehicles_) {
                    if (detector.active && detector.spec.id() != target.spec.id()) {
                        failure_monitors_.push_back(
                            {detector.spec.id(), target.spec.id(), tick_, 0, std::nullopt});
                    }
                }
                for (const auto& current : tasks_) {
                    if (current.released && current.status == sentinel::v1::TASK_STATUS_PENDING
                        && current.assigned_agent_id == target.spec.id()) {
                        auto* sample = &task_reassignments_.emplace_back();
                        sample->set_task_id(current.spec.id());
                        sample->set_failed_agent_id(target.spec.id());
                        sample->set_failure_tick(tick_);
                        sample->set_previous_epoch(current.allocation_epoch);
                        sample->set_previous_version(current.allocation_version);
                    }
                }
            }
        } else if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA
                   || source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE
                   || source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE) {
            std::int64_t value{};
            if (recorded_events) {
                auto& rng = rng_streams_.stream(source.rng_stream());
                const auto expected = rng.uniform(source.value_min(), source.value_max());
                if (event.rng_stream() != source.rng_stream() || event.resolved_value() != expected
                    || event.rng_state_after() != rng.state()) {
                    throw std::runtime_error("event log random event mismatch");
                }
                value = event.resolved_value();
            } else {
                auto& rng = rng_streams_.stream(source.rng_stream());
                value = rng.uniform(source.value_min(), source.value_max());
                event.set_resolved_value(value);
                event.set_rng_stream(source.rng_stream());
                event.set_rng_state_after(rng.state());
            }
            auto& target = vehicle(source.target_id());
            if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA) {
                target.energy_mj = std::max<std::int64_t>(0, target.energy_mj + value);
                record_energy_invariant(target);
            } else if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE) {
                target.max_speed_mm_s = std::max<std::int64_t>(1, target.spec.max_speed_mm_s() * value / 1000);
                reservations_.release_route(target.spec.id(), target.route_version);
                ++allocation_epoch_;
                begin_convergence();
            } else {
                target.energy_capacity_mj =
                    std::max<std::int64_t>(1, target.spec.initial_energy_mj() * value / 1000);
                target.energy_mj = std::min(target.energy_mj, target.energy_capacity_mj);
                record_energy_invariant(target);
                reservations_.release_route(target.spec.id(), target.route_version);
                ++allocation_epoch_;
                begin_convergence();
            }
        } else if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE) {
            network_profile_ = source.text_value();
            network_.set_profile(network_profile(network_profile_));
        } else if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED) {
            auto* regions = scenario_.mutable_world()->mutable_regions();
            const auto position = std::lower_bound(regions->begin(), regions->end(), source.target_id(),
                                                   [](const auto& region, std::string_view id) {
                                                       return region.id() < id;
                                                   });
            if (position == regions->end() || position->id() != source.target_id()) {
                throw std::invalid_argument("unknown region");
            }
            if (position->closed() != source.bool_value()) {
                position->set_closed(source.bool_value());
                scenario_.mutable_world()->set_map_version(scenario_.world().map_version() + 1);
                reservations_.release_map(scenario_.world().map_version());
                begin_convergence();
            }
        } else if (source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED) {
            const auto was_blocked = network_.link_blocked(source.target_id(), source.text_value());
            auto outcomes = network_.set_link_blocked(source.target_id(), source.text_value(), source.bool_value(),
                                                      tick_);
            network_outcomes.insert(network_outcomes.end(), std::make_move_iterator(outcomes.begin()),
                                    std::make_move_iterator(outcomes.end()));
            if (was_blocked != source.bool_value()) {
                begin_convergence();
            }
        }
        const auto random_event = source.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA
                                  || source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE
                                  || source.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE;
        if (recorded_events && !random_event
            && (event.resolved_value() != 0 || !event.rng_stream().empty() || event.rng_state_after() != 0)) {
            throw std::runtime_error("event log contains invalid resolved input");
        }
        applied.push_back(std::move(event));
        ++next_event_;
    }
    if (recorded_events && applied.size() != static_cast<std::size_t>(recorded_events->size())) {
        throw std::runtime_error("event log contains an unexpected recorded event");
    }
    return applied;
}

bool Simulator::valid_commit(const sentinel::v1::AllocationCommit& commit,
                             const sentinel::v1::Envelope& envelope) const {
    if ((scenario_.allocation_policy() != sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE
         && scenario_.allocation_policy() != sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA)
        || commit.agent_id() != envelope.sender_id() || commit.epoch() != allocation_epoch_) {
        return false;
    }
    const auto& current_task = task(commit.task_id());
    if (!current_task.released || current_task.status != sentinel::v1::TASK_STATUS_PENDING
        || !current_task.assigned_agent_id.empty()) {
        return false;
    }
    if (!vehicle(envelope.sender_id()).active) {
        return false;
    }
    if (!envelope.action().has_allocation_state()) {
        if (scenario_.allocation_policy() != sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE
            || !scenario_.network_profiles().empty()) {
            return false;
        }
        std::vector<sentinel::v1::VehicleState> candidates;
        for (const auto& current : vehicles_) {
            if (current.active) {
                candidates.push_back(vehicle_state(current));
            }
        }
        const auto winner = planning::nearest_capable(scenario_.world(), candidates, task_state(current_task), tick_,
                                                       scenario_.step_ms(), commit.epoch(), commit.version());
        return winner && winner->agent_id() == envelope.sender_id()
               && winner->distance_mm() == commit.distance_mm();
    }
    const auto& state = envelope.action().allocation_state();
    const auto winner = std::lower_bound(state.winners().begin(), state.winners().end(), commit.task_id(),
                                         [](const auto& bid, std::string_view id) {
                                             return bid.task_id() < id;
                                         });
    if (winner == state.winners().end() || winner->task_id() != commit.task_id()
        || winner->bidder_id() != envelope.sender_id() || winner->version() != commit.version()
        || winner->score() != commit.score() || winner->bundle_position() != commit.bundle_position()) {
        return false;
    }
    const auto& current_vehicle = vehicle(envelope.sender_id());
    const auto claim = planning::make_claim(scenario_.world(), vehicle_state(current_vehicle), task_state(current_task),
                                            tick_, scenario_.step_ms(), commit.epoch(), commit.version());
    return claim.feasible() && claim.distance_mm() == commit.distance_mm();
}

void Simulator::apply_commits(const sentinel::v1::ActionBatch& actions, bool coordination_fence) {
    if (coordination_fence) {
        for (const auto& envelope : actions.actions()) {
            rejected_commits_ += static_cast<std::uint64_t>(envelope.action().allocation_commits_size());
        }
        return;
    }
    struct Proposal {
        const sentinel::v1::AllocationCommit* commit;
    };
    std::map<std::string, Proposal> selected;
    for (const auto& envelope : actions.actions()) {
        for (const auto& commit : envelope.action().allocation_commits()) {
            if (!valid_commit(commit, envelope)) {
                ++rejected_commits_;
                continue;
            }
            const auto existing = selected.find(commit.task_id());
            if (existing == selected.end()) {
                selected.emplace(commit.task_id(), Proposal{&commit});
                continue;
            }
            const auto& previous = *existing->second.commit;
            const auto better = scenario_.allocation_policy() == sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA
                                    ? commit.score() > previous.score()
                                          || (commit.score() == previous.score()
                                              && commit.agent_id() < previous.agent_id())
                                    : std::tuple{commit.distance_mm(), commit.agent_id()}
                                          < std::tuple{previous.distance_mm(), previous.agent_id()};
            ++rejected_commits_;
            if (better) {
                existing->second = Proposal{&commit};
            }
        }
    }
    for (const auto& [task_id, proposal] : selected) {
        auto& current = task(task_id);
        const auto& current_vehicle = vehicle(proposal.commit->agent_id());
        record_commit_invariants(current, current_vehicle);
        current.assigned_agent_id = proposal.commit->agent_id();
        current.allocation_epoch = proposal.commit->epoch();
        current.allocation_version = proposal.commit->version();
        current.allocation_score = proposal.commit->score();
        current.bundle_position = proposal.commit->bundle_position();
        const auto sample = std::find_if(task_reassignments_.begin(), task_reassignments_.end(),
                                         [&task_id](const auto& value) {
                                             return value.task_id() == task_id && value.detection_tick() != 0
                                                    && !value.complete();
                                         });
        if (sample != task_reassignments_.end() && proposal.commit->epoch() > sample->previous_epoch()) {
            sample->set_new_agent_id(proposal.commit->agent_id());
            sample->set_new_epoch(proposal.commit->epoch());
            sample->set_new_version(proposal.commit->version());
            sample->set_commit_tick(tick_);
            sample->set_complete(true);
        }
    }
}

void Simulator::apply_rejections(const sentinel::v1::ActionBatch& actions) {
    for (const auto& envelope : actions.actions()) {
        if (!vehicle(envelope.sender_id()).active) {
            continue;
        }
        for (const auto& report : envelope.action().task_reports()) {
            if (report.kind() != sentinel::v1::TASK_REPORT_KIND_REJECTED) {
                continue;
            }
            auto& current = task(report.task_id());
            if (current.status == sentinel::v1::TASK_STATUS_PENDING
                && current.assigned_agent_id == envelope.sender_id()) {
                current.assigned_agent_id.clear();
                current.allocation_epoch = 0;
                current.allocation_version = 0;
                current.allocation_score = 0;
                current.bundle_position = 0;
                current.progress_ticks = 0;
                ++allocation_epoch_;
                begin_convergence();
            }
        }
    }
}

bool Simulator::valid_detection(const sentinel::v1::FailureDetection& detection,
                                const sentinel::v1::Envelope& envelope) const {
    if (detection.detector_agent_id() != envelope.sender_id()
        || detection.failed_agent_id() == envelope.sender_id() || !vehicle(envelope.sender_id()).active) {
        return false;
    }
    const auto& failed = vehicle(detection.failed_agent_id());
    if (failed.active || !failed.failure_tick || *failed.failure_tick != detection.failure_tick()) {
        return false;
    }
    return std::any_of(failure_monitors_.begin(), failure_monitors_.end(), [&](const auto& monitor) {
        return monitor.detector_id == envelope.sender_id() && monitor.failed_id == detection.failed_agent_id()
               && monitor.detection_tick && *monitor.detection_tick == detection.detection_tick();
    });
}

void Simulator::apply_failure_detections(const sentinel::v1::ActionBatch& actions) {
    bool orphaned = false;
    std::map<std::string, const sentinel::v1::FailureDetection*> first;
    for (const auto& envelope : actions.actions()) {
        if (!vehicle(envelope.sender_id()).active) {
            continue;
        }
        for (const auto& detection : envelope.action().failure_detections()) {
            const auto recorded = std::any_of(failure_detections_.begin(), failure_detections_.end(),
                                              [&detection](const auto& current) {
                                                  return current.failed_agent_id() == detection.failed_agent_id()
                                                         && current.detector_agent_id()
                                                                == detection.detector_agent_id();
                                              });
            if (!recorded) {
                failure_detections_.push_back(detection);
            }
            const auto selected = first.find(detection.failed_agent_id());
            if (selected == first.end()
                || std::tuple{detection.detection_tick(), detection.detector_agent_id()}
                       < std::tuple{selected->second->detection_tick(), selected->second->detector_agent_id()}) {
                first[detection.failed_agent_id()] = &detection;
            }
        }
    }
    for (const auto& [failed_id, detection] : first) {
        auto& failed = vehicle(failed_id);
        if (failed.failure_announced) {
            continue;
        }
        failed.failure_announced = true;
        for (auto& sample : task_reassignments_) {
            if (sample.failed_agent_id() == failed.spec.id() && sample.detection_tick() == 0) {
                sample.set_detector_agent_id(detection->detector_agent_id());
                sample.set_detection_tick(detection->detection_tick());
            }
        }
        for (auto& current : tasks_) {
            if (current.released && current.status == sentinel::v1::TASK_STATUS_PENDING
                && current.assigned_agent_id == failed.spec.id()) {
                current.assigned_agent_id.clear();
                current.allocation_epoch = 0;
                current.allocation_version = 0;
                current.allocation_score = 0;
                current.bundle_position = 0;
                current.progress_ticks = 0;
                orphaned = true;
            }
        }
    }
    if (orphaned) {
        ++allocation_epoch_;
        begin_convergence();
    }
}

bool Simulator::valid_reservation(const sentinel::v1::SpaceTimeReservation& proposal,
                                  const sentinel::v1::Envelope& envelope) const {
    if (proposal.agent_id() != envelope.sender_id() || proposal.version() == 0
        || proposal.route_version() == 0 || proposal.route_version() != envelope.action().route_version()
        || proposal.map_version() != scenario_.world().map_version() || proposal.start_tick() <= tick_
        || proposal.start_tick() > proposal.end_tick()) {
        return false;
    }
    const auto region = std::lower_bound(scenario_.world().regions().begin(), scenario_.world().regions().end(),
                                         proposal.resource_id(), [](const auto& current, std::string_view id) {
                                             return current.id() < id;
                                         });
    return region != scenario_.world().regions().end() && region->id() == proposal.resource_id()
           && region->kind() == sentinel::v1::REGION_KIND_CHOKEPOINT && !region->closed();
}

void Simulator::apply_reservations(const sentinel::v1::ActionBatch& actions, bool coordination_fence) {
    reservations_.release_before(tick_);
    for (const auto& envelope : actions.actions()) {
        auto& current = vehicle(envelope.sender_id());
        const auto next_route = envelope.action().route_version();
        if (next_route != current.route_version) {
            reservations_.release_route(current.spec.id(), current.route_version);
            current.route_version = next_route;
        }
    }
    if (coordination_fence) {
        return;
    }
    for (const auto& envelope : actions.actions()) {
        if (!vehicle(envelope.sender_id()).active) {
            continue;
        }
        for (const auto& proposal : envelope.action().reservation_proposals()) {
            if (proposal.map_version() != scenario_.world().map_version()
                || proposal.route_version() != vehicle(envelope.sender_id()).route_version) {
                continue;
            }
            if (reservations_.reserve(reservation(proposal)) == planning::ReservationResult::committed) {
                record_reservation_invariant();
            }
        }
    }
}

bool Simulator::has_reservation(std::string_view agent_id, std::string_view resource_id,
                                std::uint64_t route_version) const {
    return std::any_of(reservations_.committed().begin(), reservations_.committed().end(),
                       [&](const auto& current) {
                           return current.agent_id == agent_id && current.resource_id == resource_id
                                  && current.start_tick <= tick_ && tick_ <= current.end_tick
                                  && current.route_version == route_version
                                  && current.map_version == scenario_.world().map_version();
                       });
}

bool Simulator::occupied(std::string_view resource_id, std::string_view agent_id) const {
    for (const auto& current : vehicles_) {
        if (!current.active || current.spec.id() == agent_id) {
            continue;
        }
        planning::Coordinate point;
        point << current.x_mm, current.y_mm;
        const auto resource = planning::contested_resource(scenario_.world(), point, point);
        if (resource && *resource == resource_id) {
            return true;
        }
    }
    return false;
}

void Simulator::apply_actions(const sentinel::v1::ActionBatch& actions,
                              std::vector<sentinel::v1::NetworkOutcome>& network_outcomes,
                              bool coordination_fence) {
    apply_failure_detections(actions);
    apply_commits(actions, coordination_fence);
    apply_rejections(actions);
    apply_reservations(actions, coordination_fence);
    std::vector<sentinel::v1::NetworkMessage> next_messages;
    for (int index = 0; index < actions.actions_size(); ++index) {
        auto& current = vehicles_[static_cast<std::size_t>(index)];
        const auto& action = actions.actions(index).action();
        current.behavior_mode = action.behavior_mode() == sentinel::v1::BEHAVIOR_MODE_UNSPECIFIED
                                    ? sentinel::v1::BEHAVIOR_MODE_IDLE
                                    : action.behavior_mode();
        if (action.has_allocation_state()) {
            allocation_views_[static_cast<std::size_t>(index)].CopyFrom(action.allocation_state());
        } else {
            allocation_views_[static_cast<std::size_t>(index)].Clear();
        }
        if (!current.active) {
            current.velocity_x_mm_s = 0;
            current.velocity_y_mm_s = 0;
            continue;
        }
        auto waited = current.behavior_mode == sentinel::v1::BEHAVIOR_MODE_WAITING;
        wait_ticks_ += waited ? 1 : 0;
        return_ticks_ += current.behavior_mode == sentinel::v1::BEHAVIOR_MODE_RETURNING ? 1 : 0;
        replan_count_ += action.replanned() ? 1 : 0;
        for (const auto& sample : action.replanning_samples()) {
            replanning_samples_.push_back(sample);
        }
        for (const auto& message : action.outgoing_messages()) {
            next_messages.push_back(message);
        }
        auto velocity_x = axis_velocity(action.velocity_x_mm_s(), current.max_speed_mm_s);
        auto velocity_y = axis_velocity(action.velocity_y_mm_s(), current.max_speed_mm_s);
        const auto magnitude = std::abs(velocity_x) + std::abs(velocity_y);
        if (magnitude > current.max_speed_mm_s) {
            velocity_x = velocity_x * current.max_speed_mm_s / magnitude;
            velocity_y = velocity_y * current.max_speed_mm_s / magnitude;
        }
        const auto delta_x = velocity_x * static_cast<std::int64_t>(scenario_.step_ms()) / 1000;
        const auto delta_y = velocity_y * static_cast<std::int64_t>(scenario_.step_ms()) / 1000;
        planning::Coordinate from;
        planning::Coordinate to;
        from << current.x_mm, current.y_mm;
        to << current.x_mm + delta_x, current.y_mm + delta_y;
        const auto state = vehicle_state(current);
        const auto resource = planning::contested_resource(scenario_.world(), from, to);
        const auto reservation_blocked = (delta_x != 0 || delta_y != 0) && resource
                                         && (!has_reservation(current.spec.id(), *resource, current.route_version)
                                             || occupied(*resource, current.spec.id()));
        if (reservation_blocked) {
            current.velocity_x_mm_s = 0;
            current.velocity_y_mm_s = 0;
            current.behavior_mode = sentinel::v1::BEHAVIOR_MODE_WAITING;
            if (!waited) {
                ++wait_ticks_;
            }
        } else if (!planning::segment_allowed(scenario_.world(), state, from, to)) {
            current.velocity_x_mm_s = 0;
            current.velocity_y_mm_s = 0;
        } else {
            const auto distance = std::abs(delta_x) + std::abs(delta_y);
            const auto cost = planning::motion_energy(
                state, distance, planning::terrain_multiplier(scenario_.world(), from, to));
            if (cost > current.energy_mj) {
                current.velocity_x_mm_s = 0;
                current.velocity_y_mm_s = 0;
            } else {
                current.x_mm = to.x();
                current.y_mm = to.y();
                current.velocity_x_mm_s = velocity_x;
                current.velocity_y_mm_s = velocity_y;
                current.energy_mj -= cost;
                record_energy_invariant(current);
                travel_distance_mm_ += static_cast<std::uint64_t>(distance);
                energy_consumed_mj_ += static_cast<std::uint64_t>(cost);
            }
        }
        if (!action.charge_location_id().empty() && action.velocity_x_mm_s() == 0
            && action.velocity_y_mm_s() == 0) {
            const auto& target = location(action.charge_location_id());
            sentinel::v1::Point position;
            position.set_x_mm(current.x_mm);
            position.set_y_mm(current.y_mm);
            if (inside(position, target)) {
                current.energy_mj = std::min(current.energy_capacity_mj,
                                              current.energy_mj + target.charge_mj_per_tick());
                record_energy_invariant(current);
                ++recharge_ticks_;
            }
        }
    }
    auto network = network_.step(tick_, next_messages);
    delivered_messages_ = std::move(network.delivered);
    network_outcomes.insert(network_outcomes.end(), std::make_move_iterator(network.outcomes.begin()),
                            std::make_move_iterator(network.outcomes.end()));
}

void Simulator::update_failure_monitors() {
    for (auto& monitor : failure_monitors_) {
        if (monitor.detection_tick || !vehicle(monitor.detector_id).active) {
            continue;
        }
        if (network_.link_blocked(monitor.detector_id, monitor.failed_id)) {
            monitor.missed_ticks = 0;
            continue;
        }
        const auto heard = std::any_of(delivered_messages_.begin(), delivered_messages_.end(),
                                       [&monitor](const auto& message) {
                                           return message.sender_id() == monitor.failed_id
                                                  && message.recipient_id() == monitor.detector_id;
                                       });
        monitor.missed_ticks = heard ? 0 : monitor.missed_ticks + 1;
        if (monitor.missed_ticks >= scenario_.failure_detection_ticks()) {
            monitor.detection_tick = tick_ + 1;
        }
    }
}

void Simulator::update_tasks(const sentinel::v1::ActionBatch& actions) {
    bool eligible_changed = false;
    for (auto& current : tasks_) {
        if (!current.released || current.status != sentinel::v1::TASK_STATUS_PENDING) {
            continue;
        }
        if (tick_ > current.spec.deadline_tick()) {
            current.status = sentinel::v1::TASK_STATUS_MISSED;
            current.progress_ticks = 0;
            eligible_changed = true;
            continue;
        }
        if (current.assigned_agent_id.empty()) {
            continue;
        }
        auto& assigned = vehicle(current.assigned_agent_id);
        const auto distance = std::abs(assigned.x_mm - current.spec.target().x_mm())
                              + std::abs(assigned.y_mm - current.spec.target().y_mm());
        const bool capable = assigned.active && has_capability(assigned, current.spec.required_capability())
                             && assigned.payload_grams >= current.spec.payload_required_grams();
        if (current.spec.service_ticks() == 0) {
            if (capable && distance <= current.spec.completion_radius_mm()) {
                current.status = sentinel::v1::TASK_STATUS_COMPLETED;
                current.completion_tick = tick_;
                eligible_changed = true;
                if (current.spec.required_capability() == sentinel::v1::CAPABILITY_DELIVERY) {
                    assigned.payload_grams -= current.spec.payload_required_grams();
                }
            }
            continue;
        }
        const auto envelope = std::lower_bound(actions.actions().begin(), actions.actions().end(),
                                               current.assigned_agent_id,
                                               [](const auto& value, std::string_view id) {
                                                   return value.sender_id() < id;
                                               });
        bool working = false;
        if (envelope != actions.actions().end() && envelope->sender_id() == current.assigned_agent_id) {
            const auto report = std::lower_bound(envelope->action().task_reports().begin(),
                                                 envelope->action().task_reports().end(), current.spec.id(),
                                                 [](const auto& value, std::string_view id) {
                                                     return value.task_id() < id;
                                                 });
            working = report != envelope->action().task_reports().end() && report->task_id() == current.spec.id()
                      && report->kind() == sentinel::v1::TASK_REPORT_KIND_WORKING;
        }
        const auto reserve = (assigned.spec.initial_energy_mj() + 9) / 10;
        const auto service_cost = current.spec.service_energy_mj_per_tick();
        if (!capable || distance > current.spec.completion_radius_mm() || !working
            || assigned.energy_mj < service_cost + reserve) {
            // task service needs an unbroken run of eligible ticks
            current.progress_ticks = 0;
            continue;
        }
        assigned.energy_mj -= service_cost;
        record_energy_invariant(assigned);
        energy_consumed_mj_ += static_cast<std::uint64_t>(service_cost);
        ++current.progress_ticks;
        if (current.progress_ticks >= current.spec.service_ticks()) {
            current.status = sentinel::v1::TASK_STATUS_COMPLETED;
            current.completion_tick = tick_;
            eligible_changed = true;
            if (current.spec.required_capability() == sentinel::v1::CAPABILITY_DELIVERY) {
                assigned.payload_grams -= current.spec.payload_required_grams();
            }
        }
    }
    if (eligible_changed) {
        begin_convergence();
    }
}

void Simulator::begin_convergence() {
    if (scenario_.allocation_policy() == sentinel::v1::ALLOCATION_POLICY_SCRIPTED) {
        return;
    }
    convergence_start_tick_ = tick_;
    convergence_candidate_tick_ = 0;
    convergence_stable_ticks_ = 0;
    convergence_signature_.clear();
    convergence_open_ = true;
}

bool Simulator::allocation_agrees() const {
    std::vector<std::string> tasks;
    for (const auto& current : tasks_) {
        if (current.released && current.status == sentinel::v1::TASK_STATUS_PENDING) {
            tasks.push_back(current.spec.id());
        }
    }
    std::sort(tasks.begin(), tasks.end());
    std::vector<bool> visited(vehicles_.size());
    bool any_active = false;
    for (std::size_t root = 0; root < vehicles_.size(); ++root) {
        if (!vehicles_[root].active || visited[root]) {
            continue;
        }
        any_active = true;
        std::vector<std::size_t> component{root};
        visited[root] = true;
        for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
            for (std::size_t candidate = 0; candidate < vehicles_.size(); ++candidate) {
                if (!vehicles_[candidate].active || visited[candidate]
                    || network_.link_blocked(vehicles_[component[cursor]].spec.id(),
                                             vehicles_[candidate].spec.id())) {
                    continue;
                }
                visited[candidate] = true;
                component.push_back(candidate);
            }
        }
        const sentinel::v1::AllocationState* reference = nullptr;
        for (const auto index : component) {
            const auto& view = allocation_views_[index];
            if (view.sender_id() != vehicles_[index].spec.id() || view.epoch() != allocation_epoch_
                || view.map_version() != scenario_.world().map_version()) {
                return false;
            }
            std::vector<std::string> covered;
            for (const auto& winner : view.winners()) {
                covered.push_back(winner.task_id());
            }
            for (const auto& owner : view.owners()) {
                covered.push_back(owner.task_id());
            }
            std::sort(covered.begin(), covered.end());
            if (covered != tasks) {
                return false;
            }
            if (reference) {
                if (view.winners_size() != reference->winners_size()
                    || view.owners_size() != reference->owners_size()) {
                    return false;
                }
                for (int bid = 0; bid < view.winners_size(); ++bid) {
                    if (protocol::deterministic_bytes(view.winners(bid))
                        != protocol::deterministic_bytes(reference->winners(bid))) {
                        return false;
                    }
                }
                for (int owner = 0; owner < view.owners_size(); ++owner) {
                    if (protocol::deterministic_bytes(view.owners(owner))
                        != protocol::deterministic_bytes(reference->owners(owner))) {
                        return false;
                    }
                }
            } else {
                reference = &view;
            }
        }
    }
    return any_active;
}

std::string Simulator::allocation_signature() const {
    HashBuilder hash;
    hash.unsigned_integer(allocation_epoch_);
    hash.unsigned_integer(scenario_.world().map_version());
    for (std::size_t index = 0; index < vehicles_.size(); ++index) {
        if (!vehicles_[index].active) {
            continue;
        }
        hash.text(vehicles_[index].spec.id());
        hash.text(protocol::deterministic_bytes(allocation_views_[index]));
    }
    return hash.finish();
}

void Simulator::update_convergence() {
    if (!convergence_open_ || !allocation_agrees()) {
        convergence_candidate_tick_ = 0;
        convergence_stable_ticks_ = 0;
        convergence_signature_.clear();
        return;
    }
    const auto signature = allocation_signature();
    if (signature != convergence_signature_) {
        convergence_signature_ = signature;
        convergence_candidate_tick_ = tick_;
        convergence_stable_ticks_ = 1;
        return;
    }
    ++convergence_stable_ticks_;
    if (convergence_stable_ticks_ < 10) {
        return;
    }
    auto* sample = &allocation_convergence_.emplace_back();
    sample->set_epoch(allocation_epoch_);
    sample->set_start_tick(convergence_start_tick_);
    sample->set_end_tick(convergence_candidate_tick_);
    sample->set_complete(true);
    convergence_open_ = false;
}

sentinel::v1::NetworkProfile Simulator::network_profile(std::string_view id) const {
    const auto position = std::lower_bound(scenario_.network_profiles().begin(), scenario_.network_profiles().end(), id,
                                           [](const auto& profile, std::string_view value) {
                                               return profile.id() < value;
                                           });
    if (position != scenario_.network_profiles().end() && position->id() == id) {
        return *position;
    }
    sentinel::v1::NetworkProfile profile;
    profile.set_id(std::string(id));
    profile.set_bandwidth_bytes_per_tick(1'000'000'000);
    return profile;
}

Simulator::Vehicle& Simulator::vehicle(std::string_view id) {
    const auto position = std::lower_bound(vehicles_.begin(), vehicles_.end(), id,
                                           [](const auto& current, std::string_view value) {
                                               return current.spec.id() < value;
                                           });
    if (position == vehicles_.end() || position->spec.id() != id) {
        throw std::invalid_argument("unknown vehicle");
    }
    return *position;
}

const Simulator::Vehicle& Simulator::vehicle(std::string_view id) const {
    const auto position = std::lower_bound(vehicles_.begin(), vehicles_.end(), id,
                                           [](const auto& current, std::string_view value) {
                                               return current.spec.id() < value;
                                           });
    if (position == vehicles_.end() || position->spec.id() != id) {
        throw std::invalid_argument("unknown vehicle");
    }
    return *position;
}

Simulator::Task& Simulator::task(std::string_view id) {
    const auto position = std::lower_bound(tasks_.begin(), tasks_.end(), id,
                                           [](const auto& current, std::string_view value) {
                                               return current.spec.id() < value;
                                           });
    if (position == tasks_.end() || position->spec.id() != id) {
        throw std::invalid_argument("unknown task");
    }
    return *position;
}

const Simulator::Task& Simulator::task(std::string_view id) const {
    const auto position = std::lower_bound(tasks_.begin(), tasks_.end(), id,
                                           [](const auto& current, std::string_view value) {
                                               return current.spec.id() < value;
                                           });
    if (position == tasks_.end() || position->spec.id() != id) {
        throw std::invalid_argument("unknown task");
    }
    return *position;
}

const sentinel::v1::ServiceLocation& Simulator::location(std::string_view id) const {
    const auto position = std::lower_bound(scenario_.world().locations().begin(), scenario_.world().locations().end(),
                                           id, [](const auto& current, std::string_view value) {
                                               return current.id() < value;
                                           });
    if (position == scenario_.world().locations().end() || position->id() != id) {
        throw std::invalid_argument("unknown service location");
    }
    return *position;
}

bool Simulator::has_capability(const Vehicle& current, sentinel::v1::Capability capability) const {
    return std::binary_search(current.spec.capabilities().begin(), current.spec.capabilities().end(), capability);
}

void Simulator::record_energy_invariant(const Vehicle& current) {
    agent_energy_below_zero_violations_ += current.energy_mj < 0 ? 1 : 0;
}

void Simulator::record_reservation_invariant() {
    const auto& committed = reservations_.committed();
    for (std::size_t left = 0; left < committed.size(); ++left) {
        for (std::size_t right = left + 1; right < committed.size(); ++right) {
            const auto& first = committed[left];
            const auto& second = committed[right];
            committed_reservation_overlap_violations_ +=
                first.resource_id == second.resource_id && first.start_tick <= second.end_tick
                        && second.start_tick <= first.end_tick
                    ? 1
                    : 0;
        }
    }
}

void Simulator::record_commit_invariants(const Task& current, const Vehicle& owner) {
    completed_task_reassignment_violations_ += current.status == sentinel::v1::TASK_STATUS_COMPLETED ? 1 : 0;
    incapable_agent_commit_violations_ +=
        !has_capability(owner, current.spec.required_capability())
                || owner.payload_grams < current.spec.payload_required_grams()
            ? 1
            : 0;
}

}
