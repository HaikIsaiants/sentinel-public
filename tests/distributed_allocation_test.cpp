#include "test_helpers.hpp"

#include <sentinel/core/event_log.hpp>
#include <sentinel/core/scenario.hpp>
#include <sentinel/protocol/framing.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>

namespace {

sentinel::v1::NetworkProfile profile(const char* id, std::uint64_t latency = 0,
                                     std::uint64_t jitter = 0, std::uint32_t loss = 0,
                                     std::uint64_t bandwidth = 1'000'000,
                                     std::uint32_t reorder = 0, std::uint32_t window = 0) {
    sentinel::v1::NetworkProfile value;
    value.set_id(id);
    value.set_latency_ticks(latency);
    value.set_jitter_ticks(jitter);
    value.set_packet_loss_permyriad(loss);
    value.set_bandwidth_bytes_per_tick(bandwidth);
    value.set_reorder_permyriad(reorder);
    value.set_reorder_window_ticks(window);
    return value;
}

sentinel::v1::Scenario scenario(sentinel::v1::AllocationPolicy policy, std::uint64_t max_ticks = 30) {
    sentinel::v1::Scenario value;
    value.set_schema_version(1);
    value.set_name("distributed-allocation-test");
    value.set_seed(31);
    value.set_step_ms(1000);
    value.set_max_ticks(max_ticks);
    value.set_network_profile("perfect");
    value.set_allocation_policy(policy);
    value.add_network_profiles()->CopyFrom(profile("perfect"));
    value.mutable_world()->set_width_mm(10000);
    value.mutable_world()->set_height_mm(10000);
    value.mutable_world()->set_grid_cell_mm(1000);
    value.mutable_world()->set_map_version(1);
    const auto add_vehicle = [&value](const char* id, const char* kind, std::int64_t x_mm) {
        auto* vehicle = value.add_vehicles();
        vehicle->set_id(id);
        vehicle->set_kind(kind);
        vehicle->mutable_initial_position()->set_x_mm(x_mm);
        vehicle->mutable_initial_position()->set_y_mm(1000);
        vehicle->set_max_speed_mm_s(1000);
        vehicle->set_initial_energy_mj(100000);
        vehicle->set_energy_cost_mj_per_meter(100);
        vehicle->add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
        vehicle->add_terrain_access("plain");
    };
    add_vehicle("alpha", "uav", 1000);
    add_vehicle("bravo", "ugv", 2000);
    add_vehicle("charlie", "rover", 3000);
    auto* task = value.add_tasks();
    task->set_id("target");
    task->set_kind("search");
    task->mutable_target()->set_x_mm(1000);
    task->mutable_target()->set_y_mm(1000);
    task->set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    task->set_deadline_tick(max_ticks);
    task->set_released(true);
    task->set_service_ticks(1);
    task->set_service_energy_mj_per_tick(10);
    task->set_priority(80);
    return value;
}

sentinel::v1::AgentAction& action(sentinel::v1::ActionBatch& actions, std::string_view id) {
    const auto position = std::find_if(actions.mutable_actions()->begin(), actions.mutable_actions()->end(),
                                       [id](const auto& envelope) {
                                           return envelope.sender_id() == id;
                                       });
    if (position == actions.mutable_actions()->end()) {
        throw std::logic_error("missing action");
    }
    return *position->mutable_action();
}

const sentinel::v1::AgentObservation& observation(const sentinel::v1::ObservationBatch& observations,
                                                   std::string_view id) {
    const auto position = std::find_if(observations.observations().begin(), observations.observations().end(),
                                       [id](const auto& envelope) {
                                           return envelope.recipient_id() == id;
                                       });
    if (position == observations.observations().end()) {
        throw std::logic_error("missing observation");
    }
    return position->observation();
}

const sentinel::v1::TaskState& task(const sentinel::v1::AgentObservation& observation) {
    if (observation.known_tasks_size() != 1) {
        throw std::logic_error("missing task");
    }
    return observation.known_tasks(0);
}

sentinel::v1::AllocationBid bid(const char* bidder, std::int64_t score, std::int64_t distance,
                                std::uint64_t version = 1) {
    sentinel::v1::AllocationBid value;
    value.set_epoch(1);
    value.set_version(version);
    value.set_task_id("target");
    value.set_bidder_id(bidder);
    value.set_score(score);
    value.set_distance_mm(distance);
    value.set_energy_mj(distance / 10);
    value.set_completion_tick(2);
    return value;
}

void propose(sentinel::v1::ActionBatch& actions, sentinel::v1::AllocationPolicy policy,
             const char* agent_id, std::int64_t score, std::int64_t distance) {
    auto& current = action(actions, agent_id);
    auto* state = current.mutable_allocation_state();
    state->set_epoch(1);
    state->set_version(1);
    state->set_sender_id(agent_id);
    state->set_map_version(1);
    auto own = bid(agent_id, policy == sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA ? score : 0,
                   distance);
    state->add_bids()->CopyFrom(own);
    state->add_winners()->CopyFrom(own);
    auto* relay = state->add_winner_relays();
    relay->mutable_bid()->CopyFrom(own);
    relay->add_path_agent_ids(agent_id);
    if (policy == sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA) {
        state->add_bundle_task_ids("target");
    }
    auto* commit = current.add_allocation_commits();
    commit->set_epoch(1);
    commit->set_version(1);
    commit->set_task_id("target");
    commit->set_agent_id(agent_id);
    commit->set_distance_mm(distance);
    commit->set_score(own.score());
}

sentinel::v1::TaskState allocate(sentinel::v1::AllocationPolicy policy, std::int64_t alpha_score,
                                 std::int64_t bravo_score) {
    sentinel::core::Simulator simulator(scenario(policy));
    auto actions = sentinel::test::idle_actions(simulator);
    propose(actions, policy, "alpha", alpha_score, 0);
    propose(actions, policy, "bravo", bravo_score, 1000);
    const auto outcome = simulator.step(actions);
    EXPECT_EQ(simulator.summary().rejected_commits(), 1U);
    for (const auto& envelope : outcome.observations.observations()) {
        if (!envelope.observation().assigned_tasks().empty()) {
            return envelope.observation().assigned_tasks(0);
        }
    }
    throw std::logic_error("missing assigned task");
}

sentinel::v1::AllocationState allocation_state(const char* sender, const char* winner,
                                                 std::int64_t score, std::uint64_t version = 1) {
    sentinel::v1::AllocationState value;
    value.set_epoch(1);
    value.set_version(version);
    value.set_sender_id(sender);
    value.set_map_version(1);
    value.add_winners()->CopyFrom(bid(winner, score, std::string_view(winner) == "alpha" ? 0 : 1000,
                                             version));
    auto* relay = value.add_winner_relays();
    relay->mutable_bid()->CopyFrom(value.winners(0));
    relay->add_path_agent_ids(winner);
    if (std::string_view(sender) != winner) {
        relay->add_path_agent_ids(sender);
    }
    return value;
}

sentinel::v1::AllocationState ownership_state(const char* sender, const sentinel::v1::TaskState& task,
                                              std::uint64_t version = 2) {
    sentinel::v1::AllocationState value;
    value.set_epoch(1);
    value.set_version(version);
    value.set_sender_id(sender);
    value.set_map_version(1);
    auto* owner = value.add_owners();
    owner->set_epoch(task.allocation_epoch());
    owner->set_version(task.allocation_version());
    owner->set_task_id(task.id());
    owner->set_agent_id(task.assigned_agent_id());
    owner->set_score(task.allocation_score());
    owner->set_bundle_position(task.bundle_position());
    return value;
}

void message(sentinel::v1::ActionBatch& actions, const char* sender, const char* recipient,
             std::string_view payload, std::uint64_t version) {
    auto* value = action(actions, sender).add_outgoing_messages();
    value->set_sender_id(sender);
    value->set_recipient_id(recipient);
    value->set_version(version);
    value->set_payload(payload.data(), payload.size());
}

void state_message(sentinel::v1::ActionBatch& actions, const sentinel::v1::AllocationState& state,
                   const char* recipient) {
    const auto payload = sentinel::protocol::deterministic_bytes(state);
    message(actions, state.sender_id().c_str(), recipient, payload, state.version());
}

sentinel::v1::TapeEvent& event(sentinel::v1::Scenario& value, std::string_view id) {
    const auto position = std::find_if(value.mutable_events()->begin(), value.mutable_events()->end(),
                                       [id](const auto& current) {
                                           return current.id() == id;
                                       });
    if (position == value.mutable_events()->end()) {
        throw std::logic_error("missing event");
    }
    return *position;
}

}

TEST(DistributedAllocation, UsesDistanceForNearestAndScoreThenIdForCbba) {
    const auto nearest = allocate(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, 10, 100);
    EXPECT_EQ(nearest.assigned_agent_id(), "alpha");
    EXPECT_EQ(nearest.allocation_score(), 0);

    const auto cbba = allocate(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, 10, 100);
    EXPECT_EQ(cbba.assigned_agent_id(), "bravo");
    EXPECT_EQ(cbba.allocation_score(), 100);

    const auto tie = allocate(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, 100, 100);
    EXPECT_EQ(tie.assigned_agent_id(), "alpha");
}

TEST(DistributedAllocation, ProjectsDynamicOwnershipPerAgent) {
    sentinel::core::Simulator simulator(scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA));
    auto actions = sentinel::test::idle_actions(simulator);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "alpha", 100, 0);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "bravo", 10, 1000);
    auto outcome = simulator.step(actions);
    const auto& accepted = observation(outcome.observations, "alpha");
    const auto& unaware = observation(outcome.observations, "charlie");
    ASSERT_EQ(accepted.assigned_tasks_size(), 1);
    EXPECT_EQ(task(accepted).assigned_agent_id(), "alpha");
    EXPECT_TRUE(task(unaware).assigned_agent_id().empty());
    ASSERT_EQ(unaware.available_tasks_size(), 1);

    actions = sentinel::test::idle_actions(simulator);
    action(actions, "charlie").mutable_allocation_state()->CopyFrom(
        ownership_state("charlie", accepted.assigned_tasks(0)));
    outcome = simulator.step(actions);
    const auto& learned = observation(outcome.observations, "charlie");
    EXPECT_EQ(task(learned).assigned_agent_id(), "alpha");
    EXPECT_TRUE(learned.assigned_tasks().empty());
    EXPECT_TRUE(learned.available_tasks().empty());
}

TEST(DistributedAllocation, AcknowledgesSelfRevocationOverStaleOwnerView) {
    sentinel::core::Simulator simulator(scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA));
    auto actions = sentinel::test::idle_actions(simulator);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "alpha", 100, 0);
    auto outcome = simulator.step(actions);
    const auto assigned = observation(outcome.observations, "alpha").assigned_tasks(0);

    actions = sentinel::test::idle_actions(simulator);
    action(actions, "alpha").mutable_allocation_state()->CopyFrom(ownership_state("alpha", assigned));
    auto* report = action(actions, "alpha").add_task_reports();
    report->set_task_id("target");
    report->set_kind(sentinel::v1::TASK_REPORT_KIND_REJECTED);
    outcome = simulator.step(actions);
    const auto& local = observation(outcome.observations, "alpha");
    EXPECT_TRUE(task(local).assigned_agent_id().empty());
    ASSERT_EQ(local.available_tasks_size(), 1);
}

TEST(DistributedAllocation, RejectsCommitComputedBeforeLinkCut) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
    auto* cut = value.add_events();
    cut->set_id("cut");
    cut->set_tick(0);
    cut->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED);
    cut->set_target_id("alpha");
    cut->set_text_value("bravo");
    cut->set_bool_value(true);
    sentinel::core::Simulator simulator(value);
    auto actions = sentinel::test::idle_actions(simulator);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "alpha", 100, 0);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "bravo", 10, 1000);
    const auto outcome = simulator.step(actions);
    EXPECT_EQ(simulator.summary().rejected_commits(), 2U);
    EXPECT_TRUE(observation(outcome.observations, "alpha").assigned_tasks().empty());
    const auto& reachable = observation(outcome.observations, "alpha").reachable_peer_ids();
    EXPECT_EQ(std::find(reachable.begin(), reachable.end(), "bravo"), reachable.end());
}

TEST(DistributedAllocation, RejectsCommitComputedBeforeLinkHeal) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
    auto* cut = value.add_events();
    cut->set_id("cut");
    cut->set_tick(0);
    cut->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED);
    cut->set_target_id("alpha");
    cut->set_text_value("bravo");
    cut->set_bool_value(true);
    auto* heal = value.add_events();
    heal->set_id("heal");
    heal->set_tick(1);
    heal->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED);
    heal->set_target_id("alpha");
    heal->set_text_value("bravo");
    sentinel::core::Simulator simulator(value);
    simulator.step(sentinel::test::idle_actions(simulator));
    auto actions = sentinel::test::idle_actions(simulator);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "alpha", 100, 0);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "bravo", 10, 1000);
    const auto outcome = simulator.step(actions);
    EXPECT_EQ(simulator.summary().rejected_commits(), 2U);
    EXPECT_TRUE(observation(outcome.observations, "alpha").assigned_tasks().empty());
}

TEST(DistributedAllocation, ClearsLocallyDetectedOwnerProjection) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
    value.set_failure_detection_ticks(1);
    value.mutable_tasks(0)->set_assigned_agent_id("alpha");
    auto* disable = value.add_events();
    disable->set_id("disable-alpha");
    disable->set_tick(0);
    disable->set_kind(sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE);
    disable->set_target_id("alpha");
    sentinel::core::Simulator simulator(value);
    const auto assigned = task(observation(simulator.observe(), "alpha"));
    auto actions = sentinel::test::idle_actions(simulator);
    action(actions, "bravo").mutable_allocation_state()->CopyFrom(ownership_state("bravo", assigned));
    const auto outcome = simulator.step(actions);
    const auto& local = observation(outcome.observations, "bravo");
    ASSERT_EQ(local.failure_detections_size(), 1);
    EXPECT_EQ(local.failure_detections(0).failed_agent_id(), "alpha");
    EXPECT_TRUE(task(local).assigned_agent_id().empty());
    ASSERT_EQ(local.available_tasks_size(), 1);
}

TEST(DistributedAllocation, RejectsMalformedPolicyState) {
    sentinel::core::Simulator nearest(scenario(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE));
    auto actions = sentinel::test::idle_actions(nearest);
    auto* state = action(actions, "alpha").mutable_allocation_state();
    state->set_epoch(1);
    state->set_version(1);
    state->set_sender_id("alpha");
    state->set_map_version(1);
    state->add_bids()->CopyFrom(bid("alpha", 1, 0));
    state->add_winners()->CopyFrom(bid("alpha", 1, 0));
    EXPECT_THROW(nearest.step(actions), std::invalid_argument);

    sentinel::core::Simulator cbba(scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA));
    actions = sentinel::test::idle_actions(cbba);
    state = action(actions, "alpha").mutable_allocation_state();
    state->set_epoch(1);
    state->set_version(1);
    state->set_sender_id("alpha");
    state->set_map_version(1);
    state->add_bids()->CopyFrom(bid("alpha", 1, 0));
    state->add_winners()->CopyFrom(bid("alpha", 1, 0));
    EXPECT_THROW(cbba.step(actions), std::invalid_argument);
}

TEST(DistributedAllocation, PartitionRecovery) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
    value.add_network_profiles()->CopyFrom(profile("slow", 2));
    value.set_network_profile("slow");
    auto* cut = value.add_events();
    cut->set_id("cut");
    cut->set_tick(1);
    cut->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED);
    cut->set_target_id("alpha");
    cut->set_text_value("bravo");
    cut->set_bool_value(true);
    auto* heal = value.add_events();
    heal->set_id("heal");
    heal->set_tick(3);
    heal->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED);
    heal->set_target_id("bravo");
    heal->set_text_value("alpha");

    sentinel::core::Simulator simulator(value);
    auto actions = sentinel::test::idle_actions(simulator);
    state_message(actions, allocation_state("alpha", "alpha", 10), "bravo");
    auto outcome = simulator.step(actions);
    ASSERT_EQ(outcome.network_outcomes.size(), 1U);
    EXPECT_EQ(outcome.network_outcomes[0].disposition(), sentinel::v1::NETWORK_DISPOSITION_QUEUED);
    EXPECT_TRUE(observation(outcome.observations, "bravo").delivered_messages().empty());

    outcome = simulator.step(sentinel::test::idle_actions(simulator));
    ASSERT_EQ(outcome.applied_events.size(), 1U);
    ASSERT_EQ(outcome.network_outcomes.size(), 1U);
    EXPECT_EQ(outcome.network_outcomes[0].disposition(),
              sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION);
    EXPECT_EQ(outcome.network_outcomes[0].sequence(), 1U);

    simulator.step(sentinel::test::idle_actions(simulator));
    actions = sentinel::test::idle_actions(simulator);
    state_message(actions, allocation_state("alpha", "alpha", 20, 2), "bravo");
    outcome = simulator.step(actions);
    ASSERT_EQ(outcome.applied_events.size(), 1U);
    ASSERT_EQ(outcome.network_outcomes.size(), 1U);
    EXPECT_EQ(outcome.network_outcomes[0].disposition(), sentinel::v1::NETWORK_DISPOSITION_QUEUED);

    outcome = simulator.step(sentinel::test::idle_actions(simulator));
    ASSERT_EQ(outcome.network_outcomes.size(), 1U);
    EXPECT_EQ(outcome.network_outcomes[0].disposition(), sentinel::v1::NETWORK_DISPOSITION_DELIVERED);
    const auto& delivered = observation(outcome.observations, "bravo").delivered_messages();
    ASSERT_EQ(delivered.size(), 1);
    sentinel::v1::AllocationState received;
    ASSERT_TRUE(received.ParseFromString(delivered.Get(0).payload()));
    EXPECT_EQ(received.version(), 2U);
    EXPECT_EQ(received.winners(0).score(), 20);
    EXPECT_EQ(simulator.summary().communication_messages(), 2U);
    EXPECT_EQ(simulator.summary().dropped_messages(), 1U);
    EXPECT_EQ(simulator.summary().delivered_messages(), 1U);
}

TEST(DistributedAllocation, RecordsStableAllocationConvergence) {
    sentinel::core::Simulator simulator(scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA));
    for (int tick = 0; tick < 10; ++tick) {
        auto actions = sentinel::test::idle_actions(simulator);
        for (const auto& vehicle : simulator.scenario().vehicles()) {
            action(actions, vehicle.id()).mutable_allocation_state()->CopyFrom(
                allocation_state(vehicle.id().c_str(), "alpha", 10));
        }
        simulator.step(actions);
    }
    const auto summary = simulator.summary();
    ASSERT_EQ(summary.allocation_convergence_size(), 1);
    EXPECT_EQ(summary.allocation_convergence(0).epoch(), 1U);
    EXPECT_EQ(summary.allocation_convergence(0).start_tick(), 0U);
    EXPECT_EQ(summary.allocation_convergence(0).end_tick(), 1U);
    EXPECT_TRUE(summary.allocation_convergence(0).complete());
}

TEST(DistributedAllocation, RecordsComponentLocalAllocationConvergence) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
    for (const auto peer : {"bravo", "charlie"}) {
        auto* cut = value.add_events();
        cut->set_id(std::string("cut-alpha-") + peer);
        cut->set_tick(0);
        cut->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED);
        cut->set_target_id("alpha");
        cut->set_text_value(peer);
        cut->set_bool_value(true);
    }
    sentinel::core::Simulator simulator(value);
    for (int tick = 0; tick < 10; ++tick) {
        auto actions = sentinel::test::idle_actions(simulator);
        for (const auto& vehicle : simulator.scenario().vehicles()) {
            const auto* winner = vehicle.id() == "alpha" ? "alpha" : "bravo";
            action(actions, vehicle.id()).mutable_allocation_state()->CopyFrom(
                allocation_state(vehicle.id().c_str(), winner, 10));
        }
        simulator.step(actions);
    }
    const auto summary = simulator.summary();
    ASSERT_EQ(summary.allocation_convergence_size(), 1);
    EXPECT_TRUE(summary.allocation_convergence(0).complete());
}

TEST(DistributedAllocation, ConvergenceIncludesCommittedOwnershipVersion) {
    sentinel::core::Simulator simulator(scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA));
    auto actions = sentinel::test::idle_actions(simulator);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "alpha", 100, 0);
    propose(actions, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, "bravo", 10, 1000);
    auto outcome = simulator.step(actions);
    const auto assigned = task(observation(outcome.observations, "alpha"));
    ASSERT_EQ(assigned.assigned_agent_id(), "alpha");

    actions = sentinel::test::idle_actions(simulator);
    for (const auto& vehicle : simulator.scenario().vehicles()) {
        action(actions, vehicle.id()).mutable_allocation_state()->CopyFrom(
            ownership_state(vehicle.id().c_str(), assigned));
    }
    action(actions, "bravo").mutable_allocation_state()->mutable_owners(0)->set_epoch(
        assigned.allocation_epoch() + 1);
    EXPECT_THROW(simulator.step(actions), std::invalid_argument);

    for (int tick = 0; tick < 10; ++tick) {
        actions = sentinel::test::idle_actions(simulator);
        for (const auto& vehicle : simulator.scenario().vehicles()) {
            action(actions, vehicle.id()).mutable_allocation_state()->CopyFrom(
                ownership_state(vehicle.id().c_str(), assigned));
        }
        simulator.step(actions);
    }
    auto summary = simulator.summary();
    ASSERT_EQ(summary.allocation_convergence_size(), 1);
    EXPECT_TRUE(summary.allocation_convergence(0).complete());

    actions = sentinel::test::idle_actions(simulator);
    for (const auto& vehicle : simulator.scenario().vehicles()) {
        action(actions, vehicle.id()).mutable_allocation_state()->CopyFrom(
            ownership_state(vehicle.id().c_str(), assigned));
    }
    auto* report = action(actions, "alpha").add_task_reports();
    report->set_task_id("target");
    report->set_kind(sentinel::v1::TASK_REPORT_KIND_WORKING);
    simulator.step(actions);
    summary = simulator.summary();
    ASSERT_EQ(summary.allocation_convergence_size(), 2);
    EXPECT_FALSE(summary.allocation_convergence(1).complete());
    EXPECT_EQ(summary.allocation_convergence(1).start_tick(), simulator.tick());
    EXPECT_EQ(summary.allocation_convergence(1).end_tick(), simulator.tick());
}

TEST(DistributedAllocation, ReportsOpenConvergenceAtCurrentHorizon) {
    sentinel::core::Simulator simulator(scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA));
    simulator.step(sentinel::test::idle_actions(simulator));
    const auto summary = simulator.summary();
    ASSERT_EQ(summary.allocation_convergence_size(), 1);
    EXPECT_EQ(summary.allocation_convergence(0).start_tick(), 0U);
    EXPECT_EQ(summary.allocation_convergence(0).end_tick(), 1U);
    EXPECT_FALSE(summary.allocation_convergence(0).complete());
}

TEST(DistributedAllocation, NetworkOutcomeReplay) {
    const auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, 3);
    sentinel::core::Simulator source(value);
    sentinel::core::Simulator reordered(value);
    auto actions = sentinel::test::idle_actions(source);
    message(actions, "alpha", "charlie", "z", 2);
    message(actions, "alpha", "bravo", "a", 1);
    message(actions, "bravo", "alpha", "b", 1);
    auto reordered_actions = sentinel::test::idle_actions(reordered, true);
    message(reordered_actions, "bravo", "alpha", "b", 1);
    message(reordered_actions, "alpha", "bravo", "a", 1);
    message(reordered_actions, "alpha", "charlie", "z", 2);
    const auto path = std::filesystem::temp_directory_path() / "sentinel-allocation-network-replay.events.pb";
    const auto outcome = source.step(actions);
    const auto reordered_outcome = reordered.step(reordered_actions);
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(outcome.actions),
              sentinel::protocol::deterministic_bytes(reordered_outcome.actions));
    ASSERT_EQ(outcome.network_outcomes.size(), reordered_outcome.network_outcomes.size());
    for (std::size_t idx = 0; idx < outcome.network_outcomes.size(); ++idx) {
        EXPECT_EQ(sentinel::protocol::deterministic_bytes(outcome.network_outcomes[idx]),
                  sentinel::protocol::deterministic_bytes(reordered_outcome.network_outcomes[idx]));
        if (idx > 0) {
            const auto& left = outcome.network_outcomes[idx - 1];
            const auto& right = outcome.network_outcomes[idx];
            EXPECT_LE((std::tuple{left.tick(), left.sequence(), left.disposition()}),
                      (std::tuple{right.tick(), right.sequence(), right.disposition()}));
        }
    }
    EXPECT_EQ(source.state_hash(), reordered.state_hash());
    {
        sentinel::core::EventLogWriter writer(path, value);
        writer.append(source.tick(), outcome.actions, outcome.applied_events, outcome.network_outcomes,
                      source.state_hash());
        while (!source.finished()) {
            const auto next = source.step(sentinel::test::idle_actions(source));
            writer.append(source.tick(), next.actions, next.applied_events, next.network_outcomes,
                          source.state_hash());
        }
        writer.finish(source.summary());
    }
    const auto replay = sentinel::core::replay_event_log(value, path);
    EXPECT_EQ(replay.terminal_hash(), source.summary().terminal_hash());
    EXPECT_EQ(replay.communication_messages(), 3U);
    EXPECT_EQ(replay.delivered_messages(), 3U);
    std::filesystem::remove(path);
}

TEST(DistributedAllocation, HashesDroppedNetworkOutcomeBytes) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
    value.add_network_profiles()->CopyFrom(profile("loss", 0, 0, 10000));
    value.set_network_profile("loss");
    sentinel::core::Simulator first(value);
    sentinel::core::Simulator second(value);
    auto first_actions = sentinel::test::idle_actions(first);
    auto second_actions = sentinel::test::idle_actions(second);
    message(first_actions, "alpha", "bravo", "a", 1);
    message(second_actions, "alpha", "bravo", "b", 1);
    const auto first_outcome = first.step(first_actions);
    const auto second_outcome = second.step(second_actions);
    ASSERT_EQ(first_outcome.network_outcomes.size(), 1U);
    ASSERT_EQ(second_outcome.network_outcomes.size(), 1U);
    EXPECT_EQ(first_outcome.network_outcomes[0].disposition(),
              sentinel::v1::NETWORK_DISPOSITION_DROPPED_LOSS);
    EXPECT_EQ(second_outcome.network_outcomes[0].disposition(),
              sentinel::v1::NETWORK_DISPOSITION_DROPPED_LOSS);
    EXPECT_NE(first.state_hash(), second.state_hash());
}

TEST(DistributedAllocationScenario, ValidatesAndCanonicalizesNetworkAllocationFields) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
    value.add_network_profiles()->CopyFrom(profile("impaired", 2, 1, 100, 500, 2500, 3));
    auto* cut = value.add_events();
    cut->set_id("cut");
    cut->set_tick(1);
    cut->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED);
    cut->set_target_id("alpha");
    cut->set_text_value("bravo");
    cut->set_bool_value(true);
    auto* switch_profile = value.add_events();
    switch_profile->set_id("profile");
    switch_profile->set_tick(2);
    switch_profile->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE);
    switch_profile->set_text_value("impaired");
    sentinel::core::normalize_scenario(value);
    EXPECT_NO_THROW(sentinel::core::validate_scenario(value));

    auto reordered = value;
    std::reverse(reordered.mutable_network_profiles()->begin(), reordered.mutable_network_profiles()->end());
    std::reverse(reordered.mutable_events()->begin(), reordered.mutable_events()->end());
    EXPECT_EQ(sentinel::core::scenario_hash(value), sentinel::core::scenario_hash(reordered));

    auto invalid = value;
    invalid.mutable_network_profiles(0)->set_packet_loss_permyriad(10001);
    EXPECT_THROW(sentinel::core::validate_scenario(invalid), std::invalid_argument);
    invalid = value;
    invalid.mutable_network_profiles(0)->set_bandwidth_bytes_per_tick(0);
    EXPECT_THROW(sentinel::core::validate_scenario(invalid), std::invalid_argument);
    invalid = value;
    invalid.mutable_network_profiles(0)->set_reorder_permyriad(1);
    invalid.mutable_network_profiles(0)->set_reorder_window_ticks(0);
    EXPECT_THROW(sentinel::core::validate_scenario(invalid), std::invalid_argument);
    invalid = value;
    invalid.set_network_profile("missing");
    EXPECT_THROW(sentinel::core::validate_scenario(invalid), std::invalid_argument);
    invalid = value;
    event(invalid, "cut").set_text_value("alpha");
    EXPECT_THROW(sentinel::core::validate_scenario(invalid), std::invalid_argument);
    invalid = value;
    event(invalid, "profile").set_text_value("missing");
    EXPECT_THROW(sentinel::core::validate_scenario(invalid), std::invalid_argument);
    invalid = value;
    invalid.mutable_tasks(0)->set_priority(101);
    EXPECT_THROW(sentinel::core::validate_scenario(invalid), std::invalid_argument);

    auto normalized = value;
    normalized.mutable_tasks(0)->set_priority(0);
    sentinel::core::normalize_scenario(normalized);
    EXPECT_EQ(normalized.tasks(0).priority(), 50U);
    EXPECT_NO_THROW(sentinel::core::validate_scenario(normalized));
}

TEST(DistributedAllocationScenario, PairedFixturesDifferOnlyByPolicyAndIdentity) {
    const auto paired = [](const char* cbba_name, const char* nearest_name) {
        const auto root = std::filesystem::path(SENTINEL_SOURCE_DIR) / "scenarios";
        auto cbba = sentinel::core::load_scenario(root / cbba_name);
        auto nearest = sentinel::core::load_scenario(root / nearest_name);
        EXPECT_EQ(cbba.allocation_policy(), sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
        EXPECT_EQ(nearest.allocation_policy(), sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
        cbba.set_name("paired");
        nearest.set_name("paired");
        cbba.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
        nearest.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
        EXPECT_EQ(sentinel::protocol::deterministic_bytes(cbba),
                  sentinel::protocol::deterministic_bytes(nearest));
    };
    paired("connected_cbba.textproto", "connected_nearest.textproto");
    paired("impaired_network_cbba.textproto", "impaired_network_nearest.textproto");
}
