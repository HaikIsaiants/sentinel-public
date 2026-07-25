#include "test_helpers.hpp"

#include <sentinel/core/scenario.hpp>
#include <sentinel/protocol/framing.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string_view>

namespace {

sentinel::v1::Scenario scenario(std::int64_t alpha_x = 1000, std::int64_t bravo_x = 7000,
                                std::int64_t charlie_x = 9000) {
    sentinel::v1::Scenario value;
    value.set_schema_version(1);
    value.set_name("failure-recovery-test");
    value.set_seed(73);
    value.set_step_ms(1000);
    value.set_max_ticks(20);
    value.set_network_profile("perfect");
    value.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_SCRIPTED);
    value.set_failure_detection_ticks(2);
    auto* profile = value.add_network_profiles();
    profile->set_id("perfect");
    profile->set_bandwidth_bytes_per_tick(1'000'000);
    auto* world = value.mutable_world();
    world->set_width_mm(10000);
    world->set_height_mm(5000);
    world->set_grid_cell_mm(500);
    world->set_map_version(1);
    const auto add_vehicle = [&value](const char* id, const char* kind, std::int64_t x) {
        auto* vehicle = value.add_vehicles();
        vehicle->set_id(id);
        vehicle->set_kind(kind);
        vehicle->mutable_initial_position()->set_x_mm(x);
        vehicle->mutable_initial_position()->set_y_mm(1000);
        vehicle->set_max_speed_mm_s(1000);
        vehicle->set_initial_energy_mj(10000);
        vehicle->set_energy_cost_mj_per_meter(0);
        vehicle->add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
        vehicle->add_terrain_access("plain");
    };
    add_vehicle("alpha", "uav", alpha_x);
    add_vehicle("bravo", "ugv", bravo_x);
    add_vehicle("charlie", "rover", charlie_x);
    return value;
}

sentinel::v1::TaskSpec& add_task(sentinel::v1::Scenario& value, const char* id,
                                 const char* owner, std::int64_t x, std::uint64_t service_ticks) {
    auto* task = value.add_tasks();
    task->set_id(id);
    task->set_kind("search");
    task->mutable_target()->set_x_mm(x);
    task->mutable_target()->set_y_mm(1000);
    task->set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    task->set_deadline_tick(value.max_ticks());
    task->set_completion_radius_mm(0);
    task->set_assigned_agent_id(owner);
    task->set_released(true);
    task->set_service_ticks(service_ticks);
    task->set_service_energy_mj_per_tick(10);
    task->set_priority(80);
    return *task;
}

sentinel::v1::TapeEvent& add_event(sentinel::v1::Scenario& value, const char* id,
                                   std::uint64_t tick, sentinel::v1::TapeEventKind kind,
                                   const char* target) {
    auto* event = value.add_events();
    event->set_id(id);
    event->set_tick(tick);
    event->set_kind(kind);
    event->set_target_id(target);
    return *event;
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

const sentinel::v1::TaskState& task(const sentinel::v1::AgentObservation& observation,
                                    std::string_view id) {
    const auto position = std::find_if(observation.known_tasks().begin(), observation.known_tasks().end(),
                                       [id](const auto& value) {
                                           return value.id() == id;
                                       });
    if (position == observation.known_tasks().end()) {
        throw std::logic_error("missing task");
    }
    return *position;
}

void work(sentinel::v1::ActionBatch& actions, const char* task_id) {
    auto* report = action(actions, "alpha").add_task_reports();
    report->set_task_id(task_id);
    report->set_kind(sentinel::v1::TASK_REPORT_KIND_WORKING);
}

void proposal(sentinel::v1::ActionBatch& actions, const char* agent_id,
              std::uint64_t start, std::uint64_t end, std::uint64_t version) {
    auto& current = action(actions, agent_id);
    current.set_route_version(1);
    auto* value = current.add_reservation_proposals();
    value->set_resource_id("pass");
    value->set_agent_id(agent_id);
    value->set_start_tick(start);
    value->set_end_tick(end);
    value->set_version(version);
    value->set_route_version(1);
    value->set_map_version(1);
}

}

TEST(FailureRecovery, AppliesPermanentDegradationAndCapsCharging) {
    auto value = scenario();
    add_task(value, "hold", "bravo", 10000, 5);
    auto* charger = value.mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(1500);
    charger->mutable_position()->set_y_mm(1000);
    charger->set_charge_mj_per_tick(4000);

    auto& speed = add_event(value, "speed", 0, sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE,
                            "alpha");
    speed.set_value_min(500);
    speed.set_value_max(500);
    speed.set_rng_stream("failure");
    auto& endurance = add_event(value, "endurance", 0,
                                sentinel::v1::TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE, "alpha");
    endurance.set_value_min(500);
    endurance.set_value_max(500);
    endurance.set_rng_stream("failure");
    auto& drain = add_event(value, "drain", 1, sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA, "alpha");
    drain.set_value_min(-2000);
    drain.set_value_max(-2000);
    drain.set_rng_stream("environment");

    sentinel::core::Simulator simulator(value);
    auto actions = sentinel::test::idle_actions(simulator);
    action(actions, "alpha").set_velocity_x_mm_s(1000);
    auto outcome = simulator.step(actions);
    const auto& degraded = observation(outcome.observations, "alpha").self();
    EXPECT_EQ(degraded.max_speed_mm_s(), 500);
    EXPECT_EQ(degraded.position().x_mm(), 1500);
    EXPECT_EQ(degraded.initial_energy_mj(), 10000);
    EXPECT_EQ(degraded.energy_capacity_mj(), 5000);
    EXPECT_EQ(degraded.energy_mj(), 5000);

    actions = sentinel::test::idle_actions(simulator);
    action(actions, "alpha").set_charge_location_id("charger");
    outcome = simulator.step(actions);
    const auto& charged = observation(outcome.observations, "alpha").self();
    EXPECT_EQ(charged.energy_capacity_mj(), 5000);
    EXPECT_EQ(charged.energy_mj(), 5000);
    EXPECT_EQ(simulator.summary().recharge_ticks(), 1U);
}

TEST(FailureRecovery, HidesRemoteTaskProgress) {
    auto value = scenario(1000);
    add_task(value, "work", "alpha", 1000, 5);
    sentinel::core::Simulator simulator(value);
    auto actions = sentinel::test::idle_actions(simulator);
    work(actions, "work");
    const auto outcome = simulator.step(actions);
    EXPECT_EQ(task(observation(outcome.observations, "alpha"), "work").progress_ticks(), 1U);
    EXPECT_EQ(task(observation(outcome.observations, "bravo"), "work").progress_ticks(), 0U);
}

TEST(FailureRecovery, RetainsGrantAndRejectsProposalComputedBeforeLinkCut) {
    auto value = scenario();
    add_task(value, "hold", "charlie", 10000, 10);
    auto* pass = value.mutable_world()->add_regions();
    pass->set_id("pass");
    pass->set_kind(sentinel::v1::REGION_KIND_CHOKEPOINT);
    pass->mutable_minimum()->set_x_mm(3000);
    pass->mutable_maximum()->set_x_mm(5000);
    pass->mutable_maximum()->set_y_mm(2000);
    pass->set_energy_multiplier_permille(1000);
    auto& cut = add_event(value, "cut", 1, sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED, "alpha");
    cut.set_text_value("bravo");
    cut.set_bool_value(true);
    sentinel::core::Simulator simulator(value);

    auto actions = sentinel::test::idle_actions(simulator);
    proposal(actions, "alpha", 1, 5, 1);
    auto outcome = simulator.step(actions);
    ASSERT_EQ(observation(outcome.observations, "alpha").committed_reservations_size(), 1);

    actions = sentinel::test::idle_actions(simulator);
    action(actions, "alpha").set_route_version(1);
    proposal(actions, "bravo", 2, 6, 1);
    outcome = simulator.step(actions);
    ASSERT_EQ(observation(outcome.observations, "alpha").committed_reservations_size(), 1);
    EXPECT_TRUE(observation(outcome.observations, "bravo").committed_reservations().empty());
}

TEST(FailureRecovery, OwnerReleaseAfterFailureReport) {
    auto value = scenario(1000, 3000, 5000);
    add_task(value, "complete", "alpha", 1000, 1);
    add_task(value, "orphan-a", "alpha", 1000, 5);
    add_task(value, "orphan-b", "alpha", 1000, 6);
    auto& block_ab = add_event(value, "block-ab", 1, sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                               "alpha");
    block_ab.set_text_value("bravo");
    block_ab.set_bool_value(true);
    auto& block_ac = add_event(value, "block-ac", 1, sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                               "alpha");
    block_ac.set_text_value("charlie");
    block_ac.set_bool_value(true);
    add_event(value, "disable", 2, sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE, "alpha");
    auto& heal_ab = add_event(value, "heal-ab", 4, sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                              "alpha");
    heal_ab.set_text_value("bravo");
    auto& heal_ac = add_event(value, "heal-ac", 4, sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                              "alpha");
    heal_ac.set_text_value("charlie");

    sentinel::core::Simulator simulator(value);
    auto actions = sentinel::test::idle_actions(simulator);
    work(actions, "complete");
    work(actions, "orphan-a");
    work(actions, "orphan-b");
    simulator.step(actions);

    actions = sentinel::test::idle_actions(simulator);
    work(actions, "orphan-a");
    work(actions, "orphan-b");
    simulator.step(actions);

    auto outcome = simulator.step(sentinel::test::idle_actions(simulator));
    auto current = observation(outcome.observations, "bravo");
    EXPECT_EQ(simulator.summary().completed_tasks(), 1U);
    EXPECT_EQ(simulator.summary().task_reassignments_size(), 2);
    EXPECT_EQ(current.failure_detections_size(), 0);
    EXPECT_EQ(current.available_tasks_size(), 0);
    EXPECT_EQ(task(current, "orphan-a").assigned_agent_id(), "alpha");
    EXPECT_EQ(task(current, "orphan-b").assigned_agent_id(), "alpha");
    EXPECT_EQ(task(current, "orphan-a").progress_ticks(), 0U);

    outcome = simulator.step(sentinel::test::idle_actions(simulator));
    EXPECT_EQ(observation(outcome.observations, "bravo").failure_detections_size(), 0);
    outcome = simulator.step(sentinel::test::idle_actions(simulator));
    EXPECT_EQ(observation(outcome.observations, "bravo").failure_detections_size(), 0);
    outcome = simulator.step(sentinel::test::idle_actions(simulator));
    current = observation(outcome.observations, "bravo");
    ASSERT_EQ(current.failure_detections_size(), 1);
    EXPECT_EQ(current.failure_detections(0).failure_tick(), 2U);
    EXPECT_EQ(current.failure_detections(0).detection_tick(), 6U);
    EXPECT_EQ(current.available_tasks_size(), 0);
    EXPECT_EQ(task(current, "orphan-a").assigned_agent_id(), "alpha");

    const auto old_epoch = current.allocation_epoch();
    actions = sentinel::test::idle_actions(simulator);
    action(actions, "bravo").add_failure_detections()->CopyFrom(current.failure_detections(0));
    outcome = simulator.step(actions);
    current = observation(outcome.observations, "bravo");
    EXPECT_EQ(current.allocation_epoch(), old_epoch + 1);
    ASSERT_EQ(current.available_tasks_size(), 2);
    EXPECT_TRUE(task(current, "orphan-a").assigned_agent_id().empty());
    EXPECT_TRUE(task(current, "orphan-b").assigned_agent_id().empty());
    EXPECT_EQ(task(current, "orphan-a").progress_ticks(), 0U);
    EXPECT_EQ(task(current, "orphan-b").progress_ticks(), 0U);
    const auto summary = simulator.summary();
    EXPECT_EQ(summary.completed_tasks(), 1U);
    EXPECT_EQ(summary.failure_detections_size(), 1);
    EXPECT_EQ(summary.task_reassignments_size(), 2);
    EXPECT_EQ(summary.missing_reassignments(), 2U);
    for (const auto& reassignment : summary.task_reassignments()) {
        EXPECT_EQ(reassignment.detector_agent_id(), "bravo");
        EXPECT_EQ(reassignment.detection_tick(), 6U);
        EXPECT_FALSE(reassignment.complete());
    }
}

TEST(FailureRecovery, UsesEarliestDetectionWhenReportsArriveTogether) {
    auto value = scenario();
    value.set_failure_detection_ticks(1);
    add_task(value, "orphan", "alpha", 10000, 5);
    auto& block = add_event(value, "block", 0, sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                            "alpha");
    block.set_text_value("bravo");
    block.set_bool_value(true);
    add_event(value, "disable", 0, sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE, "alpha");
    auto& heal = add_event(value, "heal", 2, sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED,
                           "alpha");
    heal.set_text_value("bravo");

    sentinel::core::Simulator simulator(value);
    auto outcome = simulator.step(sentinel::test::idle_actions(simulator));
    ASSERT_EQ(observation(outcome.observations, "charlie").failure_detections_size(), 1);
    const auto charlie = observation(outcome.observations, "charlie").failure_detections(0);
    EXPECT_EQ(charlie.detection_tick(), 1U);
    simulator.step(sentinel::test::idle_actions(simulator));
    outcome = simulator.step(sentinel::test::idle_actions(simulator));
    ASSERT_EQ(observation(outcome.observations, "bravo").failure_detections_size(), 1);
    const auto bravo = observation(outcome.observations, "bravo").failure_detections(0);
    EXPECT_EQ(bravo.detection_tick(), 3U);

    auto actions = sentinel::test::idle_actions(simulator);
    action(actions, "bravo").add_failure_detections()->CopyFrom(bravo);
    action(actions, "charlie").add_failure_detections()->CopyFrom(charlie);
    simulator.step(actions);

    const auto summary = simulator.summary();
    ASSERT_EQ(summary.task_reassignments_size(), 1);
    EXPECT_EQ(summary.task_reassignments(0).detector_agent_id(), "charlie");
    EXPECT_EQ(summary.task_reassignments(0).detection_tick(), 1U);
}

TEST(FailureRecovery, ReservationContention) {
    auto value = scenario(1000, 4000, 8000);
    add_task(value, "hold", "charlie", 10000, 5);
    auto* pass = value.mutable_world()->add_regions();
    pass->set_id("pass");
    pass->set_kind(sentinel::v1::REGION_KIND_CHOKEPOINT);
    pass->mutable_minimum()->set_x_mm(2000);
    pass->mutable_minimum()->set_y_mm(500);
    pass->mutable_maximum()->set_x_mm(3000);
    pass->mutable_maximum()->set_y_mm(1500);
    pass->set_energy_multiplier_permille(1000);

    sentinel::core::Simulator first(value);
    sentinel::core::Simulator second(value);
    auto first_actions = sentinel::test::idle_actions(first);
    auto second_actions = sentinel::test::idle_actions(second, true);
    for (auto* actions : {&first_actions, &second_actions}) {
        action(*actions, "alpha").set_route_version(1);
        action(*actions, "alpha").set_velocity_x_mm_s(1000);
        action(*actions, "bravo").set_route_version(1);
        action(*actions, "bravo").set_velocity_x_mm_s(-1000);
    }
    auto first_outcome = first.step(first_actions);
    auto second_outcome = second.step(second_actions);
    EXPECT_EQ(observation(first_outcome.observations, "alpha").self().position().x_mm(), 1000);
    EXPECT_EQ(observation(first_outcome.observations, "bravo").self().position().x_mm(), 4000);
    EXPECT_EQ(first.state_hash(), second.state_hash());

    first_actions = sentinel::test::idle_actions(first);
    second_actions = sentinel::test::idle_actions(second, true);
    for (auto* actions : {&first_actions, &second_actions}) {
        proposal(*actions, "alpha", 2, 3, 1);
        proposal(*actions, "bravo", 2, 3, 1);
    }
    first_outcome = first.step(first_actions);
    second_outcome = second.step(second_actions);
    const auto& first_grants = observation(first_outcome.observations, "alpha").committed_reservations();
    const auto& second_grants = observation(second_outcome.observations, "alpha").committed_reservations();
    ASSERT_EQ(first_grants.size(), 1);
    ASSERT_EQ(second_grants.size(), 1);
    EXPECT_EQ(first_grants.Get(0).agent_id(), "alpha");
    EXPECT_EQ(second_grants.Get(0).agent_id(), "alpha");
    EXPECT_TRUE(observation(first_outcome.observations, "bravo").committed_reservations().empty());
    EXPECT_EQ(first.summary().route_conflicts(), 1U);
    EXPECT_TRUE(first.summary().committed_reservations_never_overlap());
    EXPECT_EQ(first.summary().committed_reservation_overlap_violations(), 0U);
    EXPECT_EQ(first.state_hash(), second.state_hash());

    first_actions = sentinel::test::idle_actions(first);
    second_actions = sentinel::test::idle_actions(second, true);
    for (auto* actions : {&first_actions, &second_actions}) {
        action(*actions, "alpha").set_route_version(1);
        action(*actions, "alpha").set_velocity_x_mm_s(1000);
        action(*actions, "bravo").set_route_version(1);
        action(*actions, "bravo").set_velocity_x_mm_s(-1000);
        proposal(*actions, "bravo", 4, 5, 2);
    }
    first_outcome = first.step(first_actions);
    second_outcome = second.step(second_actions);
    const auto& alpha = observation(first_outcome.observations, "alpha");
    const auto& bravo = observation(first_outcome.observations, "bravo");
    EXPECT_EQ(alpha.self().position().x_mm(), 2000);
    EXPECT_EQ(bravo.self().position().x_mm(), 4000);
    ASSERT_EQ(alpha.committed_reservations_size(), 1);
    ASSERT_EQ(bravo.committed_reservations_size(), 1);
    EXPECT_EQ(alpha.committed_reservations(0).agent_id(), "alpha");
    EXPECT_EQ(bravo.committed_reservations(0).agent_id(), "bravo");
    EXPECT_EQ(first.summary().route_conflicts(), 1U);
    EXPECT_EQ(first.state_hash(), second.state_hash());
}

TEST(FailureRecoveryScenario, PairedFixturesDifferOnlyByPolicyAndIdentity) {
    const auto paired = [](const char* cbba_name, const char* nearest_name) {
        const auto root = std::filesystem::path(SENTINEL_SOURCE_DIR) / "scenarios";
        auto cbba = sentinel::core::load_scenario(root / cbba_name);
        auto nearest = sentinel::core::load_scenario(root / nearest_name);
        EXPECT_EQ(cbba.allocation_policy(), sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
        EXPECT_EQ(nearest.allocation_policy(), sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
        cbba.set_name("paired");
        nearest.set_name("paired");
        nearest.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
        EXPECT_EQ(sentinel::protocol::deterministic_bytes(cbba),
                  sentinel::protocol::deterministic_bytes(nearest));
    };
    paired("chokepoint_cbba.textproto", "chokepoint_nearest.textproto");
    paired("compound_failure_cbba.textproto", "compound_failure_nearest.textproto");
}
