#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string_view>

namespace {

sentinel::v1::Scenario scenario(sentinel::v1::AllocationPolicy policy) {
    sentinel::v1::Scenario value;
    value.set_schema_version(1);
    value.set_name("mission-execution-test");
    value.set_seed(7);
    value.set_step_ms(100);
    value.set_max_ticks(20);
    value.set_network_profile("perfect");
    value.set_allocation_policy(policy);
    auto* world = value.mutable_world();
    world->set_width_mm(10000);
    world->set_height_mm(10000);
    world->set_grid_cell_mm(100);
    world->set_map_version(1);
    auto* region = world->add_regions();
    region->set_id("gate");
    region->set_kind(sentinel::v1::REGION_KIND_OBSTACLE);
    region->mutable_minimum()->set_x_mm(6000);
    region->mutable_minimum()->set_y_mm(6000);
    region->mutable_maximum()->set_x_mm(7000);
    region->mutable_maximum()->set_y_mm(7000);
    region->set_energy_multiplier_permille(1000);
    auto* charger = world->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(1000);
    charger->mutable_position()->set_y_mm(1000);
    charger->set_charge_mj_per_tick(2000);

    const auto add_vehicle = [&value](const char* id, const char* kind, std::int64_t x_mm,
                                      std::int64_t payload_grams,
                                      std::initializer_list<sentinel::v1::Capability> capabilities) {
        auto* vehicle = value.add_vehicles();
        vehicle->set_id(id);
        vehicle->set_kind(kind);
        vehicle->mutable_initial_position()->set_x_mm(x_mm);
        vehicle->mutable_initial_position()->set_y_mm(1000);
        vehicle->set_max_speed_mm_s(1000);
        vehicle->set_initial_energy_mj(10000);
        vehicle->set_energy_cost_mj_per_meter(100);
        vehicle->set_payload_grams(payload_grams);
        for (const auto capability : capabilities) {
            vehicle->add_capabilities(capability);
        }
        vehicle->add_terrain_access("plain");
    };
    add_vehicle("alpha", "uav", 1000, 0,
                {sentinel::v1::CAPABILITY_SEARCH});
    add_vehicle("bravo", "ugv", 2000, 0,
                {sentinel::v1::CAPABILITY_SEARCH, sentinel::v1::CAPABILITY_INSPECTION});
    add_vehicle("charlie", "rover", 3000, 500,
                {sentinel::v1::CAPABILITY_RELAY, sentinel::v1::CAPABILITY_DELIVERY});
    return value;
}

sentinel::v1::TaskSpec& add_task(sentinel::v1::Scenario& value, const char* id, const char* kind,
                                 sentinel::v1::Capability capability, const char* agent_id,
                                 std::int64_t x_mm, std::int64_t payload_grams = 0) {
    auto* task = value.add_tasks();
    task->set_id(id);
    task->set_kind(kind);
    task->mutable_target()->set_x_mm(x_mm);
    task->mutable_target()->set_y_mm(1000);
    task->set_required_capability(capability);
    task->set_payload_required_grams(payload_grams);
    task->set_deadline_tick(15);
    task->set_completion_radius_mm(0);
    if (agent_id) {
        task->set_assigned_agent_id(agent_id);
    }
    task->set_released(true);
    task->set_service_ticks(1);
    task->set_service_energy_mj_per_tick(10);
    return *task;
}

sentinel::v1::AgentAction& action(sentinel::v1::ActionBatch& actions, std::string_view agent_id) {
    const auto position = std::find_if(actions.mutable_actions()->begin(), actions.mutable_actions()->end(),
                                       [agent_id](const auto& envelope) {
                                           return envelope.sender_id() == agent_id;
                                       });
    if (position == actions.mutable_actions()->end()) {
        throw std::logic_error("missing action");
    }
    return *position->mutable_action();
}

const sentinel::v1::AgentObservation& observation(const sentinel::v1::ObservationBatch& observations,
                                                   std::string_view agent_id) {
    const auto position = std::find_if(observations.observations().begin(), observations.observations().end(),
                                       [agent_id](const auto& envelope) {
                                           return envelope.recipient_id() == agent_id;
                                       });
    if (position == observations.observations().end()) {
        throw std::logic_error("missing observation");
    }
    return position->observation();
}

const sentinel::v1::TaskState& assigned_task(const sentinel::v1::AgentObservation& observation,
                                              std::string_view task_id) {
    const auto position = std::find_if(observation.assigned_tasks().begin(), observation.assigned_tasks().end(),
                                       [task_id](const auto& task) {
                                           return task.id() == task_id;
                                       });
    if (position == observation.assigned_tasks().end()) {
        throw std::logic_error("missing assigned task");
    }
    return *position;
}

const sentinel::v1::TaskState& available_task(const sentinel::v1::AgentObservation& observation,
                                               std::string_view task_id) {
    const auto position = std::find_if(observation.available_tasks().begin(), observation.available_tasks().end(),
                                       [task_id](const auto& task) {
                                           return task.id() == task_id;
                                       });
    if (position == observation.available_tasks().end()) {
        throw std::logic_error("missing available task");
    }
    return *position;
}

void working(sentinel::v1::ActionBatch& actions, std::string_view agent_id, const char* task_id) {
    auto* report = action(actions, agent_id).add_task_reports();
    report->set_task_id(task_id);
    report->set_kind(sentinel::v1::TASK_REPORT_KIND_WORKING);
}

void commit(sentinel::v1::ActionBatch& actions, std::string_view agent_id, const char* task_id,
            std::uint64_t version, std::int64_t distance_mm) {
    auto* value = action(actions, agent_id).add_allocation_commits();
    value->set_epoch(1);
    value->set_version(version);
    value->set_task_id(task_id);
    value->set_agent_id(agent_id.data(), agent_id.size());
    value->set_distance_mm(distance_mm);
}

void add_energy_event(sentinel::v1::Scenario& value, const char* id, const char* vehicle_id) {
    auto* event = value.add_events();
    event->set_id(id);
    event->set_kind(sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA);
    event->set_target_id(vehicle_id);
    event->set_value_min(-3000);
    event->set_value_max(-3000);
    event->set_rng_stream(id);
}

}

TEST(MissionExecution, ServiceAndDeliveryAccounting) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SCRIPTED);
    add_task(value, "search", "search", sentinel::v1::CAPABILITY_SEARCH, "alpha", 1000);
    add_task(value, "inspection", "inspection", sentinel::v1::CAPABILITY_INSPECTION, "bravo", 2000);
    add_task(value, "relay", "relay", sentinel::v1::CAPABILITY_RELAY, "charlie", 3000);
    add_task(value, "delivery", "delivery", sentinel::v1::CAPABILITY_DELIVERY, "charlie", 3000, 200);
    sentinel::core::Simulator simulator(value);

    const auto idle = simulator.step(sentinel::test::idle_actions(simulator));
    EXPECT_EQ(assigned_task(observation(idle.observations, "alpha"), "search").progress_ticks(), 0U);
    EXPECT_EQ(assigned_task(observation(idle.observations, "bravo"), "inspection").progress_ticks(), 0U);
    EXPECT_EQ(assigned_task(observation(idle.observations, "charlie"), "relay").progress_ticks(), 0U);
    EXPECT_EQ(assigned_task(observation(idle.observations, "charlie"), "delivery").progress_ticks(), 0U);
    EXPECT_EQ(simulator.summary().completed_tasks(), 0U);

    auto actions = sentinel::test::idle_actions(simulator);
    working(actions, "alpha", "search");
    working(actions, "bravo", "inspection");
    working(actions, "charlie", "relay");
    working(actions, "charlie", "delivery");
    const auto completed = simulator.step(actions);
    EXPECT_TRUE(simulator.finished());
    EXPECT_EQ(simulator.summary().completed_tasks(), 4U);
    EXPECT_EQ(observation(completed.observations, "charlie").self().payload_grams(), 300);
}

TEST(MissionExecution, IncompatibleCommitDoesNotChangeOwnership) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    add_task(value, "search", "search", sentinel::v1::CAPABILITY_SEARCH, nullptr, 1000);
    sentinel::core::Simulator simulator(value);
    auto actions = sentinel::test::idle_actions(simulator);
    commit(actions, "charlie", "search", 1, 2000);
    const auto outcome = simulator.step(actions);
    const auto summary = simulator.summary();
    EXPECT_EQ(summary.rejected_commits(), 1U);
    EXPECT_TRUE(summary.incapable_agents_never_commit_tasks());
    EXPECT_EQ(summary.incapable_agent_commit_violations(), 0U);
    for (const auto& envelope : outcome.observations.observations()) {
        const auto& task = available_task(envelope.observation(), "search");
        EXPECT_TRUE(task.assigned_agent_id().empty());
        EXPECT_EQ(task.allocation_epoch(), 0U);
        EXPECT_EQ(task.allocation_version(), 0U);
    }
}

TEST(MissionExecution, ChargingRequiresLocationAndCapsAtInitialEnergy) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SCRIPTED);
    add_task(value, "hold", "search", sentinel::v1::CAPABILITY_SEARCH, "alpha", 1000);
    add_energy_event(value, "energy-alpha", "alpha");
    add_energy_event(value, "energy-bravo", "bravo");
    sentinel::core::Simulator simulator(value);

    auto actions = sentinel::test::idle_actions(simulator);
    action(actions, "alpha").set_charge_location_id("charger");
    action(actions, "bravo").set_charge_location_id("charger");
    auto outcome = simulator.step(actions);
    EXPECT_EQ(observation(outcome.observations, "alpha").self().energy_mj(), 9000);
    EXPECT_EQ(observation(outcome.observations, "bravo").self().energy_mj(), 7000);

    actions = sentinel::test::idle_actions(simulator);
    action(actions, "alpha").set_charge_location_id("charger");
    outcome = simulator.step(actions);
    EXPECT_EQ(observation(outcome.observations, "alpha").self().energy_mj(), 10000);

    actions = sentinel::test::idle_actions(simulator);
    action(actions, "alpha").set_charge_location_id("charger");
    outcome = simulator.step(actions);
    EXPECT_EQ(observation(outcome.observations, "alpha").self().energy_mj(), 10000);
    EXPECT_EQ(simulator.summary().recharge_ticks(), 3U);
}

TEST(MissionExecution, RegionClosureReplaysWithMapVersionAndStateHash) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_SCRIPTED);
    add_task(value, "hold", "search", sentinel::v1::CAPABILITY_SEARCH, "alpha", 1000);
    auto* event = value.add_events();
    event->set_id("close-gate");
    event->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED);
    event->set_target_id("gate");
    event->set_bool_value(true);

    sentinel::core::Simulator source(value);
    EXPECT_EQ(source.scenario().world().map_version(), 1U);
    const auto outcome = source.step(sentinel::test::idle_actions(source));
    ASSERT_EQ(outcome.applied_events.size(), 1U);
    EXPECT_EQ(source.scenario().world().map_version(), 2U);
    EXPECT_TRUE(source.scenario().world().regions(0).closed());

    google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent> recorded;
    recorded.Add()->CopyFrom(outcome.applied_events.front());
    sentinel::core::Simulator replay(value);
    replay.replay_step(sentinel::test::idle_actions(replay), recorded);
    EXPECT_EQ(replay.scenario().world().map_version(), 2U);
    EXPECT_EQ(replay.state_hash(), source.state_hash());

    value.clear_events();
    sentinel::core::Simulator unchanged(value);
    unchanged.step(sentinel::test::idle_actions(unchanged));
    EXPECT_EQ(unchanged.scenario().world().map_version(), 1U);
    EXPECT_NE(unchanged.state_hash(), source.state_hash());
}

TEST(MissionExecution, CompletedTaskCannotBeReassigned) {
    auto value = scenario(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    add_task(value, "target", "search", sentinel::v1::CAPABILITY_SEARCH, nullptr, 1000);
    add_task(value, "guard", "search", sentinel::v1::CAPABILITY_SEARCH, nullptr, 1000);
    auto* event = value.add_events();
    event->set_id("disable-alpha");
    event->set_tick(1);
    event->set_kind(sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE);
    event->set_target_id("alpha");
    sentinel::core::Simulator attempted(value);
    sentinel::core::Simulator control(value);

    auto complete = [](sentinel::core::Simulator& simulator) {
        auto actions = sentinel::test::idle_actions(simulator);
        commit(actions, "alpha", "target", 1, 0);
        working(actions, "alpha", "target");
        simulator.step(actions);
    };
    complete(attempted);
    complete(control);
    ASSERT_EQ(attempted.summary().completed_tasks(), 1U);

    auto actions = sentinel::test::idle_actions(attempted);
    commit(actions, "bravo", "target", 2, 1000);
    attempted.step(actions);
    actions = sentinel::test::idle_actions(control);
    commit(actions, "bravo", "guard", 2, 1001);
    control.step(actions);

    EXPECT_EQ(attempted.summary().completed_tasks(), 1U);
    EXPECT_EQ(attempted.summary().rejected_commits(), 1U);
    EXPECT_TRUE(attempted.summary().completed_tasks_are_never_reassigned());
    EXPECT_EQ(attempted.summary().completed_task_reassignment_violations(), 0U);
    EXPECT_EQ(attempted.state_hash(), control.state_hash());
}
