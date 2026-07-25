#include "test_helpers.hpp"

#include <sentinel/core/scenario.hpp>

#include <gtest/gtest.h>

#include <algorithm>

TEST(Scenario, LoadsAndNormalizesBasicMission) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    EXPECT_EQ(scenario.schema_version(), 1U);
    EXPECT_EQ(scenario.vehicles_size(), 4);
    EXPECT_EQ(scenario.tasks_size(), 4);
    EXPECT_EQ(scenario.events_size(), 4);
    EXPECT_EQ(scenario.failure_detection_ticks(), 5U);
    EXPECT_LT(scenario.vehicles(0).id(), scenario.vehicles(1).id());
    EXPECT_LT(scenario.tasks(0).id(), scenario.tasks(1).id());
}

TEST(Scenario, AcceptsVehicleDegradationEvents) {
    for (const auto kind : {sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE,
                            sentinel::v1::TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE}) {
        auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
        auto* event = scenario.add_events();
        event->set_id(kind == sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE ? "speed-drop" : "endurance-drop");
        event->set_tick(1);
        event->set_kind(kind);
        event->set_target_id(scenario.vehicles(0).id());
        event->set_value_min(350);
        event->set_value_max(700);
        event->set_rng_stream(event->id());
        EXPECT_NO_THROW(sentinel::core::validate_scenario(scenario));
    }
}

TEST(Scenario, RejectsInvalidFailureDetectionAndDegradationEvents) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.set_failure_detection_ticks(scenario.max_ticks() + 1);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    const auto invalid = [](auto change) {
        auto value = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
        auto* event = value.add_events();
        event->set_id("speed-drop");
        event->set_tick(1);
        event->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE);
        event->set_target_id(value.vehicles(0).id());
        event->set_value_min(350);
        event->set_value_max(650);
        event->set_rng_stream("speed-drop");
        change(*event);
        EXPECT_THROW(sentinel::core::validate_scenario(value), std::invalid_argument);
    };
    invalid([](auto& event) { event.set_target_id("missing-agent"); });
    invalid([](auto& event) { event.set_value_min(0); });
    invalid([](auto& event) { event.set_value_max(1001); });
    invalid([](auto& event) {
        event.set_value_min(700);
        event.set_value_max(600);
    });
    invalid([](auto& event) { event.clear_rng_stream(); });
}

TEST(Scenario, HashIgnoresInputOrdering) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    auto reversed = scenario;
    std::reverse(reversed.mutable_vehicles()->begin(), reversed.mutable_vehicles()->end());
    std::reverse(reversed.mutable_tasks()->begin(), reversed.mutable_tasks()->end());
    std::reverse(reversed.mutable_events()->begin(), reversed.mutable_events()->end());
    std::reverse(reversed.mutable_world()->mutable_regions()->begin(),
                 reversed.mutable_world()->mutable_regions()->end());
    EXPECT_EQ(sentinel::core::scenario_hash(scenario), sentinel::core::scenario_hash(reversed));
}

TEST(Scenario, RejectsInvalidAgentCountAndAssignments) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    while (scenario.vehicles_size() > 2) {
        scenario.mutable_vehicles()->RemoveLast();
    }
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.mutable_tasks(0)->set_assigned_agent_id("missing-agent");
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
}

TEST(Scenario, RejectsIncapableAndOverloadedAssignments) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    auto task = std::find_if(scenario.mutable_tasks()->begin(), scenario.mutable_tasks()->end(), [](const auto& value) {
        return value.id() == "search-sector-a";
    });
    ASSERT_NE(task, scenario.mutable_tasks()->end());
    task->set_required_capability(sentinel::v1::CAPABILITY_DELIVERY);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    task = std::find_if(scenario.mutable_tasks()->begin(), scenario.mutable_tasks()->end(), [](const auto& value) {
        return value.id() == "search-sector-a";
    });
    ASSERT_NE(task, scenario.mutable_tasks()->end());
    task->set_payload_required_grams(1001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
}

TEST(Scenario, RejectsArithmeticOverflowInputs) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.set_step_ms(60001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.mutable_world()->set_width_mm(1000000001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.mutable_vehicles(0)->set_max_speed_mm_s(1000001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    auto event = std::find_if(scenario.mutable_events()->begin(), scenario.mutable_events()->end(), [](const auto& value) {
        return value.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA;
    });
    ASSERT_NE(event, scenario.mutable_events()->end());
    event->set_value_min(1000000000000000);
    event->set_value_max(1000000000000000);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
}

TEST(Scenario, RejectsUnknownEnumsAndEmptyTerrainLabels) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.mutable_world()->mutable_regions(0)->set_kind(static_cast<sentinel::v1::RegionKind>(99));
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.mutable_events(0)->set_kind(static_cast<sentinel::v1::TapeEventKind>(99));
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    auto& task = *scenario.mutable_tasks(0);
    auto vehicle = std::find_if(scenario.mutable_vehicles()->begin(), scenario.mutable_vehicles()->end(),
                                [&task](const auto& value) {
                                    return value.id() == task.assigned_agent_id();
                                });
    ASSERT_NE(vehicle, scenario.mutable_vehicles()->end());
    const auto unknown = static_cast<sentinel::v1::Capability>(99);
    task.set_required_capability(unknown);
    vehicle->add_capabilities(unknown);
    sentinel::core::normalize_scenario(scenario);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    auto terrain = std::find_if(scenario.mutable_world()->mutable_regions()->begin(),
                                scenario.mutable_world()->mutable_regions()->end(), [](const auto& region) {
                                    return region.kind() == sentinel::v1::REGION_KIND_TERRAIN;
                                });
    ASSERT_NE(terrain, scenario.mutable_world()->mutable_regions()->end());
    terrain->clear_terrain();
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);

    scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    scenario.mutable_vehicles(0)->set_terrain_access(0, "");
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
}
