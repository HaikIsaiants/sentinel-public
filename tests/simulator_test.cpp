#include "test_helpers.hpp"

#include <sentinel/core/scenario.hpp>

#include <gtest/gtest.h>

#include <algorithm>

TEST(Simulator, ExposesOnlyLocalVehicleAndAssignedTasks) {
    sentinel::core::Simulator simulator(sentinel::core::load_scenario(sentinel::test::nominal_scenario_path()));
    const auto observations = simulator.observe();
    ASSERT_EQ(observations.observations_size(), 4);
    for (const auto& envelope : observations.observations()) {
        ASSERT_TRUE(envelope.has_observation());
        EXPECT_EQ(envelope.recipient_id(), envelope.observation().self().id());
        for (const auto& task : envelope.observation().assigned_tasks()) {
            EXPECT_EQ(task.assigned_agent_id(), envelope.recipient_id());
        }
    }
}

TEST(Simulator, CanonicalizesActionOrder) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    sentinel::core::Simulator first(scenario);
    sentinel::core::Simulator second(scenario);
    const auto first_outcome = first.step(sentinel::test::idle_actions(first));
    const auto second_outcome = second.step(sentinel::test::idle_actions(second, true));
    EXPECT_EQ(first.state_hash(), second.state_hash());
    EXPECT_EQ(first_outcome.actions.actions(0).sender_id(), second_outcome.actions.actions(0).sender_id());
    EXPECT_LT(second_outcome.actions.actions(0).sender_id(), second_outcome.actions.actions(1).sender_id());
}

TEST(Simulator, AppliesSeededTapeEventsDeterministically) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    sentinel::core::Simulator first(scenario);
    sentinel::core::Simulator second(scenario);
    for (int index = 0; index < 6; ++index) {
        const auto first_outcome = first.step(sentinel::test::idle_actions(first));
        const auto second_outcome = second.step(sentinel::test::idle_actions(second));
        ASSERT_EQ(first_outcome.applied_events.size(), second_outcome.applied_events.size());
        EXPECT_EQ(first.state_hash(), second.state_hash());
    }
    const auto observations = first.observe();
    const auto position = std::find_if(observations.observations().begin(), observations.observations().end(),
                                       [](const auto& envelope) {
                                           return envelope.recipient_id() == "uav-charlie";
                                       });
    ASSERT_NE(position, observations.observations().end());
    EXPECT_EQ(position->observation().assigned_tasks_size(), 1);
}

TEST(Simulator, ReplayUsesRecordedRandomInput) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    sentinel::core::Simulator source(scenario);
    for (int index = 0; index < 3; ++index) {
        source.step(sentinel::test::idle_actions(source));
    }
    const auto recorded = source.step(sentinel::test::idle_actions(source));
    ASSERT_EQ(recorded.applied_events.size(), 1U);
    google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent> events;
    events.Add()->CopyFrom(recorded.applied_events.front());
    sentinel::core::Simulator replay(scenario);
    for (int index = 0; index < 3; ++index) {
        replay.step(sentinel::test::idle_actions(replay));
    }
    replay.replay_step(sentinel::test::idle_actions(replay), events);
    EXPECT_EQ(replay.state_hash(), source.state_hash());

    sentinel::core::Simulator tampered(scenario);
    for (int index = 0; index < 3; ++index) {
        tampered.step(sentinel::test::idle_actions(tampered));
    }
    events.Mutable(0)->set_rng_state_after(events.Get(0).rng_state_after() + 1);
    EXPECT_THROW(tampered.replay_step(sentinel::test::idle_actions(tampered), events), std::runtime_error);
}

TEST(Simulator, RejectsRestrictedMovementAndInsufficientEnergy) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    for (auto& vehicle : *scenario.mutable_vehicles()) {
        if (vehicle.id() == "uav-alpha") {
            vehicle.mutable_initial_position()->set_x_mm(500);
            vehicle.mutable_initial_position()->set_y_mm(6900);
        }
        if (vehicle.id() == "uav-charlie") {
            vehicle.set_initial_energy_mj(1);
        }
    }
    sentinel::core::Simulator simulator(scenario);
    auto actions = sentinel::test::idle_actions(simulator);
    for (auto& envelope : *actions.mutable_actions()) {
        if (envelope.sender_id() == "uav-alpha") {
            envelope.mutable_action()->set_velocity_y_mm_s(2000);
        }
        if (envelope.sender_id() == "uav-charlie") {
            envelope.mutable_action()->set_velocity_x_mm_s(2500);
        }
    }
    simulator.step(actions);
    const auto summary = simulator.summary();
    EXPECT_TRUE(summary.agent_energy_never_drops_below_zero());
    EXPECT_EQ(summary.agent_energy_below_zero_violations(), 0U);
    const auto observations = simulator.observe();
    for (const auto& envelope : observations.observations()) {
        if (envelope.recipient_id() == "uav-alpha") {
            EXPECT_EQ(envelope.observation().self().position().y_mm(), 6900);
        }
        if (envelope.recipient_id() == "uav-charlie") {
            EXPECT_EQ(envelope.observation().self().position().x_mm(), 8000);
        }
    }
}

TEST(Simulator, DebitsTerrainAdjustedEnergy) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    for (auto& vehicle : *scenario.mutable_vehicles()) {
        if (vehicle.id() == "ugv-bravo") {
            vehicle.mutable_initial_position()->set_x_mm(2600);
            vehicle.mutable_initial_position()->set_y_mm(4000);
        }
    }
    sentinel::core::Simulator simulator(scenario);
    auto actions = sentinel::test::idle_actions(simulator);
    for (auto& envelope : *actions.mutable_actions()) {
        if (envelope.sender_id() == "ugv-bravo") {
            envelope.mutable_action()->set_velocity_x_mm_s(1000);
        }
    }
    const auto outcome = simulator.step(actions);
    const auto position = std::find_if(outcome.observations.observations().begin(),
                                       outcome.observations.observations().end(), [](const auto& envelope) {
                                           return envelope.recipient_id() == "ugv-bravo";
                                       });
    ASSERT_NE(position, outcome.observations.observations().end());
    EXPECT_EQ(position->observation().self().position().x_mm(), 2700);
    EXPECT_EQ(position->observation().self().energy_mj(), 1499775);
}

TEST(Simulator, RejectsSegmentTunnelingAndChargesCrossedTerrain) {
    auto restricted_scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    auto restricted = std::find_if(restricted_scenario.mutable_world()->mutable_regions()->begin(),
                                   restricted_scenario.mutable_world()->mutable_regions()->end(), [](const auto& region) {
                                       return region.id() == "restricted-northwest";
                                   });
    ASSERT_NE(restricted, restricted_scenario.mutable_world()->mutable_regions()->end());
    restricted->mutable_minimum()->set_x_mm(1050);
    restricted->mutable_minimum()->set_y_mm(900);
    restricted->mutable_maximum()->set_x_mm(1150);
    restricted->mutable_maximum()->set_y_mm(1100);
    auto alpha = std::find_if(restricted_scenario.mutable_vehicles()->begin(),
                              restricted_scenario.mutable_vehicles()->end(), [](const auto& vehicle) {
                                  return vehicle.id() == "uav-alpha";
                              });
    ASSERT_NE(alpha, restricted_scenario.mutable_vehicles()->end());
    alpha->set_max_speed_mm_s(2000);
    sentinel::core::Simulator blocked(restricted_scenario);
    auto actions = sentinel::test::idle_actions(blocked);
    for (auto& envelope : *actions.mutable_actions()) {
        if (envelope.sender_id() == "uav-alpha") {
            envelope.mutable_action()->set_velocity_x_mm_s(2000);
        }
    }
    auto outcome = blocked.step(actions);
    auto observation = std::find_if(outcome.observations.observations().begin(),
                                    outcome.observations.observations().end(), [](const auto& envelope) {
                                        return envelope.recipient_id() == "uav-alpha";
                                    });
    ASSERT_NE(observation, outcome.observations.observations().end());
    EXPECT_EQ(observation->observation().self().position().x_mm(), 1000);

    auto terrain_scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    auto terrain = std::find_if(terrain_scenario.mutable_world()->mutable_regions()->begin(),
                                terrain_scenario.mutable_world()->mutable_regions()->end(), [](const auto& region) {
                                    return region.id() == "mud-lane";
                                });
    ASSERT_NE(terrain, terrain_scenario.mutable_world()->mutable_regions()->end());
    terrain->mutable_minimum()->set_x_mm(1050);
    terrain->mutable_minimum()->set_y_mm(3900);
    terrain->mutable_maximum()->set_x_mm(1150);
    terrain->mutable_maximum()->set_y_mm(4100);
    auto bravo = std::find_if(terrain_scenario.mutable_vehicles()->begin(), terrain_scenario.mutable_vehicles()->end(),
                              [](const auto& vehicle) {
                                  return vehicle.id() == "ugv-bravo";
                              });
    ASSERT_NE(bravo, terrain_scenario.mutable_vehicles()->end());
    bravo->set_max_speed_mm_s(2000);
    sentinel::core::Simulator charged(terrain_scenario);
    actions = sentinel::test::idle_actions(charged);
    for (auto& envelope : *actions.mutable_actions()) {
        if (envelope.sender_id() == "ugv-bravo") {
            envelope.mutable_action()->set_velocity_x_mm_s(2000);
        }
    }
    outcome = charged.step(actions);
    observation = std::find_if(outcome.observations.observations().begin(),
                               outcome.observations.observations().end(), [](const auto& envelope) {
                                   return envelope.recipient_id() == "ugv-bravo";
                               });
    ASSERT_NE(observation, outcome.observations.observations().end());
    EXPECT_EQ(observation->observation().self().position().x_mm(), 1200);
    EXPECT_EQ(observation->observation().self().energy_mj(), 1499550);
}

TEST(Simulator, CompletesValidTasksAndExpiresDeadlines) {
    auto completed_scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    completed_scenario.clear_events();
    for (auto& task : *completed_scenario.mutable_tasks()) {
        task.set_released(true);
        const auto vehicle = std::find_if(completed_scenario.vehicles().begin(), completed_scenario.vehicles().end(),
                                          [&task](const auto& current) {
                                              return current.id() == task.assigned_agent_id();
                                          });
        ASSERT_NE(vehicle, completed_scenario.vehicles().end());
        task.mutable_target()->CopyFrom(vehicle->initial_position());
    }
    sentinel::core::Simulator completed(completed_scenario);
    completed.step(sentinel::test::idle_actions(completed));
    EXPECT_TRUE(completed.finished());
    EXPECT_TRUE(completed.summary().success());
    EXPECT_EQ(completed.summary().completed_tasks(), 4U);

    sentinel::core::Simulator expired(sentinel::core::load_scenario(sentinel::test::nominal_scenario_path()));
    while (!expired.finished()) {
        expired.step(sentinel::test::idle_actions(expired));
    }
    EXPECT_FALSE(expired.summary().success());
    EXPECT_EQ(expired.summary().completed_tasks(), 0U);
    EXPECT_EQ(expired.tick(), 81U);
}

TEST(Simulator, RejectsInvalidActionEnvelopes) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    sentinel::core::Simulator wrong_schema(scenario);
    auto actions = sentinel::test::idle_actions(wrong_schema);
    actions.mutable_actions(0)->set_schema_version(2);
    EXPECT_THROW(wrong_schema.step(actions), std::invalid_argument);

    sentinel::core::Simulator stale(scenario);
    actions = sentinel::test::idle_actions(stale);
    actions.set_tick(1);
    EXPECT_THROW(stale.step(actions), std::invalid_argument);

    sentinel::core::Simulator missing(scenario);
    actions = sentinel::test::idle_actions(missing);
    actions.mutable_actions()->RemoveLast();
    EXPECT_THROW(missing.step(actions), std::invalid_argument);

    sentinel::core::Simulator wrong_sequence(scenario);
    actions = sentinel::test::idle_actions(wrong_sequence);
    actions.mutable_actions(0)->set_sequence(99);
    EXPECT_THROW(wrong_sequence.step(actions), std::invalid_argument);
}

TEST(Simulator, RoutesOnlyValidatedMessages) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    sentinel::core::Simulator routed(scenario);
    auto actions = sentinel::test::idle_actions(routed);
    auto* message = actions.mutable_actions(0)->mutable_action()->add_outgoing_messages();
    message->set_sender_id(actions.actions(0).sender_id());
    message->set_recipient_id("ugv-bravo");
    message->set_version(1);
    message->set_payload("allocation-v1");
    const auto outcome = routed.step(actions);
    for (const auto& envelope : outcome.observations.observations()) {
        const auto expected = envelope.recipient_id() == "ugv-bravo" ? 1 : 0;
        EXPECT_EQ(envelope.observation().delivered_messages_size(), expected);
    }
    sentinel::core::Simulator no_message(scenario);
    no_message.step(sentinel::test::idle_actions(no_message));
    EXPECT_NE(routed.state_hash(), no_message.state_hash());

    sentinel::core::Simulator wrong_sender(scenario);
    actions = sentinel::test::idle_actions(wrong_sender);
    message = actions.mutable_actions(0)->mutable_action()->add_outgoing_messages();
    message->set_sender_id("other-agent");
    message->set_recipient_id("ugv-bravo");
    message->set_version(1);
    EXPECT_THROW(wrong_sender.step(actions), std::invalid_argument);

    sentinel::core::Simulator unknown_recipient(scenario);
    actions = sentinel::test::idle_actions(unknown_recipient);
    message = actions.mutable_actions(0)->mutable_action()->add_outgoing_messages();
    message->set_sender_id(actions.actions(0).sender_id());
    message->set_recipient_id("missing-agent");
    message->set_version(1);
    EXPECT_THROW(unknown_recipient.step(actions), std::invalid_argument);

    sentinel::core::Simulator zero_version(scenario);
    actions = sentinel::test::idle_actions(zero_version);
    message = actions.mutable_actions(0)->mutable_action()->add_outgoing_messages();
    message->set_sender_id(actions.actions(0).sender_id());
    message->set_recipient_id("ugv-bravo");
    EXPECT_THROW(zero_version.step(actions), std::invalid_argument);
}
