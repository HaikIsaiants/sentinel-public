#include "test_helpers.hpp"

#include <sentinel/core/event_log.hpp>
#include <sentinel/core/scenario.hpp>
#include <sentinel/protocol/framing.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <vector>

TEST(EventLog, ReplaysEveryTickToTheSameTerminalHash) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    const auto path = std::filesystem::temp_directory_path() / "sentinel-nominal-replay.events.pb";
    sentinel::core::Simulator simulator(scenario);
    {
        sentinel::core::EventLogWriter writer(path, scenario);
        while (!simulator.finished()) {
            const auto outcome = simulator.step(sentinel::test::idle_actions(simulator));
            writer.append(simulator.tick(), outcome.actions, outcome.applied_events, outcome.network_outcomes,
                          simulator.state_hash());
        }
        writer.finish(simulator.summary());
    }
    const auto replay = sentinel::core::replay_event_log(scenario, path);
    EXPECT_EQ(replay.terminal_hash(), simulator.summary().terminal_hash());
    EXPECT_EQ(replay.tick_hashes_size(), simulator.summary().tick_hashes_size());
    std::filesystem::remove(path);
}

TEST(EventLog, CapturesAuthoritativeReplayObservations) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    const auto path = std::filesystem::temp_directory_path() / "sentinel-trace-observations.events.pb";
    sentinel::core::Simulator simulator(scenario);
    std::vector<sentinel::v1::ObservationBatch> expected{simulator.observe()};
    {
        sentinel::core::EventLogWriter writer(path, scenario);
        while (!simulator.finished()) {
            const auto outcome = simulator.step(sentinel::test::idle_actions(simulator));
            expected.push_back(outcome.observations);
            writer.append(simulator.tick(), outcome.actions, outcome.applied_events, outcome.network_outcomes,
                          simulator.state_hash());
        }
        writer.finish(simulator.summary());
    }
    std::vector<sentinel::v1::ObservationBatch> actual;
    const auto replay = sentinel::core::replay_event_log(scenario, path, &actual);
    ASSERT_EQ(actual.size(), expected.size());
    EXPECT_EQ(actual.size(), replay.ticks() + 1);
    for (std::size_t i = 0; i < actual.size(); ++i) {
        EXPECT_EQ(actual[i].tick(), i);
        EXPECT_EQ(sentinel::protocol::deterministic_bytes(actual[i]),
                  sentinel::protocol::deterministic_bytes(expected[i]));
    }
    ASSERT_FALSE(actual.empty());
    EXPECT_TRUE(actual.back().finished());
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(actual.back().summary()),
              sentinel::protocol::deterministic_bytes(replay));
    std::filesystem::remove(path);
}

TEST(EventLog, RejectsScenarioMismatch) {
    auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    const auto path = std::filesystem::temp_directory_path() / "sentinel-nominal-mismatch.events.pb";
    sentinel::core::Simulator simulator(scenario);
    {
        sentinel::core::EventLogWriter writer(path, scenario);
        const auto outcome = simulator.step(sentinel::test::idle_actions(simulator));
        writer.append(simulator.tick(), outcome.actions, outcome.applied_events, outcome.network_outcomes,
                      simulator.state_hash());
        writer.finish(simulator.summary());
    }
    scenario.set_seed(scenario.seed() + 1);
    EXPECT_THROW(sentinel::core::replay_event_log(scenario, path), std::runtime_error);
    std::filesystem::remove(path);
}

TEST(EventLog, RejectsMissingTerminalEvent) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    const auto path = std::filesystem::temp_directory_path() / "sentinel-nominal-unfinished.events.pb";
    sentinel::core::Simulator simulator(scenario);
    {
        sentinel::core::EventLogWriter writer(path, scenario);
        while (!simulator.finished()) {
            const auto outcome = simulator.step(sentinel::test::idle_actions(simulator));
            writer.append(simulator.tick(), outcome.actions, outcome.applied_events, outcome.network_outcomes,
                          simulator.state_hash());
        }
    }
    EXPECT_THROW(sentinel::core::replay_event_log(scenario, path), std::runtime_error);
    std::filesystem::remove(path);
}
