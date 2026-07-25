#include "test_helpers.hpp"

#include <sentinel/core/event_log.hpp>
#include <sentinel/core/scenario.hpp>
#include <sentinel/protocol/framing.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

namespace {

std::vector<sentinel::v1::ObservationBatch> write_log(const sentinel::v1::Scenario& scenario,
                                                      const std::filesystem::path& path, bool finish) {
    sentinel::core::Simulator simulator(scenario);
    std::vector<sentinel::v1::ObservationBatch> observations{simulator.observe()};
    sentinel::core::EventLogWriter writer(path, scenario);
    while (!simulator.finished()) {
        const auto outcome = simulator.step(sentinel::test::idle_actions(simulator));
        observations.push_back(outcome.observations);
        writer.append(simulator.tick(), outcome.actions, outcome.applied_events, outcome.network_outcomes,
                      simulator.state_hash());
    }
    if (finish) {
        writer.finish(simulator.summary());
    }
    return observations;
}

std::vector<sentinel::v1::SimulationFrame> read_trace(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("failed to open trace");
    }
    std::vector<sentinel::v1::SimulationFrame> frames;
    sentinel::v1::SimulationFrame frame;
    while (sentinel::protocol::read_frame(input, frame)) {
        frames.push_back(frame);
        frame.Clear();
    }
    return frames;
}

std::string file_bytes(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void remove_artifact(const std::filesystem::path& path) {
    std::error_code error;
    std::filesystem::remove(path, error);
    auto temporary = path;
    temporary += ".tmp";
    std::filesystem::remove(temporary, error);
}

}

TEST(TraceExport, ExportIsCompleteAndDeterministic) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    const auto directory = std::filesystem::temp_directory_path();
    const auto log = directory / "sentinel-trace-trace.events.pb";
    const auto first = directory / "sentinel-trace-trace-a.pb";
    const auto second = directory / "sentinel-trace-trace-b.pb";
    const auto summary_path = directory / "sentinel-trace-trace-summary.json";
    remove_artifact(log);
    remove_artifact(first);
    remove_artifact(second);
    remove_artifact(summary_path);
    const auto expected = write_log(scenario, log, true);
    const auto first_summary = sentinel::core::export_replay_trace(scenario, log, first);
    const auto second_summary = sentinel::core::export_replay_trace(scenario, log, second);
    const auto frames = read_trace(first);
    ASSERT_EQ(frames.size(), expected.size());
    EXPECT_EQ(frames.size(), first_summary.ticks() + 1);
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(first_summary),
              sentinel::protocol::deterministic_bytes(second_summary));
    EXPECT_TRUE(first_summary.agent_energy_never_drops_below_zero());
    EXPECT_TRUE(first_summary.committed_reservations_never_overlap());
    EXPECT_TRUE(first_summary.completed_tasks_are_never_reassigned());
    EXPECT_TRUE(first_summary.incapable_agents_never_commit_tasks());
    EXPECT_EQ(first_summary.agent_energy_below_zero_violations(), 0U);
    EXPECT_EQ(first_summary.committed_reservation_overlap_violations(), 0U);
    EXPECT_EQ(first_summary.completed_task_reassignment_violations(), 0U);
    EXPECT_EQ(first_summary.incapable_agent_commit_violations(), 0U);
    sentinel::core::write_summary_json(summary_path, first_summary);
    const auto summary_json = file_bytes(summary_path);
    EXPECT_NE(summary_json.find("\"agent_energy_never_drops_below_zero\": true"), std::string::npos);
    EXPECT_NE(summary_json.find("\"committed_reservations_never_overlap\": true"), std::string::npos);
    EXPECT_NE(summary_json.find("\"completed_tasks_are_never_reassigned\": true"), std::string::npos);
    EXPECT_NE(summary_json.find("\"incapable_agents_never_commit_tasks\": true"), std::string::npos);
    EXPECT_EQ(file_bytes(first), file_bytes(second));
    for (std::size_t i = 0; i < frames.size(); ++i) {
        ASSERT_TRUE(frames[i].has_observations());
        EXPECT_EQ(frames[i].observations().tick(), i);
        EXPECT_EQ(frames[i].observations().observations_size(), scenario.vehicles_size());
        EXPECT_EQ(sentinel::protocol::deterministic_bytes(frames[i].observations()),
                  sentinel::protocol::deterministic_bytes(expected[i]));
    }
    ASSERT_TRUE(frames.back().observations().finished());
    EXPECT_EQ(frames.back().observations().summary().terminal_hash(), first_summary.terminal_hash());
    ASSERT_GT(frames.size(), 13U);
    const auto& disabled = frames[13].observations();
    const auto position = std::find_if(disabled.observations().begin(), disabled.observations().end(),
                                       [](const auto& envelope) {
                                           return envelope.observation().self().id() == "ugv-delta";
                                       });
    ASSERT_NE(position, disabled.observations().end());
    EXPECT_FALSE(position->observation().self().active());
    remove_artifact(log);
    remove_artifact(first);
    remove_artifact(second);
    remove_artifact(summary_path);
}

TEST(TraceExport, InvalidReplayLeavesNoTrace) {
    const auto scenario = sentinel::core::load_scenario(sentinel::test::nominal_scenario_path());
    const auto directory = std::filesystem::temp_directory_path();
    const auto log = directory / "sentinel-trace-invalid.events.pb";
    const auto trace = directory / "sentinel-trace-invalid.trace.pb";
    remove_artifact(log);
    remove_artifact(trace);
    write_log(scenario, log, false);
    EXPECT_THROW(sentinel::core::export_replay_trace(scenario, log, trace), std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(trace));
    auto temporary = trace;
    temporary += ".tmp";
    EXPECT_FALSE(std::filesystem::exists(temporary));
    remove_artifact(log);
    remove_artifact(trace);
}
