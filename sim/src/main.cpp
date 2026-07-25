#include <sentinel/core/event_log.hpp>
#include <sentinel/core/scenario.hpp>
#include <sentinel/core/simulator.hpp>
#include <sentinel/protocol/arguments.hpp>
#include <sentinel/protocol/framing.hpp>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

std::filesystem::path option(int argc, char** argv, std::string_view name) {
    return sentinel::protocol::required_option(argc, argv, name, 2);
}

void run(int argc, char** argv) {
    const auto scenario_path = option(argc, argv, "--scenario");
    const auto log_path = option(argc, argv, "--log");
    const auto summary_path = option(argc, argv, "--summary");
    if (!log_path.parent_path().empty()) {
        std::filesystem::create_directories(log_path.parent_path());
    }
    sentinel::protocol::configure_binary_stdio();
    auto scenario = sentinel::core::load_scenario(scenario_path);
    sentinel::core::Simulator simulator(scenario);
    sentinel::core::EventLogWriter event_log(log_path, scenario);
    sentinel::v1::SimulationFrame output;
    // Prime the lockstep exchange with the initial observation batch.
    output.mutable_observations()->CopyFrom(simulator.observe());
    sentinel::protocol::write_frame(std::cout, output);
    while (!simulator.finished()) {
        sentinel::v1::SimulationFrame input;
        if (!sentinel::protocol::read_frame(std::cin, input) || !input.has_actions()) {
            throw std::runtime_error("simulation action stream ended early");
        }
        const auto outcome = simulator.step(input.actions());
        event_log.append(simulator.tick(), outcome.actions, outcome.applied_events, outcome.network_outcomes,
                         simulator.state_hash());
        output.Clear();
        output.mutable_observations()->CopyFrom(outcome.observations);
        sentinel::protocol::write_frame(std::cout, output);
    }
    const auto summary = simulator.summary();
    event_log.finish(summary);
    sentinel::core::write_summary_json(summary_path, summary);
}

void replay(int argc, char** argv) {
    const auto scenario = sentinel::core::load_scenario(option(argc, argv, "--scenario"));
    const auto summary = sentinel::core::replay_event_log(scenario, option(argc, argv, "--log"));
    sentinel::core::write_summary_json(option(argc, argv, "--summary"), summary);
    std::cout << summary.terminal_hash() << '\n';
}

void export_trace(int argc, char** argv) {
    const auto scenario = sentinel::core::load_scenario(option(argc, argv, "--scenario"));
    const auto summary = sentinel::core::export_replay_trace(scenario, option(argc, argv, "--log"),
                                                              option(argc, argv, "--trace"));
    std::cout << summary.terminal_hash() << '\n';
}

}

int main(int argc, char** argv) {
    try {
        if (argc < 2) {
            throw std::invalid_argument("missing command");
        }
        const std::string command = argv[1];
        if (command == "run") {
            run(argc, argv);
        } else if (command == "replay") {
            replay(argc, argv);
        } else if (command == "export") {
            export_trace(argc, argv);
        } else {
            throw std::invalid_argument("unknown command");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
