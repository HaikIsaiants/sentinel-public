#pragma once

#include <sentinel/v1/sentinel.pb.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace sentinel::core {

class EventLogWriter {
public:
    EventLogWriter(const std::filesystem::path& path, const sentinel::v1::Scenario& scenario);
    void append(std::uint64_t tick, const sentinel::v1::ActionBatch& actions,
                const std::vector<sentinel::v1::AppliedEvent>& applied_events,
                const std::vector<sentinel::v1::NetworkOutcome>& network_outcomes,
                const std::string& state_hash);
    void finish(const sentinel::v1::SimulationSummary& summary);

private:
    std::ofstream output_;
    std::uint64_t sequence_{};
    bool finished_{};
};

sentinel::v1::SimulationSummary replay_event_log(const sentinel::v1::Scenario& scenario,
                                                  const std::filesystem::path& path,
                                                  std::vector<sentinel::v1::ObservationBatch>* observations = nullptr);
sentinel::v1::SimulationSummary export_replay_trace(const sentinel::v1::Scenario& scenario,
                                                    const std::filesystem::path& log_path,
                                                    const std::filesystem::path& trace_path);
void write_summary_json(const std::filesystem::path& path, const sentinel::v1::SimulationSummary& summary);

}
