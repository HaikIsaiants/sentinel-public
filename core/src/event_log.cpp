#include <sentinel/core/event_log.hpp>

#include <sentinel/core/scenario.hpp>
#include <sentinel/core/simulator.hpp>
#include <sentinel/protocol/framing.hpp>

#include <filesystem>
#include <stdexcept>

namespace sentinel::core {

EventLogWriter::EventLogWriter(const std::filesystem::path& path, const sentinel::v1::Scenario& scenario)
    : output_(path, std::ios::binary | std::ios::trunc) {
    if (!output_) {
        throw std::runtime_error("failed to open event log");
    }
    sentinel::v1::EventLogEntry entry;
    auto* header = entry.mutable_header();
    header->set_schema_version(1);
    header->set_scenario_name(scenario.name());
    header->set_scenario_seed(scenario.seed());
    header->set_scenario_hash(scenario_hash(scenario));
    protocol::write_frame(output_, entry, false);
}

void EventLogWriter::append(std::uint64_t tick, const sentinel::v1::ActionBatch& actions,
                            const std::vector<sentinel::v1::AppliedEvent>& applied_events,
                            const std::vector<sentinel::v1::NetworkOutcome>& network_outcomes,
                            const std::string& state_hash) {
    if (finished_) {
        throw std::logic_error("event log is finalized");
    }
    sentinel::v1::EventLogEntry entry;
    auto* record = entry.mutable_record();
    record->set_sequence(++sequence_);
    record->set_tick(tick);
    record->mutable_actions()->CopyFrom(actions);
    for (const auto& event : applied_events) {
        record->add_applied_events()->CopyFrom(event);
    }
    for (const auto& outcome : network_outcomes) {
        record->add_network_outcomes()->CopyFrom(outcome);
    }
    record->set_state_hash(state_hash);
    protocol::write_frame(output_, entry, false);
}

void EventLogWriter::finish(const sentinel::v1::SimulationSummary& summary) {
    if (finished_) {
        throw std::logic_error("event log is finalized");
    }
    sentinel::v1::EventLogEntry entry;
    entry.mutable_footer()->mutable_summary()->CopyFrom(summary);
    protocol::write_frame(output_, entry);
    finished_ = true;
}

sentinel::v1::SimulationSummary replay_event_log(const sentinel::v1::Scenario& scenario,
                                                  const std::filesystem::path& path,
                                                  std::vector<sentinel::v1::ObservationBatch>* observations) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("failed to open event log");
    }
    sentinel::v1::EventLogEntry entry;
    if (!protocol::read_frame(input, entry) || !entry.has_header()) {
        throw std::runtime_error("event log header is missing");
    }
    const auto& header = entry.header();
    if (header.schema_version() != 1 || header.scenario_name() != scenario.name()
        || header.scenario_seed() != scenario.seed() || header.scenario_hash() != scenario_hash(scenario)) {
        throw std::runtime_error("event log scenario mismatch");
    }
    Simulator simulator(scenario);
    if (observations != nullptr) {
        observations->clear();
        observations->push_back(simulator.observe());
    }
    std::uint64_t sequence{};
    bool footer_seen{};
    sentinel::v1::SimulationSummary footer_summary;
    entry.Clear();
    while (protocol::read_frame(input, entry)) {
        if (entry.has_footer()) {
            if (footer_seen) {
                throw std::runtime_error("event log contains duplicate terminal events");
            }
            footer_summary.CopyFrom(entry.footer().summary());
            footer_seen = true;
            entry.Clear();
            continue;
        }
        if (footer_seen || !entry.has_record() || entry.record().sequence() != ++sequence) {
            throw std::runtime_error("invalid event log sequence");
        }
        const auto& record = entry.record();
        const auto outcome = simulator.replay_step(record.actions(), record.applied_events());
        if (record.tick() != simulator.tick() || record.state_hash() != simulator.state_hash()
            || record.applied_events_size() != static_cast<int>(outcome.applied_events.size())
            || record.network_outcomes_size() != static_cast<int>(outcome.network_outcomes.size())) {
            throw std::runtime_error("event log replay diverged");
        }
        for (int index = 0; index < record.network_outcomes_size(); ++index) {
            if (protocol::deterministic_bytes(record.network_outcomes(index))
                != protocol::deterministic_bytes(outcome.network_outcomes[static_cast<std::size_t>(index)])) {
                throw std::runtime_error("event log network replay diverged");
            }
        }
        for (int index = 0; index < record.applied_events_size(); ++index) {
            if (protocol::deterministic_bytes(record.applied_events(index))
                != protocol::deterministic_bytes(outcome.applied_events[static_cast<std::size_t>(index)])) {
                throw std::runtime_error("event log event replay diverged");
            }
        }
        if (observations != nullptr) {
            observations->push_back(outcome.observations);
        }
        entry.Clear();
    }
    const auto summary = simulator.summary();
    if (!simulator.finished() || !footer_seen
        || protocol::deterministic_bytes(footer_summary) != protocol::deterministic_bytes(summary)) {
        throw std::runtime_error("event log terminal event is invalid");
    }
    return summary;
}

sentinel::v1::SimulationSummary export_replay_trace(const sentinel::v1::Scenario& scenario,
                                                    const std::filesystem::path& log_path,
                                                    const std::filesystem::path& trace_path) {
    std::vector<sentinel::v1::ObservationBatch> observations;
    const auto summary = replay_event_log(scenario, log_path, &observations);
    if (!trace_path.parent_path().empty()) {
        std::filesystem::create_directories(trace_path.parent_path());
    }
    auto tmp_path = trace_path;
    tmp_path += ".tmp";
    try {
        std::ofstream output(tmp_path, std::ios::binary | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("failed to open trace output");
        }
        sentinel::v1::SimulationFrame frame;
        for (const auto& observation : observations) {
            frame.Clear();
            frame.mutable_observations()->CopyFrom(observation);
            protocol::write_frame(output, frame, false);
        }
        output.close();
        if (!output) {
            throw std::runtime_error("failed to write trace output");
        }
        std::filesystem::rename(tmp_path, trace_path);
    } catch (...) {
        std::error_code error;
        std::filesystem::remove(tmp_path, error);
        throw;
    }
    return summary;
}

void write_summary_json(const std::filesystem::path& path, const sentinel::v1::SimulationSummary& summary) {
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        throw std::runtime_error("failed to open summary output");
    }
    output << "{\n";
    output << "  \"success\": " << (summary.success() ? "true" : "false") << ",\n";
    output << "  \"ticks\": " << summary.ticks() << ",\n";
    output << "  \"terminal_hash\": \"" << summary.terminal_hash() << "\",\n";
    output << "  \"completed_tasks\": " << summary.completed_tasks() << ",\n";
    output << "  \"total_tasks\": " << summary.total_tasks() << ",\n";
    output << "  \"active_agents\": " << summary.active_agents() << ",\n";
    output << "  \"wait_ticks\": " << summary.wait_ticks() << ",\n";
    output << "  \"replan_count\": " << summary.replan_count() << ",\n";
    output << "  \"recharge_ticks\": " << summary.recharge_ticks() << ",\n";
    output << "  \"return_ticks\": " << summary.return_ticks() << ",\n";
    output << "  \"route_conflicts\": " << summary.route_conflicts() << ",\n";
    output << "  \"travel_distance_mm\": " << summary.travel_distance_mm() << ",\n";
    output << "  \"energy_consumed_mj\": " << summary.energy_consumed_mj() << ",\n";
    output << "  \"rejected_commits\": " << summary.rejected_commits() << ",\n";
    output << "  \"communication_bytes\": " << summary.communication_bytes() << ",\n";
    output << "  \"communication_messages\": " << summary.communication_messages() << ",\n";
    output << "  \"delivered_messages\": " << summary.delivered_messages() << ",\n";
    output << "  \"dropped_messages\": " << summary.dropped_messages() << ",\n";
    output << "  \"reordered_messages\": " << summary.reordered_messages() << ",\n";
    output << "  \"maximum_reassignment_delay_ms\": " << summary.maximum_reassignment_delay_ms() << ",\n";
    output << "  \"missing_reassignments\": " << summary.missing_reassignments() << ",\n";
    output << "  \"agent_energy_never_drops_below_zero\": "
           << (summary.agent_energy_never_drops_below_zero() ? "true" : "false") << ",\n";
    output << "  \"agent_energy_below_zero_violations\": "
           << summary.agent_energy_below_zero_violations() << ",\n";
    output << "  \"committed_reservations_never_overlap\": "
           << (summary.committed_reservations_never_overlap() ? "true" : "false") << ",\n";
    output << "  \"committed_reservation_overlap_violations\": "
           << summary.committed_reservation_overlap_violations() << ",\n";
    output << "  \"completed_tasks_are_never_reassigned\": "
           << (summary.completed_tasks_are_never_reassigned() ? "true" : "false") << ",\n";
    output << "  \"completed_task_reassignment_violations\": "
           << summary.completed_task_reassignment_violations() << ",\n";
    output << "  \"incapable_agents_never_commit_tasks\": "
           << (summary.incapable_agents_never_commit_tasks() ? "true" : "false") << ",\n";
    output << "  \"incapable_agent_commit_violations\": "
           << summary.incapable_agent_commit_violations() << ",\n";
    output << "  \"allocation_convergence\": [";
    for (int index = 0; index < summary.allocation_convergence_size(); ++index) {
        const auto& sample = summary.allocation_convergence(index);
        output << (index == 0 ? "" : ", ") << "{\"epoch\": " << sample.epoch()
                << ", \"start_tick\": " << sample.start_tick() << ", \"end_tick\": "
                << sample.end_tick() << ", \"complete\": " << (sample.complete() ? "true" : "false") << '}';
    }
    output << "],\n";
    output << "  \"failure_detections\": [";
    for (int index = 0; index < summary.failure_detections_size(); ++index) {
        const auto& sample = summary.failure_detections(index);
        output << (index == 0 ? "" : ", ") << "{\"failed_agent_id\": \"" << sample.failed_agent_id()
               << "\", \"detector_agent_id\": \"" << sample.detector_agent_id()
               << "\", \"failure_tick\": " << sample.failure_tick() << ", \"detection_tick\": "
               << sample.detection_tick() << '}';
    }
    output << "],\n";
    output << "  \"task_reassignments\": [";
    for (int index = 0; index < summary.task_reassignments_size(); ++index) {
        const auto& sample = summary.task_reassignments(index);
        output << (index == 0 ? "" : ", ") << "{\"task_id\": \"" << sample.task_id()
               << "\", \"failed_agent_id\": \"" << sample.failed_agent_id()
               << "\", \"detector_agent_id\": \"" << sample.detector_agent_id()
               << "\", \"failure_tick\": " << sample.failure_tick() << ", \"detection_tick\": "
               << sample.detection_tick() << ", \"previous_epoch\": " << sample.previous_epoch()
               << ", \"previous_version\": " << sample.previous_version() << ", \"new_agent_id\": \""
               << sample.new_agent_id() << "\", \"new_epoch\": " << sample.new_epoch()
               << ", \"new_version\": " << sample.new_version() << ", \"commit_tick\": "
               << sample.commit_tick() << ", \"complete\": " << (sample.complete() ? "true" : "false") << '}';
    }
    output << "],\n";
    output << "  \"replanning_samples\": [";
    for (int index = 0; index < summary.replanning_samples_size(); ++index) {
        const auto& sample = summary.replanning_samples(index);
        output << (index == 0 ? "" : ", ") << "{\"agent_id\": \"" << sample.agent_id()
               << "\", \"reason\": " << sample.reason() << ", \"start_tick\": " << sample.start_tick()
               << ", \"end_tick\": " << sample.end_tick() << ", \"wait_plan\": "
               << (sample.wait_plan() ? "true" : "false") << ", \"complete\": "
               << (sample.complete() ? "true" : "false") << '}';
    }
    output << "],\n";
    output << "  \"tick_hashes\": [";
    for (int index = 0; index < summary.tick_hashes_size(); ++index) {
        output << (index == 0 ? "" : ", ") << '"' << summary.tick_hashes(index) << '"';
    }
    output << "]\n}\n";
    if (!output) {
        throw std::runtime_error("failed to write summary output");
    }
}

}
