#pragma once

#include <sentinel/core/simulator.hpp>

#include <algorithm>
#include <filesystem>

namespace sentinel::test {

inline std::filesystem::path nominal_scenario_path() {
    return std::filesystem::path(SENTINEL_SOURCE_DIR) / "scenarios" / "stress.textproto";
}

inline sentinel::v1::ActionBatch idle_actions(const sentinel::core::Simulator& simulator, bool reverse = false) {
    sentinel::v1::ActionBatch batch;
    batch.set_tick(simulator.tick());
    for (const auto& vehicle : simulator.scenario().vehicles()) {
        auto* envelope = batch.add_actions();
        envelope->set_schema_version(1);
        envelope->set_sequence(simulator.tick() * simulator.scenario().vehicles_size() + batch.actions_size());
        envelope->set_simulation_time_ms(simulator.tick() * simulator.scenario().step_ms());
        envelope->set_sender_id(vehicle.id());
        envelope->set_recipient_id("sim");
        envelope->mutable_action()->set_tick(simulator.tick());
    }
    if (reverse) {
        std::reverse(batch.mutable_actions()->begin(), batch.mutable_actions()->end());
    }
    return batch;
}

}
