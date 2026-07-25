#include <sentinel/core/scenario.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/protocol/framing.hpp>

#include <google/protobuf/text_format.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace sentinel::core {
namespace {

constexpr std::int64_t max_world_mm = 1'000'000'000;
constexpr std::uint64_t max_step_ms = 60'000;
constexpr std::uint64_t max_ticks = 10'000'000;
constexpr std::int64_t max_speed_mm_s = 1'000'000;
constexpr std::int64_t max_energy_mj = 1'000'000'000'000'000;
constexpr std::int64_t max_energy_cost_mj_per_meter = 1'000'000;
constexpr std::int64_t max_payload_grams = 1'000'000'000;
constexpr std::uint32_t max_energy_multiplier_permille = 100'000;
constexpr std::int64_t max_grid_cells = 1'000'000;
constexpr std::uint64_t max_bandwidth_bytes_per_tick = 1'000'000'000;

bool valid_id(std::string_view value) {
    return !value.empty() && value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")
                                 == std::string_view::npos;
}

bool valid_capability(sentinel::v1::Capability value) {
    return value == sentinel::v1::CAPABILITY_SEARCH || value == sentinel::v1::CAPABILITY_INSPECTION
           || value == sentinel::v1::CAPABILITY_RELAY || value == sentinel::v1::CAPABILITY_DELIVERY;
}

bool valid_region_kind(sentinel::v1::RegionKind value) {
    return value == sentinel::v1::REGION_KIND_OBSTACLE || value == sentinel::v1::REGION_KIND_RESTRICTED
           || value == sentinel::v1::REGION_KIND_TERRAIN || value == sentinel::v1::REGION_KIND_CHOKEPOINT;
}

bool valid_event_kind(sentinel::v1::TapeEventKind value) {
    return value == sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK
           || value == sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE
           || value == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE;
}

bool multiplier_event(sentinel::v1::TapeEventKind value) {
    return value == sentinel::v1::TAPE_EVENT_KIND_SET_SPEED_PERMILLE
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE;
}

bool valid_location_kind(sentinel::v1::LocationKind value) {
    return value == sentinel::v1::LOCATION_KIND_CHARGING || value == sentinel::v1::LOCATION_KIND_RETURN;
}

bool valid_allocation_policy(sentinel::v1::AllocationPolicy value) {
    return value == sentinel::v1::ALLOCATION_POLICY_SCRIPTED
           || value == sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE
           || value == sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA;
}

template <typename Range, typename Selector>
void require_unique(const Range& values, Selector selector, std::string_view label) {
    for (int i = 1; i < values.size(); ++i) {
        if (selector(values.Get(i - 1)) == selector(values.Get(i))) {
            throw std::invalid_argument(std::string("duplicate ") + std::string(label));
        }
    }
}

bool has_capability(const sentinel::v1::VehicleSpec& vehicle, sentinel::v1::Capability capability) {
    return std::find(vehicle.capabilities().begin(), vehicle.capabilities().end(), capability)
           != vehicle.capabilities().end();
}

const sentinel::v1::VehicleSpec& vehicle_by_id(const sentinel::v1::Scenario& scenario, std::string_view id) {
    const auto position = std::lower_bound(scenario.vehicles().begin(), scenario.vehicles().end(), id,
                                           [](const sentinel::v1::VehicleSpec& vehicle, std::string_view value) {
                                               return vehicle.id() < value;
                                           });
    if (position == scenario.vehicles().end() || position->id() != id) {
        throw std::invalid_argument("unknown assigned vehicle");
    }
    return *position;
}

bool task_exists(const sentinel::v1::Scenario& scenario, std::string_view id) {
    return std::any_of(scenario.tasks().begin(), scenario.tasks().end(), [id](const auto& task) {
        return task.id() == id;
    });
}

bool vehicle_exists(const sentinel::v1::Scenario& scenario, std::string_view id) {
    return std::any_of(scenario.vehicles().begin(), scenario.vehicles().end(), [id](const auto& vehicle) {
        return vehicle.id() == id;
    });
}

bool region_exists(const sentinel::v1::Scenario& scenario, std::string_view id) {
    return std::any_of(scenario.world().regions().begin(), scenario.world().regions().end(), [id](const auto& region) {
        return region.id() == id;
    });
}

bool network_profile_exists(const sentinel::v1::Scenario& scenario, std::string_view id) {
    return std::any_of(scenario.network_profiles().begin(), scenario.network_profiles().end(), [id](const auto& profile) {
        return profile.id() == id;
    });
}

const sentinel::v1::ServiceLocation* location_by_id(const sentinel::v1::Scenario& scenario, std::string_view id) {
    const auto position = std::lower_bound(scenario.world().locations().begin(), scenario.world().locations().end(), id,
                                           [](const auto& location, std::string_view value) {
                                               return location.id() < value;
                                           });
    return position == scenario.world().locations().end() || position->id() != id ? nullptr : &*position;
}

void validate_point(const sentinel::v1::Scenario& scenario, const sentinel::v1::Point& point) {
    if (point.x_mm() < 0 || point.y_mm() < 0 || point.x_mm() > scenario.world().width_mm()
        || point.y_mm() > scenario.world().height_mm()) {
        throw std::invalid_argument("point lies outside world bounds");
    }
}

bool grid_aligned(const sentinel::v1::Scenario& scenario, const sentinel::v1::Point& point) {
    return point.x_mm() % scenario.world().grid_cell_mm() == 0
           && point.y_mm() % scenario.world().grid_cell_mm() == 0;
}

}

sentinel::v1::Scenario load_scenario(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open scenario");
    }
    const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    sentinel::v1::Scenario scenario;
    if (!google::protobuf::TextFormat::ParseFromString(text, &scenario)) {
        throw std::invalid_argument("failed to parse scenario");
    }
    normalize_scenario(scenario);
    validate_scenario(scenario);
    return scenario;
}

void normalize_scenario(sentinel::v1::Scenario& scenario) {
    if (scenario.world().grid_cell_mm() == 0) {
        scenario.mutable_world()->set_grid_cell_mm(100);
    }
    if (scenario.world().map_version() == 0) {
        scenario.mutable_world()->set_map_version(1);
    }
    if (scenario.allocation_policy() == sentinel::v1::ALLOCATION_POLICY_UNSPECIFIED) {
        scenario.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_SCRIPTED);
    }
    if (scenario.failure_detection_ticks() == 0) {
        scenario.set_failure_detection_ticks(std::min<std::uint64_t>(5, scenario.max_ticks()));
    }
    // One canonical order feeds lookup, validation, and hashing.
    std::sort(scenario.mutable_network_profiles()->begin(), scenario.mutable_network_profiles()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    std::sort(scenario.mutable_world()->mutable_regions()->begin(), scenario.mutable_world()->mutable_regions()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    std::sort(scenario.mutable_world()->mutable_locations()->begin(), scenario.mutable_world()->mutable_locations()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    std::sort(scenario.mutable_vehicles()->begin(), scenario.mutable_vehicles()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    for (auto& vehicle : *scenario.mutable_vehicles()) {
        std::sort(vehicle.mutable_capabilities()->begin(), vehicle.mutable_capabilities()->end());
        std::sort(vehicle.mutable_terrain_access()->begin(), vehicle.mutable_terrain_access()->end());
    }
    std::sort(scenario.mutable_tasks()->begin(), scenario.mutable_tasks()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    for (auto& task : *scenario.mutable_tasks()) {
        if (task.priority() == 0) {
            task.set_priority(50);
        }
    }
    // event id breaks same-tick ties
    std::sort(scenario.mutable_events()->begin(), scenario.mutable_events()->end(),
              [](const auto& left, const auto& right) {
                  return left.tick() < right.tick() || (left.tick() == right.tick() && left.id() < right.id());
              });
}

void validate_scenario(const sentinel::v1::Scenario& scenario) {
    if (scenario.schema_version() != 1 || !valid_id(scenario.name())) {
        throw std::invalid_argument("invalid scenario identity");
    }
    if (scenario.step_ms() == 0 || scenario.step_ms() > max_step_ms || scenario.max_ticks() == 0
        || scenario.max_ticks() > max_ticks || scenario.world().width_mm() <= 0
        || scenario.world().width_mm() > max_world_mm || scenario.world().height_mm() <= 0
        || scenario.world().height_mm() > max_world_mm || !valid_id(scenario.network_profile())
        || scenario.world().grid_cell_mm() <= 0 || scenario.world().grid_cell_mm() > max_world_mm
        || scenario.world().width_mm() % scenario.world().grid_cell_mm() != 0
        || scenario.world().height_mm() % scenario.world().grid_cell_mm() != 0
        || (scenario.world().width_mm() / scenario.world().grid_cell_mm() + 1)
               * (scenario.world().height_mm() / scenario.world().grid_cell_mm() + 1) > max_grid_cells
        || scenario.world().map_version() == 0 || !valid_allocation_policy(scenario.allocation_policy())
        || scenario.failure_detection_ticks() == 0
        || scenario.failure_detection_ticks() > scenario.max_ticks()) {
        throw std::invalid_argument("invalid scenario timing or world");
    }
    if (scenario.vehicles_size() < 3 || scenario.vehicles_size() > 5 || scenario.tasks().empty()) {
        throw std::invalid_argument("scenario requires three to five vehicles and at least one task");
    }
    require_unique(scenario.world().regions(), [](const auto& value) { return value.id(); }, "region id");
    require_unique(scenario.world().locations(), [](const auto& value) { return value.id(); }, "location id");
    require_unique(scenario.vehicles(), [](const auto& value) { return value.id(); }, "vehicle id");
    require_unique(scenario.tasks(), [](const auto& value) { return value.id(); }, "task id");
    require_unique(scenario.network_profiles(), [](const auto& value) { return value.id(); }, "network profile id");
    if (!scenario.network_profiles().empty() && !network_profile_exists(scenario, scenario.network_profile())) {
        throw std::invalid_argument("unknown active network profile");
    }
    for (const auto& profile : scenario.network_profiles()) {
        if (!valid_id(profile.id()) || profile.latency_ticks() > scenario.max_ticks()
            || profile.jitter_ticks() > scenario.max_ticks() || profile.packet_loss_permyriad() > 10'000
            || profile.bandwidth_bytes_per_tick() == 0
            || profile.bandwidth_bytes_per_tick() > max_bandwidth_bytes_per_tick
            || profile.reorder_permyriad() > 10'000 || profile.reorder_window_ticks() > scenario.max_ticks()
            || (profile.reorder_permyriad() != 0 && profile.reorder_window_ticks() == 0)) {
            throw std::invalid_argument("invalid network profile");
        }
    }
    for (const auto& region : scenario.world().regions()) {
        if (!valid_id(region.id()) || !valid_region_kind(region.kind())
            || region.minimum().x_mm() >= region.maximum().x_mm()
            || region.minimum().y_mm() >= region.maximum().y_mm()
            || region.energy_multiplier_permille() == 0
            || region.energy_multiplier_permille() > max_energy_multiplier_permille
            || (region.kind() == sentinel::v1::REGION_KIND_TERRAIN && !valid_id(region.terrain()))) {
            throw std::invalid_argument("invalid region");
        }
        validate_point(scenario, region.minimum());
        validate_point(scenario, region.maximum());
    }
    for (const auto& location : scenario.world().locations()) {
        if (!valid_id(location.id()) || !valid_location_kind(location.kind()) || location.radius_mm() < 0
            || location.radius_mm() > max_world_mm || location.charge_mj_per_tick() < 0
            || location.charge_mj_per_tick() > max_energy_mj
            || (location.kind() == sentinel::v1::LOCATION_KIND_CHARGING && location.charge_mj_per_tick() == 0)
            || (location.kind() == sentinel::v1::LOCATION_KIND_RETURN && location.charge_mj_per_tick() != 0)) {
            throw std::invalid_argument("invalid service location");
        }
        validate_point(scenario, location.position());
        if (!grid_aligned(scenario, location.position())) {
            throw std::invalid_argument("service location is not grid aligned");
        }
    }
    std::vector<std::string> kinds;
    for (const auto& vehicle : scenario.vehicles()) {
        if (!valid_id(vehicle.id()) || vehicle.kind().empty() || vehicle.max_speed_mm_s() <= 0
            || vehicle.max_speed_mm_s() > max_speed_mm_s || vehicle.initial_energy_mj() <= 0
            || vehicle.initial_energy_mj() > max_energy_mj || vehicle.energy_cost_mj_per_meter() < 0
            || vehicle.energy_cost_mj_per_meter() > max_energy_cost_mj_per_meter || vehicle.payload_grams() < 0
            || vehicle.payload_grams() > max_payload_grams || vehicle.capabilities().empty()
            || vehicle.terrain_access().empty()) {
            throw std::invalid_argument("invalid vehicle");
        }
        if (!std::all_of(vehicle.capabilities().begin(), vehicle.capabilities().end(), [](int capability) {
                return valid_capability(static_cast<sentinel::v1::Capability>(capability));
            })
            || !std::all_of(vehicle.terrain_access().begin(), vehicle.terrain_access().end(), valid_id)
            || std::adjacent_find(vehicle.capabilities().begin(), vehicle.capabilities().end())
                   != vehicle.capabilities().end()
            || std::adjacent_find(vehicle.terrain_access().begin(), vehicle.terrain_access().end())
                   != vehicle.terrain_access().end()) {
            throw std::invalid_argument("invalid vehicle capabilities or terrain access");
        }
        validate_point(scenario, vehicle.initial_position());
        if (!grid_aligned(scenario, vehicle.initial_position())) {
            throw std::invalid_argument("vehicle is not grid aligned");
        }
        if (!vehicle.return_location_id().empty()) {
            const auto* location = location_by_id(scenario, vehicle.return_location_id());
            if (!location || location->kind() != sentinel::v1::LOCATION_KIND_RETURN) {
                throw std::invalid_argument("invalid return location");
            }
        }
        kinds.push_back(vehicle.kind());
    }
    std::sort(kinds.begin(), kinds.end());
    kinds.erase(std::unique(kinds.begin(), kinds.end()), kinds.end());
    if (kinds.size() < 2) {
        throw std::invalid_argument("scenario vehicles are not heterogeneous");
    }
    for (const auto& task : scenario.tasks()) {
        if (!valid_id(task.id()) || task.kind().empty()
            || !valid_capability(task.required_capability())
            || task.payload_required_grams() < 0 || task.payload_required_grams() > max_payload_grams
            || task.deadline_tick() == 0
            || task.deadline_tick() > scenario.max_ticks() || task.completion_radius_mm() < 0
            || task.completion_radius_mm() > max_world_mm
            || task.service_ticks() > scenario.max_ticks() || task.service_energy_mj_per_tick() < 0
            || task.service_energy_mj_per_tick() > max_energy_mj
            || task.priority() == 0 || task.priority() > 100
            || (!task.assigned_agent_id().empty() && !valid_id(task.assigned_agent_id()))) {
            throw std::invalid_argument("invalid task");
        }
        validate_point(scenario, task.target());
        if (!grid_aligned(scenario, task.target())
            || (task.service_ticks() != 0
                && task.service_energy_mj_per_tick() > max_energy_mj / static_cast<std::int64_t>(task.service_ticks()))) {
            throw std::invalid_argument("invalid task service bounds");
        }
        if (scenario.allocation_policy() == sentinel::v1::ALLOCATION_POLICY_SCRIPTED
            && task.assigned_agent_id().empty()) {
            throw std::invalid_argument("scripted task has no assigned vehicle");
        }
        if (!task.assigned_agent_id().empty()) {
            const auto& vehicle = vehicle_by_id(scenario, task.assigned_agent_id());
            if (!has_capability(vehicle, task.required_capability())
                || vehicle.payload_grams() < task.payload_required_grams()) {
                throw std::invalid_argument("task assignment is infeasible");
            }
        } else if (!std::any_of(scenario.vehicles().begin(), scenario.vehicles().end(), [&task](const auto& vehicle) {
                       return has_capability(vehicle, task.required_capability())
                              && vehicle.payload_grams() >= task.payload_required_grams();
                   })) {
            throw std::invalid_argument("task assignment is infeasible");
        }
    }
    std::vector<std::string> event_ids;
    std::vector<std::int64_t> maximum_energy;
    maximum_energy.reserve(scenario.vehicles_size());
    for (const auto& vehicle : scenario.vehicles()) {
        maximum_energy.push_back(vehicle.initial_energy_mj());
    }
    for (const auto& event : scenario.events()) {
        if (!valid_id(event.id()) || event.tick() >= scenario.max_ticks() || !valid_event_kind(event.kind())) {
            throw std::invalid_argument("invalid tape event");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK && !task_exists(scenario, event.target_id())) {
            throw std::invalid_argument("release event targets unknown task");
        }
        if ((event.kind() == sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE
             || event.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA
             || multiplier_event(event.kind()))
            && !vehicle_exists(scenario, event.target_id())) {
            throw std::invalid_argument("vehicle event targets unknown vehicle");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED
            && !region_exists(scenario, event.target_id())) {
            throw std::invalid_argument("region event targets unknown region");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA
            && (event.value_min() > event.value_max() || event.value_min() < -max_energy_mj
                || event.value_max() > max_energy_mj || !valid_id(event.rng_stream()))) {
            throw std::invalid_argument("invalid energy event range");
        }
        if (multiplier_event(event.kind())
            && (event.value_min() < 1 || event.value_max() > 1000
                || event.value_min() > event.value_max() || !valid_id(event.rng_stream()))) {
            throw std::invalid_argument("invalid degradation event range");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA && event.value_max() > 0) {
            const auto position = std::lower_bound(scenario.vehicles().begin(), scenario.vehicles().end(),
                                                   event.target_id(), [](const auto& vehicle, std::string_view id) {
                                                       return vehicle.id() < id;
                                                   });
            const auto index = static_cast<std::size_t>(std::distance(scenario.vehicles().begin(), position));
            if (maximum_energy[index] > max_energy_mj - event.value_max()) {
                throw std::invalid_argument("energy events exceed numeric bounds");
            }
            maximum_energy[index] += event.value_max();
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE
            && (event.text_value().empty()
                || (!scenario.network_profiles().empty()
                    && !network_profile_exists(scenario, event.text_value())))) {
            throw std::invalid_argument("network event has invalid profile");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_LINK_BLOCKED
            && (!vehicle_exists(scenario, event.target_id()) || !vehicle_exists(scenario, event.text_value())
                || event.target_id() == event.text_value())) {
            throw std::invalid_argument("link event has invalid endpoints");
        }
        event_ids.push_back(event.id());
    }
    std::sort(event_ids.begin(), event_ids.end());
    if (std::adjacent_find(event_ids.begin(), event_ids.end()) != event_ids.end()) {
        throw std::invalid_argument("duplicate event id");
    }
}

std::string scenario_hash(const sentinel::v1::Scenario& scenario) {
    auto normalized = scenario;
    normalize_scenario(normalized);
    return hash_bytes(protocol::deterministic_bytes(normalized));
}

}
