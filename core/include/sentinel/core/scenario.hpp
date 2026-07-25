#pragma once

#include <sentinel/v1/sentinel.pb.h>

#include <filesystem>
#include <string>

namespace sentinel::core {

sentinel::v1::Scenario load_scenario(const std::filesystem::path& path);
void normalize_scenario(sentinel::v1::Scenario& scenario);
void validate_scenario(const sentinel::v1::Scenario& scenario);
std::string scenario_hash(const sentinel::v1::Scenario& scenario);

}
