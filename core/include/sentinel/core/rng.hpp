#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sentinel::core {

class DeterministicRng {
public:
    explicit DeterministicRng(std::uint64_t seed);
    std::uint64_t next();
    std::int64_t uniform(std::int64_t minimum, std::int64_t maximum);
    std::uint64_t state() const;

private:
    std::uint64_t state_{};
};

class RngStreams {
public:
    explicit RngStreams(std::uint64_t root_seed);
    DeterministicRng& stream(std::string_view name);
    std::vector<std::pair<std::string, std::uint64_t>> states() const;

private:
    struct Entry {
        std::string name;
        DeterministicRng rng;
    };

    std::uint64_t root_seed_{};
    std::vector<Entry> entries_;
};

std::uint64_t stream_seed(std::uint64_t root_seed, std::string_view name);

}
