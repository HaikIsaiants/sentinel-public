#include <sentinel/core/rng.hpp>

#include <algorithm>
#include <stdexcept>

namespace sentinel::core {
namespace {

std::uint64_t mix(std::uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

std::uint64_t name_hash(std::string_view name) {
    std::uint64_t hash = 14695981039346656037ULL;
    for (const unsigned char value : name) {
        hash ^= value;
        hash *= 1099511628211ULL;
    }
    return hash;
}

}

DeterministicRng::DeterministicRng(std::uint64_t seed) : state_(seed) {}

std::uint64_t DeterministicRng::next() {
    state_ += 0x9e3779b97f4a7c15ULL;
    std::uint64_t value = state_;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

std::int64_t DeterministicRng::uniform(std::int64_t minimum, std::int64_t maximum) {
    if (minimum > maximum) {
        throw std::invalid_argument("invalid random range");
    }
    const std::uint64_t span = static_cast<std::uint64_t>(maximum) - static_cast<std::uint64_t>(minimum) + 1U;
    if (span == 0) {
        return static_cast<std::int64_t>(next());
    }
    // unbiased rejection sampling for an inclusive range.
    const std::uint64_t threshold = static_cast<std::uint64_t>(-span) % span;
    std::uint64_t value{};
    do {
        value = next();
    } while (value < threshold);
    return minimum + static_cast<std::int64_t>(value % span);
}

std::uint64_t DeterministicRng::state() const {
    return state_;
}

RngStreams::RngStreams(std::uint64_t root_seed) : root_seed_(root_seed) {}

DeterministicRng& RngStreams::stream(std::string_view name) {
    const auto position = std::lower_bound(entries_.begin(), entries_.end(), name,
                                           [](const Entry& entry, std::string_view value) {
                                               return entry.name < value;
                                           });
    if (position != entries_.end() && position->name == name) {
        return position->rng;
    }
    const auto inserted = entries_.insert(position, Entry{std::string(name), DeterministicRng(stream_seed(root_seed_, name))});
    return inserted->rng;
}

std::vector<std::pair<std::string, std::uint64_t>> RngStreams::states() const {
    std::vector<std::pair<std::string, std::uint64_t>> values;
    values.reserve(entries_.size());
    for (const auto& entry : entries_) {
        values.emplace_back(entry.name, entry.rng.state());
    }
    return values;
}

std::uint64_t stream_seed(std::uint64_t root_seed, std::string_view name) {
    return mix(root_seed ^ name_hash(name));
}

}
