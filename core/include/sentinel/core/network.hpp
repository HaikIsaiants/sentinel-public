#pragma once

#include <sentinel/core/hash.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <cstdint>
#include <deque>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sentinel::core {

struct NetworkStep {
    std::vector<sentinel::v1::NetworkMessage> delivered;
    std::vector<sentinel::v1::NetworkOutcome> outcomes;
};

class NetworkEmulator {
public:
    NetworkEmulator(std::uint64_t seed, std::uint64_t step_ms);
    void set_profile(const sentinel::v1::NetworkProfile& profile);
    std::vector<sentinel::v1::NetworkOutcome> set_link_blocked(
        std::string_view first, std::string_view second, bool blocked, std::uint64_t tick);
    bool link_blocked(std::string_view first, std::string_view second) const;
    NetworkStep step(std::uint64_t tick, const std::vector<sentinel::v1::NetworkMessage>& outgoing);
    std::uint64_t communication_bytes() const;
    std::uint64_t communication_messages() const;
    std::uint64_t delivered_messages() const;
    std::uint64_t dropped_messages() const;
    std::uint64_t reordered_messages() const;
    void append_hash(HashBuilder& hash) const;

private:
    struct Packet {
        sentinel::v1::NetworkMessage message;
        sentinel::v1::NetworkProfile profile;
        std::uint64_t sequence{};
        std::uint64_t serialized_bytes{};
        std::uint64_t remaining_bytes{};
        std::uint64_t enqueue_tick{};
        std::uint64_t transmit_start_tick{};
        std::uint64_t transmit_end_tick{};
        std::uint64_t delivery_tick{};
        std::uint64_t reorder_delay_ticks{};
        std::int64_t jitter_ticks{};
        bool started{};
        bool reordered{};
    };

    struct Link {
        std::string sender_id;
        std::string recipient_id;
        std::deque<Packet> queue;
    };

    Link& link(std::string_view sender, std::string_view recipient);
    Packet packet(const sentinel::v1::NetworkMessage& message, std::uint64_t tick);
    sentinel::v1::NetworkOutcome outcome(const Packet& packet, sentinel::v1::NetworkDisposition disposition,
                                         std::uint64_t tick) const;
    void transmit(std::uint64_t tick);

    std::uint64_t seed_{};
    std::uint64_t step_ms_{};
    sentinel::v1::NetworkProfile profile_;
    std::vector<Link> links_;
    std::vector<Packet> in_flight_;
    std::vector<std::pair<std::string, std::string>> blocked_links_;
    std::uint64_t next_sequence_{1};
    std::uint64_t communication_bytes_{};
    std::uint64_t communication_messages_{};
    std::uint64_t delivered_messages_{};
    std::uint64_t dropped_messages_{};
    std::uint64_t reordered_messages_{};
};

}
