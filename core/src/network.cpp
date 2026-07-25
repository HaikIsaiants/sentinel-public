#include <sentinel/core/network.hpp>

#include <sentinel/core/rng.hpp>
#include <sentinel/protocol/framing.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace sentinel::core {
namespace {

bool message_order(const sentinel::v1::NetworkMessage& left, const sentinel::v1::NetworkMessage& right) {
    return std::tuple{left.recipient_id(), left.sender_id(), left.version(), left.payload()}
           < std::tuple{right.recipient_id(), right.sender_id(), right.version(), right.payload()};
}

bool outcome_order(const sentinel::v1::NetworkOutcome& left, const sentinel::v1::NetworkOutcome& right) {
    return std::tuple{left.tick(), left.sequence(), left.disposition()}
           < std::tuple{right.tick(), right.sequence(), right.disposition()};
}

std::pair<std::string, std::string> link_pair(std::string_view first, std::string_view second) {
    if (first.empty() || second.empty() || first == second) {
        throw std::invalid_argument("invalid network link");
    }
    return first < second ? std::pair{std::string(first), std::string(second)}
                          : std::pair{std::string(second), std::string(first)};
}

void validate_profile(const sentinel::v1::NetworkProfile& profile) {
    if (profile.id().empty() || profile.packet_loss_permyriad() > 10000
        || profile.reorder_permyriad() > 10000 || profile.bandwidth_bytes_per_tick() == 0
        || (profile.reorder_permyriad() > 0 && profile.reorder_window_ticks() == 0)
        || profile.latency_ticks() > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())
        || profile.jitter_ticks() > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())
        || profile.jitter_ticks()
               > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())
                     - profile.latency_ticks()) {
        throw std::invalid_argument("invalid network profile");
    }
}

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right) {
    if (right > std::numeric_limits<std::uint64_t>::max() - left) {
        throw std::overflow_error("network tick overflow");
    }
    return left + right;
}

std::string draw_key(std::string_view label, const sentinel::v1::NetworkMessage& message,
                     std::uint64_t sequence) {
    return std::string("network-") + std::string(label) + ":" + message.sender_id() + ":"
           + message.recipient_id() + ":" + std::to_string(sequence);
}

std::int64_t draw(std::uint64_t seed, std::string_view label, const sentinel::v1::NetworkMessage& message,
                  std::uint64_t sequence, std::int64_t minimum, std::int64_t maximum) {
    DeterministicRng rng(stream_seed(seed, draw_key(label, message, sequence)));
    return rng.uniform(minimum, maximum);
}

void append_profile(HashBuilder& hash, const sentinel::v1::NetworkProfile& profile) {
    hash.text(profile.id());
    hash.unsigned_integer(profile.latency_ticks());
    hash.unsigned_integer(profile.jitter_ticks());
    hash.unsigned_integer(profile.packet_loss_permyriad());
    hash.unsigned_integer(profile.bandwidth_bytes_per_tick());
    hash.unsigned_integer(profile.reorder_permyriad());
    hash.unsigned_integer(profile.reorder_window_ticks());
}

}

NetworkEmulator::NetworkEmulator(std::uint64_t seed, std::uint64_t step_ms)
    : seed_(seed), step_ms_(step_ms) {
    if (step_ms == 0) {
        throw std::invalid_argument("invalid network clock");
    }
}

void NetworkEmulator::set_profile(const sentinel::v1::NetworkProfile& profile) {
    validate_profile(profile);
    profile_.CopyFrom(profile);
}

std::vector<sentinel::v1::NetworkOutcome> NetworkEmulator::set_link_blocked(
    std::string_view first, std::string_view second, bool value, std::uint64_t tick) {
    const auto key = link_pair(first, second);
    const auto position = std::lower_bound(blocked_links_.begin(), blocked_links_.end(), key);
    const auto present = position != blocked_links_.end() && *position == key;
    if (value == present) {
        return {};
    }
    if (!value) {
        blocked_links_.erase(position);
        return {};
    }
    blocked_links_.insert(position, key);
    std::vector<sentinel::v1::NetworkOutcome> outcomes;
    for (auto& current : links_) {
        if (link_pair(current.sender_id, current.recipient_id) != key) {
            continue;
        }
        for (const auto& queued : current.queue) {
            outcomes.push_back(outcome(queued, sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION, tick));
            ++dropped_messages_;
        }
        current.queue.clear();
    }
    for (auto current = in_flight_.begin(); current != in_flight_.end();) {
        if (link_pair(current->message.sender_id(), current->message.recipient_id()) == key) {
            outcomes.push_back(outcome(*current, sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION, tick));
            ++dropped_messages_;
            current = in_flight_.erase(current);
        } else {
            ++current;
        }
    }
    std::sort(outcomes.begin(), outcomes.end(), outcome_order);
    return outcomes;
}

bool NetworkEmulator::link_blocked(std::string_view first, std::string_view second) const {
    const auto key = link_pair(first, second);
    return std::binary_search(blocked_links_.begin(), blocked_links_.end(), key);
}

NetworkStep NetworkEmulator::step(std::uint64_t tick,
                                  const std::vector<sentinel::v1::NetworkMessage>& outgoing) {
    if (!outgoing.empty() && profile_.id().empty()) {
        throw std::logic_error("network profile is not set");
    }
    auto messages = outgoing;
    // Sort first; sequence numbers feed the keyed network draws.
    std::sort(messages.begin(), messages.end(), message_order);
    NetworkStep result;
    for (const auto& message : messages) {
        if (message.sender_id().empty() || message.recipient_id().empty()
            || message.sender_id() == message.recipient_id() || message.version() == 0) {
            throw std::invalid_argument("invalid network message");
        }
        auto current = packet(message, tick);
        communication_bytes_ += current.serialized_bytes;
        ++communication_messages_;
        if (link_blocked(message.sender_id(), message.recipient_id())) {
            result.outcomes.push_back(
                outcome(current, sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION, tick));
            ++dropped_messages_;
            continue;
        }
        const auto lost = draw(seed_, "loss", message, current.sequence, 0, 9999)
                          < static_cast<std::int64_t>(current.profile.packet_loss_permyriad());
        if (lost) {
            result.outcomes.push_back(outcome(current, sentinel::v1::NETWORK_DISPOSITION_DROPPED_LOSS, tick));
            ++dropped_messages_;
            continue;
        }
        link(message.sender_id(), message.recipient_id()).queue.push_back(current);
        result.outcomes.push_back(outcome(current, sentinel::v1::NETWORK_DISPOSITION_QUEUED, tick));
    }
    transmit(tick);
    // available to agents at the next tick boundary
    const auto observation_tick = checked_add(tick, 1);
    for (auto current = in_flight_.begin(); current != in_flight_.end();) {
        if (current->delivery_tick > observation_tick) {
            ++current;
            continue;
        }
        if (link_blocked(current->message.sender_id(), current->message.recipient_id())) {
            result.outcomes.push_back(
                outcome(*current, sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION, observation_tick));
            ++dropped_messages_;
        } else {
            result.delivered.push_back(current->message);
            result.outcomes.push_back(
                outcome(*current, sentinel::v1::NETWORK_DISPOSITION_DELIVERED, observation_tick));
            ++delivered_messages_;
            reordered_messages_ += current->reordered ? 1 : 0;
        }
        current = in_flight_.erase(current);
    }
    std::sort(result.outcomes.begin(), result.outcomes.end(), outcome_order);
    return result;
}

std::uint64_t NetworkEmulator::communication_bytes() const {
    return communication_bytes_;
}

std::uint64_t NetworkEmulator::communication_messages() const {
    return communication_messages_;
}

std::uint64_t NetworkEmulator::delivered_messages() const {
    return delivered_messages_;
}

std::uint64_t NetworkEmulator::dropped_messages() const {
    return dropped_messages_;
}

std::uint64_t NetworkEmulator::reordered_messages() const {
    return reordered_messages_;
}

void NetworkEmulator::append_hash(HashBuilder& hash) const {
    const auto append = [&hash](const Packet& packet) {
        hash.text(protocol::deterministic_bytes(packet.message));
        append_profile(hash, packet.profile);
        hash.unsigned_integer(packet.sequence);
        hash.unsigned_integer(packet.serialized_bytes);
        hash.unsigned_integer(packet.remaining_bytes);
        hash.unsigned_integer(packet.enqueue_tick);
        hash.unsigned_integer(packet.transmit_start_tick);
        hash.unsigned_integer(packet.transmit_end_tick);
        hash.unsigned_integer(packet.delivery_tick);
        hash.unsigned_integer(packet.reorder_delay_ticks);
        hash.signed_integer(packet.jitter_ticks);
        hash.boolean(packet.started);
        hash.boolean(packet.reordered);
    };
    hash.unsigned_integer(seed_);
    hash.unsigned_integer(step_ms_);
    append_profile(hash, profile_);
    hash.unsigned_integer(next_sequence_);
    hash.unsigned_integer(communication_bytes_);
    hash.unsigned_integer(communication_messages_);
    hash.unsigned_integer(delivered_messages_);
    hash.unsigned_integer(dropped_messages_);
    hash.unsigned_integer(reordered_messages_);
    hash.unsigned_integer(blocked_links_.size());
    for (const auto& [first, second] : blocked_links_) {
        hash.text(first);
        hash.text(second);
    }
    hash.unsigned_integer(links_.size());
    for (const auto& current : links_) {
        hash.text(current.sender_id);
        hash.text(current.recipient_id);
        hash.unsigned_integer(current.queue.size());
        for (const auto& queued : current.queue) {
            append(queued);
        }
    }
    hash.unsigned_integer(in_flight_.size());
    for (const auto& current : in_flight_) {
        append(current);
    }
}

NetworkEmulator::Link& NetworkEmulator::link(std::string_view sender, std::string_view recipient) {
    const auto key = std::pair{sender, recipient};
    const auto position = std::lower_bound(links_.begin(), links_.end(), key, [](const Link& current, const auto& value) {
        return std::pair{std::string_view(current.sender_id), std::string_view(current.recipient_id)} < value;
    });
    if (position != links_.end() && position->sender_id == sender && position->recipient_id == recipient) {
        return *position;
    }
    return *links_.insert(position, Link{std::string(sender), std::string(recipient), {}});
}

NetworkEmulator::Packet NetworkEmulator::packet(const sentinel::v1::NetworkMessage& message,
                                                std::uint64_t tick) {
    Packet result;
    result.message.CopyFrom(message);
    result.profile.CopyFrom(profile_);
    result.sequence = next_sequence_++;
    result.enqueue_tick = tick;
    const auto jitter = static_cast<std::int64_t>(result.profile.jitter_ticks());
    result.jitter_ticks = draw(seed_, "jitter", message, result.sequence, -jitter, jitter);
    result.reordered = result.profile.reorder_window_ticks() > 0
                       && draw(seed_, "reorder", message, result.sequence, 0, 9999)
                              < static_cast<std::int64_t>(result.profile.reorder_permyriad());
    if (result.reordered) {
        result.reorder_delay_ticks = static_cast<std::uint64_t>(
            draw(seed_, "reorder-delay", message, result.sequence, 1,
                 result.profile.reorder_window_ticks()));
    }
    sentinel::v1::Envelope envelope;
    envelope.set_schema_version(1);
    envelope.set_sequence(result.sequence);
    if (tick > std::numeric_limits<std::uint64_t>::max() / step_ms_) {
        throw std::overflow_error("network time overflow");
    }
    envelope.set_simulation_time_ms(tick * step_ms_);
    envelope.set_sender_id(message.sender_id());
    envelope.set_recipient_id(message.recipient_id());
    envelope.mutable_message()->CopyFrom(message);
    result.serialized_bytes = protocol::deterministic_bytes(envelope).size();
    result.remaining_bytes = result.serialized_bytes;
    return result;
}

sentinel::v1::NetworkOutcome NetworkEmulator::outcome(
    const Packet& packet, sentinel::v1::NetworkDisposition disposition, std::uint64_t tick) const {
    sentinel::v1::NetworkOutcome result;
    result.set_sequence(packet.sequence);
    result.set_disposition(disposition);
    result.set_tick(tick);
    result.mutable_message()->CopyFrom(packet.message);
    result.set_serialized_bytes(packet.serialized_bytes);
    result.set_enqueue_tick(packet.enqueue_tick);
    result.set_transmit_start_tick(packet.transmit_start_tick);
    result.set_transmit_end_tick(packet.transmit_end_tick);
    result.set_delivery_tick(packet.delivery_tick);
    result.set_jitter_ticks(packet.jitter_ticks);
    result.set_reordered(packet.reordered);
    result.set_profile_id(packet.profile.id());
    return result;
}

void NetworkEmulator::transmit(std::uint64_t tick) {
    for (auto& current : links_) {
        if (current.queue.empty() || link_blocked(current.sender_id, current.recipient_id)) {
            continue;
        }
        // bandwidth is captured when the head packet enters service.
        const auto rate = current.queue.front().profile.bandwidth_bytes_per_tick();
        auto budget = rate;
        while (budget > 0 && !current.queue.empty()
               && current.queue.front().profile.bandwidth_bytes_per_tick() == rate) {
            auto& packet = current.queue.front();
            if (!packet.started) {
                packet.started = true;
                packet.transmit_start_tick = tick;
            }
            const auto transmitted = std::min(budget, packet.remaining_bytes);
            budget -= transmitted;
            packet.remaining_bytes -= transmitted;
            if (packet.remaining_bytes > 0) {
                break;
            }
            packet.transmit_end_tick = tick;
            const auto latency = static_cast<std::int64_t>(packet.profile.latency_ticks());
            const auto resolved = latency + packet.jitter_ticks;
            const auto delay = static_cast<std::uint64_t>(std::max<std::int64_t>(1, resolved));
            packet.delivery_tick = checked_add(checked_add(tick, delay), packet.reorder_delay_ticks);
            in_flight_.push_back(std::move(packet));
            current.queue.pop_front();
        }
    }
    std::sort(in_flight_.begin(), in_flight_.end(), [](const Packet& left, const Packet& right) {
        return std::tuple{left.delivery_tick, left.sequence} < std::tuple{right.delivery_tick, right.sequence};
    });
}

}
