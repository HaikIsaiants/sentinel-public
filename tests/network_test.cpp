#include <sentinel/core/network.hpp>
#include <sentinel/protocol/framing.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace {

sentinel::v1::NetworkProfile profile(const char* id, std::uint64_t latency, std::uint64_t jitter,
                                     std::uint32_t loss, std::uint64_t bandwidth,
                                     std::uint32_t reorder = 0, std::uint32_t window = 0) {
    sentinel::v1::NetworkProfile result;
    result.set_id(id);
    result.set_latency_ticks(latency);
    result.set_jitter_ticks(jitter);
    result.set_packet_loss_permyriad(loss);
    result.set_bandwidth_bytes_per_tick(bandwidth);
    result.set_reorder_permyriad(reorder);
    result.set_reorder_window_ticks(window);
    return result;
}

sentinel::v1::NetworkMessage message(const char* sender, const char* recipient, const char* payload,
                                     std::uint64_t version = 1) {
    sentinel::v1::NetworkMessage result;
    result.set_sender_id(sender);
    result.set_recipient_id(recipient);
    result.set_version(version);
    result.set_payload(payload);
    return result;
}

std::uint64_t serialized_bytes(const sentinel::v1::NetworkMessage& message, std::uint64_t sequence,
                               std::uint64_t tick, std::uint64_t step_ms) {
    sentinel::v1::Envelope envelope;
    envelope.set_schema_version(1);
    envelope.set_sequence(sequence);
    envelope.set_simulation_time_ms(tick * step_ms);
    envelope.set_sender_id(message.sender_id());
    envelope.set_recipient_id(message.recipient_id());
    envelope.mutable_message()->CopyFrom(message);
    return sentinel::protocol::deterministic_bytes(envelope).size();
}

const sentinel::v1::NetworkOutcome* disposition(const sentinel::core::NetworkStep& step,
                                                sentinel::v1::NetworkDisposition value,
                                                std::uint64_t sequence) {
    const auto position = std::find_if(step.outcomes.begin(), step.outcomes.end(), [value, sequence](const auto& item) {
        return item.disposition() == value && item.sequence() == sequence;
    });
    return position == step.outcomes.end() ? nullptr : &*position;
}

std::string state(const sentinel::core::NetworkEmulator& network) {
    sentinel::core::HashBuilder hash;
    network.append_hash(hash);
    return hash.finish();
}

}

TEST(NetworkEmulator, CanonicalizesAttemptsAndCountsExactEnvelopes) {
    const auto first = message("alpha", "bravo", "a");
    const auto second = message("alpha", "bravo", "b");
    sentinel::core::NetworkEmulator forward(17, 100);
    sentinel::core::NetworkEmulator reverse(17, 100);
    const auto perfect = profile("perfect", 0, 0, 0, 10000);
    forward.set_profile(perfect);
    reverse.set_profile(perfect);

    const auto left = forward.step(0, {second, first});
    const auto right = reverse.step(0, {first, second});
    ASSERT_EQ(left.delivered.size(), 2U);
    ASSERT_EQ(left.outcomes.size(), 4U);
    EXPECT_EQ(left.delivered[0].payload(), "a");
    EXPECT_EQ(left.delivered[1].payload(), "b");
    EXPECT_EQ(left.outcomes[0].serialized_bytes(), serialized_bytes(first, 1, 0, 100));
    EXPECT_EQ(left.outcomes[1].serialized_bytes(), serialized_bytes(second, 2, 0, 100));
    EXPECT_EQ(forward.communication_bytes(), left.outcomes[0].serialized_bytes() + left.outcomes[1].serialized_bytes());
    EXPECT_EQ(forward.communication_messages(), 2U);
    EXPECT_EQ(forward.delivered_messages(), 2U);
    EXPECT_EQ(forward.dropped_messages(), 0U);
    ASSERT_EQ(left.outcomes.size(), right.outcomes.size());
    for (std::size_t i = 0; i < left.outcomes.size(); ++i) {
        EXPECT_EQ(sentinel::protocol::deterministic_bytes(left.outcomes[i]),
                  sentinel::protocol::deterministic_bytes(right.outcomes[i]));
    }
    EXPECT_EQ(state(forward), state(reverse));
}

TEST(NetworkEmulator, SerializesAcrossTicksWithoutStarvation) {
    const auto value = message("alpha", "bravo", "bandwidth");
    const auto bytes = serialized_bytes(value, 1, 0, 100);
    ASSERT_GT(bytes, 1U);
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile("limited", 0, 0, 0, bytes - 1));

    const auto first = network.step(0, {value});
    EXPECT_TRUE(first.delivered.empty());
    ASSERT_NE(disposition(first, sentinel::v1::NETWORK_DISPOSITION_QUEUED, 1), nullptr);
    const auto second = network.step(1, {});
    ASSERT_EQ(second.delivered.size(), 1U);
    const auto* delivered = disposition(second, sentinel::v1::NETWORK_DISPOSITION_DELIVERED, 1);
    ASSERT_NE(delivered, nullptr);
    EXPECT_EQ(delivered->transmit_start_tick(), 0U);
    EXPECT_EQ(delivered->transmit_end_tick(), 1U);
    EXPECT_EQ(delivered->delivery_tick(), 2U);
}

TEST(NetworkEmulator, UsesIndependentDeterministicImpairmentDraws) {
    const auto value = message("alpha", "bravo", "impaired");
    sentinel::core::NetworkEmulator dropped(33, 100);
    dropped.set_profile(profile("loss", 4, 3, 10000, 10000, 10000, 3));
    const auto loss = dropped.step(0, {value});
    ASSERT_EQ(loss.outcomes.size(), 1U);
    EXPECT_EQ(loss.outcomes[0].disposition(), sentinel::v1::NETWORK_DISPOSITION_DROPPED_LOSS);
    EXPECT_GE(loss.outcomes[0].jitter_ticks(), -3);
    EXPECT_LE(loss.outcomes[0].jitter_ticks(), 3);
    EXPECT_EQ(dropped.dropped_messages(), 1U);

    sentinel::core::NetworkEmulator first(33, 100);
    sentinel::core::NetworkEmulator second(33, 100);
    const auto impaired = profile("reorder", 4, 3, 0, 10000, 10000, 3);
    first.set_profile(impaired);
    second.set_profile(impaired);
    auto left = first.step(0, {value});
    auto right = second.step(0, {value});
    for (std::uint64_t tick = 1; left.delivered.empty() && tick < 12; ++tick) {
        left = first.step(tick, {});
        right = second.step(tick, {});
    }
    ASSERT_EQ(left.delivered.size(), 1U);
    const auto* delivered = disposition(left, sentinel::v1::NETWORK_DISPOSITION_DELIVERED, 1);
    const auto* repeated = disposition(right, sentinel::v1::NETWORK_DISPOSITION_DELIVERED, 1);
    ASSERT_NE(delivered, nullptr);
    ASSERT_NE(repeated, nullptr);
    EXPECT_TRUE(delivered->reordered());
    EXPECT_GE(delivered->delivery_tick(), 2U);
    EXPECT_LE(delivered->delivery_tick(), 10U);
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(*delivered),
              sentinel::protocol::deterministic_bytes(*repeated));
    EXPECT_EQ(first.reordered_messages(), 1U);
}

TEST(NetworkEmulator, PartitionFlushesInflightPackets) {
    sentinel::core::NetworkEmulator network(45, 100);
    network.set_profile(profile("slow", 4, 0, 0, 10000));
    const auto queued = network.step(0, {message("alpha", "bravo", "before")});
    EXPECT_TRUE(queued.delivered.empty());
    const auto cut = network.set_link_blocked("bravo", "alpha", true, 1);
    ASSERT_EQ(cut.size(), 1U);
    EXPECT_EQ(cut[0].sequence(), 1U);
    EXPECT_EQ(cut[0].disposition(), sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION);
    EXPECT_EQ(cut[0].delivery_tick(), 4U);
    EXPECT_TRUE(network.link_blocked("alpha", "bravo"));
    EXPECT_TRUE(network.link_blocked("bravo", "alpha"));

    const auto blocked = network.step(1, {message("alpha", "bravo", "out"),
                                          message("bravo", "alpha", "back")});
    EXPECT_TRUE(blocked.delivered.empty());
    ASSERT_EQ(blocked.outcomes.size(), 2U);
    EXPECT_EQ(blocked.outcomes[0].disposition(), sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION);
    EXPECT_EQ(blocked.outcomes[1].disposition(), sentinel::v1::NETWORK_DISPOSITION_DROPPED_PARTITION);
    EXPECT_EQ(network.dropped_messages(), 3U);

    EXPECT_TRUE(network.set_link_blocked("alpha", "bravo", false, 2).empty());
    EXPECT_FALSE(network.link_blocked("alpha", "bravo"));
    network.set_profile(profile("healed", 0, 0, 0, 10000));
    const auto healed = network.step(2, {message("alpha", "bravo", "retry")});
    ASSERT_EQ(healed.delivered.size(), 1U);
    EXPECT_EQ(healed.delivered[0].payload(), "retry");
}

TEST(NetworkEmulator, ProfileChangesDoNotRewriteInflightPackets) {
    sentinel::core::NetworkEmulator network(57, 100);
    network.set_profile(profile("old", 4, 0, 0, 10000));
    network.step(0, {message("alpha", "bravo", "old")});
    network.set_profile(profile("new", 0, 0, 0, 10000));
    const auto fast = network.step(1, {message("alpha", "bravo", "new")});
    ASSERT_EQ(fast.delivered.size(), 1U);
    const auto* new_delivery = disposition(fast, sentinel::v1::NETWORK_DISPOSITION_DELIVERED, 2);
    ASSERT_NE(new_delivery, nullptr);
    EXPECT_EQ(new_delivery->profile_id(), "new");
    EXPECT_EQ(new_delivery->delivery_tick(), 2U);

    network.step(2, {});
    const auto slow = network.step(3, {});
    ASSERT_EQ(slow.delivered.size(), 1U);
    const auto* old_delivery = disposition(slow, sentinel::v1::NETWORK_DISPOSITION_DELIVERED, 1);
    ASSERT_NE(old_delivery, nullptr);
    EXPECT_EQ(old_delivery->profile_id(), "old");
    EXPECT_EQ(old_delivery->delivery_tick(), 4U);
}

TEST(NetworkEmulator, RejectsInvalidInputs) {
    sentinel::core::NetworkEmulator network(1, 100);
    EXPECT_THROW(network.set_profile(profile("bad", 0, 0, 0, 0)), std::invalid_argument);
    EXPECT_THROW(network.set_profile(profile("bad-reorder", 0, 0, 0, 100, 1, 0)), std::invalid_argument);
    network.set_profile(profile("perfect", 0, 0, 0, 100));
    EXPECT_THROW(network.step(0, {message("alpha", "alpha", "self")}), std::invalid_argument);
    EXPECT_THROW(network.set_link_blocked("alpha", "alpha", true, 0), std::invalid_argument);
}
