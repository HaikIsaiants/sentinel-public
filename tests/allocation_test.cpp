#include <sentinel/agent/allocation.hpp>
#include <sentinel/protocol/framing.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

sentinel::v1::TaskState task(std::string id, std::int64_t x, std::uint32_t priority = 1) {
    sentinel::v1::TaskState value;
    value.set_id(std::move(id));
    value.set_kind("search");
    value.mutable_target()->set_x_mm(x);
    value.set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    value.set_deadline_tick(100);
    value.set_completion_radius_mm(100);
    value.set_status(sentinel::v1::TASK_STATUS_PENDING);
    value.set_service_ticks(1);
    value.set_service_energy_mj_per_tick(100);
    value.set_priority(priority);
    return value;
}

sentinel::v1::AgentObservation observation(std::string id, std::int64_t x,
                                           sentinel::v1::AllocationPolicy policy,
                                           const std::vector<sentinel::v1::TaskState>& tasks,
                                           std::uint64_t tick = 0, std::uint64_t map_version = 1) {
    sentinel::v1::AgentObservation value;
    value.set_tick(tick);
    value.set_step_ms(1000);
    value.set_allocation_epoch(1);
    value.set_allocation_policy(policy);
    value.mutable_world()->set_width_mm(10000);
    value.mutable_world()->set_height_mm(4000);
    value.mutable_world()->set_grid_cell_mm(1000);
    value.mutable_world()->set_map_version(map_version);
    auto* self = value.mutable_self();
    self->set_id(std::move(id));
    self->set_kind("uav");
    self->mutable_position()->set_x_mm(x);
    self->set_max_speed_mm_s(1000);
    self->set_initial_energy_mj(1000000);
    self->set_energy_mj(1000000);
    self->set_energy_cost_mj_per_meter(1000);
    self->set_payload_grams(1000);
    self->set_active(true);
    self->add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    self->add_terrain_access("air");
    for (const auto& known : tasks) {
        value.add_known_tasks()->CopyFrom(known);
    }
    return value;
}

sentinel::v1::NetworkMessage message_to(const sentinel::agent::AllocationResult& result,
                                        const std::string& recipient) {
    const auto position = std::find_if(result.outgoing_messages.begin(), result.outgoing_messages.end(),
                                       [&recipient](const auto& message) {
                                           return message.recipient_id() == recipient;
                                       });
    if (position == result.outgoing_messages.end()) {
        return {};
    }
    return *position;
}

sentinel::v1::NetworkMessage message(const sentinel::v1::AllocationState& state,
                                     const std::string& recipient) {
    sentinel::v1::NetworkMessage value;
    value.set_sender_id(state.sender_id());
    value.set_recipient_id(recipient);
    value.set_version(state.version());
    value.set_payload(sentinel::protocol::deterministic_bytes(state));
    return value;
}

sentinel::v1::AllocationState nearest_state(const std::string& sender, std::uint64_t version,
                                            const std::string& task_id, std::int64_t distance,
                                            const sentinel::v1::AllocationBid* winner = nullptr,
                                            std::uint64_t map_version = 1) {
    sentinel::v1::AllocationState state;
    state.set_epoch(1);
    state.set_version(version);
    state.set_sender_id(sender);
    state.set_map_version(map_version);
    auto* bid = state.add_bids();
    bid->set_epoch(1);
    bid->set_version(version);
    bid->set_task_id(task_id);
    bid->set_bidder_id(sender);
    bid->set_distance_mm(distance);
    bid->set_energy_mj(distance);
    bid->set_completion_tick(10);
    state.add_winners()->CopyFrom(winner ? *winner : *bid);
    auto* relay = state.add_winner_relays();
    relay->mutable_bid()->CopyFrom(state.winners(0));
    relay->add_path_agent_ids(state.winners(0).bidder_id());
    if (state.winners(0).bidder_id() != sender) {
        relay->add_path_agent_ids(sender);
    }
    return state;
}

sentinel::v1::AllocationBid nearest_bid(const std::string& bidder, std::uint64_t version,
                                        const std::string& task_id, std::int64_t distance) {
    return nearest_state(bidder, version, task_id, distance).bids(0);
}

sentinel::v1::AllocationState relay_state(const std::string& sender, std::uint64_t version,
                                          const sentinel::v1::AllocationBid& bid) {
    sentinel::v1::AllocationState state;
    state.set_epoch(1);
    state.set_version(version);
    state.set_sender_id(sender);
    state.set_map_version(1);
    state.add_winners()->CopyFrom(bid);
    auto* relay = state.add_winner_relays();
    relay->mutable_bid()->CopyFrom(bid);
    relay->add_path_agent_ids(bid.bidder_id());
    if (bid.bidder_id() != sender) {
        relay->add_path_agent_ids(sender);
    }
    return state;
}

sentinel::v1::AllocationState empty_state(const std::string& sender, std::uint64_t version) {
    sentinel::v1::AllocationState state;
    state.set_epoch(1);
    state.set_version(version);
    state.set_sender_id(sender);
    state.set_map_version(1);
    return state;
}

void topology(sentinel::v1::AgentObservation& observation, const std::vector<std::string>& members,
              const std::vector<std::string>& reachable) {
    for (const auto& member : members) {
        observation.add_peer_ids(member);
    }
    for (const auto& peer : reachable) {
        observation.add_reachable_peer_ids(peer);
    }
}

const sentinel::v1::AllocationBid& winner(const sentinel::v1::AllocationState& state,
                                          const std::string& task_id) {
    const auto position = std::find_if(state.winners().begin(), state.winners().end(),
                                       [&task_id](const auto& bid) {
                                           return bid.task_id() == task_id;
                                       });
    if (position == state.winners().end()) {
        throw std::runtime_error("winner missing");
    }
    return *position;
}

}

TEST(Allocator, NearestConsensusCommitsOnlyAfterMatchingPeerViews) {
    const std::vector tasks{task("target", 3000)};
    sentinel::agent::Allocator alpha("alpha");
    sentinel::agent::Allocator beta("beta");
    auto alpha_input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    auto beta_input = observation("beta", 4000, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    alpha_input.add_peer_ids("alpha");
    alpha_input.add_peer_ids("beta");
    beta_input.add_peer_ids("alpha");
    beta_input.add_peer_ids("beta");

    const auto alpha_first = alpha.update(alpha_input);
    const auto beta_first = beta.update(beta_input);
    EXPECT_TRUE(alpha_first.pending);
    EXPECT_TRUE(beta_first.pending);
    EXPECT_TRUE(alpha_first.commits.empty());
    EXPECT_TRUE(beta_first.commits.empty());

    alpha_input.set_tick(1);
    beta_input.set_tick(1);
    alpha_input.add_delivered_messages()->CopyFrom(message_to(beta_first, "alpha"));
    beta_input.add_delivered_messages()->CopyFrom(message_to(alpha_first, "beta"));
    const auto alpha_second = alpha.update(alpha_input);
    const auto beta_second = beta.update(beta_input);
    EXPECT_EQ(winner(alpha_second.state, "target").bidder_id(), "beta");
    EXPECT_EQ(winner(beta_second.state, "target").bidder_id(), "beta");
    EXPECT_TRUE(beta_second.commits.empty());
    EXPECT_EQ(beta_second.state.version(), beta_first.state.version());
    EXPECT_EQ(message_to(beta_second, "alpha").payload(), message_to(beta_first, "alpha").payload());

    alpha_input.clear_delivered_messages();
    beta_input.clear_delivered_messages();
    alpha_input.set_tick(2);
    beta_input.set_tick(2);
    alpha_input.add_delivered_messages()->CopyFrom(message_to(beta_second, "alpha"));
    beta_input.add_delivered_messages()->CopyFrom(message_to(alpha_second, "beta"));
    const auto alpha_third = alpha.update(alpha_input);
    const auto beta_third = beta.update(beta_input);
    EXPECT_FALSE(alpha_third.pending);
    EXPECT_FALSE(beta_third.pending);
    ASSERT_EQ(beta_third.commits.size(), 1U);
    EXPECT_EQ(beta_third.commits.front().agent_id(), "beta");
    EXPECT_EQ(beta_third.commits.front().task_id(), "target");
    EXPECT_EQ(beta_third.commits.front().distance_mm(), 1000);
    EXPECT_EQ(beta_third.commits.front().version(), winner(beta_third.state, "target").version());
}

TEST(Allocator, RetransmitsUnchangedStateEveryHalfSecond) {
    const std::vector tasks{task("target", 3000)};
    sentinel::agent::Allocator allocator("alpha");
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    input.set_step_ms(100);
    input.add_peer_ids("alpha");
    input.add_peer_ids("beta");

    const auto first = allocator.update(input);
    ASSERT_EQ(first.outgoing_messages.size(), 1U);

    for (std::uint64_t tick = 1; tick < 5; ++tick) {
        input.set_tick(tick);
        EXPECT_TRUE(allocator.update(input).outgoing_messages.empty());
    }

    input.set_tick(5);
    const auto retry = allocator.update(input);
    ASSERT_EQ(retry.outgoing_messages.size(), 1U);
    EXPECT_EQ(retry.outgoing_messages.front().version(), first.outgoing_messages.front().version());
    EXPECT_EQ(retry.outgoing_messages.front().payload(), first.outgoing_messages.front().payload());

    input.set_tick(6);
    const auto beta = nearest_state("beta", 1, "target", 0);
    input.add_delivered_messages()->CopyFrom(message(beta, "alpha"));
    const auto changed = allocator.update(input);
    ASSERT_EQ(changed.outgoing_messages.size(), 1U);
    EXPECT_GT(changed.outgoing_messages.front().version(), retry.outgoing_messages.front().version());
}

TEST(Allocator, RetransmissionRecoversDroppedState) {
    const std::vector tasks{task("target", 3000)};
    sentinel::agent::Allocator alpha("alpha");
    sentinel::agent::Allocator beta("beta");
    auto alpha_input = observation("alpha", 2000, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    auto beta_input = observation("beta", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    alpha_input.set_step_ms(100);
    beta_input.set_step_ms(100);
    topology(alpha_input, {"alpha", "beta"}, {"alpha", "beta"});
    topology(beta_input, {"alpha", "beta"}, {"alpha", "beta"});

    const auto alpha_first = alpha.update(alpha_input);
    const auto beta_first = beta.update(beta_input);
    EXPECT_EQ(winner(alpha_first.state, "target").bidder_id(), "alpha");
    EXPECT_EQ(winner(beta_first.state, "target").bidder_id(), "beta");

    alpha_input.set_tick(1);
    alpha_input.add_delivered_messages()->CopyFrom(message_to(beta_first, "alpha"));
    EXPECT_TRUE(alpha.update(alpha_input).outgoing_messages.empty());
    alpha_input.clear_delivered_messages();
    beta_input.set_tick(1);
    EXPECT_TRUE(beta.update(beta_input).outgoing_messages.empty());

    for (std::uint64_t tick = 2; tick < 5; ++tick) {
        alpha_input.set_tick(tick);
        beta_input.set_tick(tick);
        EXPECT_TRUE(alpha.update(alpha_input).outgoing_messages.empty());
        EXPECT_TRUE(beta.update(beta_input).outgoing_messages.empty());
    }

    alpha_input.set_tick(5);
    beta_input.set_tick(5);
    const auto alpha_retry = alpha.update(alpha_input);
    ASSERT_EQ(alpha_retry.outgoing_messages.size(), 1U);
    EXPECT_EQ(beta.update(beta_input).outgoing_messages.size(), 1U);

    beta_input.set_tick(6);
    beta_input.add_delivered_messages()->CopyFrom(message_to(alpha_retry, "beta"));
    const auto beta_changed = beta.update(beta_input);
    ASSERT_EQ(beta_changed.outgoing_messages.size(), 1U);
    EXPECT_EQ(winner(beta_changed.state, "target").bidder_id(), "alpha");

    alpha_input.set_tick(7);
    alpha_input.add_delivered_messages()->CopyFrom(message_to(beta_changed, "alpha"));
    const auto alpha_final = alpha.update(alpha_input);
    ASSERT_EQ(alpha_final.commits.size(), 1U);
    EXPECT_EQ(alpha_final.commits.front().agent_id(), "alpha");
}

TEST(Allocator, PartitionedComponentCommitsWithReachableConsensus) {
    const std::vector tasks{task("target", 3000)};
    sentinel::agent::Allocator allocator("alpha");
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    input.add_peer_ids("alpha");
    input.add_peer_ids("beta");
    input.add_peer_ids("gamma");
    input.add_reachable_peer_ids("alpha");
    input.add_reachable_peer_ids("beta");
    const auto first = allocator.update(input);

    input.set_tick(1);
    const auto beta = nearest_state("beta", 1, "target", 4000, &winner(first.state, "target"));
    input.add_delivered_messages()->CopyFrom(message(beta, "alpha"));
    const auto partitioned = allocator.update(input);
    EXPECT_FALSE(partitioned.pending);
    ASSERT_EQ(partitioned.commits.size(), 1U);
    EXPECT_EQ(partitioned.commits.front().agent_id(), "alpha");
}

TEST(Allocator, LearnsAndClearsOwnerOnlyFromDirectPeerState) {
    const std::vector tasks{task("target", 3000)};
    sentinel::agent::Allocator allocator("alpha");
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    input.add_peer_ids("alpha");
    input.add_peer_ids("beta");
    input.add_reachable_peer_ids("alpha");
    input.add_reachable_peer_ids("beta");
    allocator.update(input);

    auto owned = nearest_state("beta", 2, "target", 2000);
    auto* owner = owned.add_owners();
    owner->set_epoch(1);
    owner->set_version(1);
    owner->set_task_id("target");
    owner->set_agent_id("beta");
    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(message(owned, "alpha"));
    const auto learned = allocator.update(input);
    ASSERT_EQ(learned.state.owners_size(), 1);
    EXPECT_EQ(learned.state.owners(0).agent_id(), "beta");
    EXPECT_TRUE(learned.commits.empty());

    input.set_tick(2);
    input.clear_delivered_messages();
    input.add_delivered_messages()->CopyFrom(message(nearest_state("beta", 3, "target", 2000), "alpha"));
    const auto cleared = allocator.update(input);
    EXPECT_TRUE(cleared.state.owners().empty());
}

TEST(Allocator, ChoosesNewestDirectStateIndependentOfDeliveryOrder) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    input.add_peer_ids("beta");
    sentinel::agent::Allocator forward("alpha");
    sentinel::agent::Allocator reverse("alpha");
    forward.update(input);
    reverse.update(input);
    const auto older = message(nearest_state("beta", 1, "target", 1500), "alpha");
    const auto newer = message(nearest_state("beta", 2, "target", 1000), "alpha");

    auto first_order = input;
    first_order.set_tick(1);
    first_order.add_delivered_messages()->CopyFrom(older);
    first_order.add_delivered_messages()->CopyFrom(newer);
    auto second_order = input;
    second_order.set_tick(1);
    second_order.add_delivered_messages()->CopyFrom(newer);
    second_order.add_delivered_messages()->CopyFrom(older);
    const auto first = forward.update(first_order);
    const auto second = reverse.update(second_order);
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(first.state),
              sentinel::protocol::deterministic_bytes(second.state));
    EXPECT_EQ(winner(first.state, "target").bidder_id(), "beta");
    EXPECT_EQ(winner(first.state, "target").version(), 2U);
}

void expect_winner_across_connected_line(sentinel::v1::AllocationPolicy policy) {
    const std::vector tasks{task("target", 3000)};
    const std::vector<std::string> members{"alpha", "bravo", "charlie"};
    auto alpha_input = observation("alpha", 0, policy, tasks);
    auto bravo_input = observation("bravo", 2000, policy, tasks);
    auto charlie_input = observation("charlie", 3000, policy, tasks);
    topology(alpha_input, members, {"bravo"});
    topology(bravo_input, members, {"alpha", "charlie"});
    topology(charlie_input, members, {"bravo"});
    sentinel::agent::Allocator alpha("alpha");
    sentinel::agent::Allocator bravo("bravo");
    sentinel::agent::Allocator charlie("charlie");
    auto alpha_result = alpha.update(alpha_input);
    auto bravo_result = bravo.update(bravo_input);
    auto charlie_result = charlie.update(charlie_input);

    for (std::uint64_t tick = 1; tick <= 4; ++tick) {
        alpha_input.set_tick(tick);
        bravo_input.set_tick(tick);
        charlie_input.set_tick(tick);
        alpha_input.clear_delivered_messages();
        bravo_input.clear_delivered_messages();
        charlie_input.clear_delivered_messages();
        alpha_input.add_delivered_messages()->CopyFrom(message_to(bravo_result, "alpha"));
        bravo_input.add_delivered_messages()->CopyFrom(message_to(alpha_result, "bravo"));
        bravo_input.add_delivered_messages()->CopyFrom(message_to(charlie_result, "bravo"));
        charlie_input.add_delivered_messages()->CopyFrom(message_to(bravo_result, "charlie"));
        alpha_result = alpha.update(alpha_input);
        bravo_result = bravo.update(bravo_input);
        charlie_result = charlie.update(charlie_input);
    }

    EXPECT_EQ(winner(alpha_result.state, "target").bidder_id(), "charlie");
    EXPECT_EQ(winner(bravo_result.state, "target").bidder_id(), "charlie");
    EXPECT_EQ(winner(charlie_result.state, "target").bidder_id(), "charlie");
    EXPECT_FALSE(alpha_result.pending);
    EXPECT_FALSE(bravo_result.pending);
    EXPECT_FALSE(charlie_result.pending);
    EXPECT_TRUE(alpha_result.commits.empty());
    EXPECT_TRUE(bravo_result.commits.empty());
    ASSERT_EQ(charlie_result.commits.size(), 1U);
    EXPECT_EQ(charlie_result.commits.front().agent_id(), "charlie");
}

TEST(Allocator, RelaysNearestWinnerAcrossConnectedLine) {
    expect_winner_across_connected_line(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
}

TEST(Allocator, RelaysCbbaWinnerAcrossConnectedLine) {
    expect_winner_across_connected_line(sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA);
}

TEST(Allocator, NewestRelayedClaimReplacesBetterStaleClaim) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    topology(input, {"alpha", "bravo", "charlie", "delta"}, {"bravo", "delta"});
    sentinel::agent::Allocator allocator("alpha");
    allocator.update(input);

    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(
        message(relay_state("bravo", 5, nearest_bid("charlie", 1, "target", 500)), "alpha"));
    const auto stale = allocator.update(input);
    EXPECT_EQ(winner(stale.state, "target").bidder_id(), "charlie");

    input.set_tick(2);
    input.clear_delivered_messages();
    input.add_delivered_messages()->CopyFrom(
        message(relay_state("delta", 5, nearest_bid("charlie", 2, "target", 2500)), "alpha"));
    const auto result = allocator.update(input);
    EXPECT_EQ(winner(result.state, "target").bidder_id(), "alpha");
}

TEST(Allocator, DuplicateRelaysAreOrderIndependent) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    topology(input, {"alpha", "bravo", "charlie", "delta"}, {"bravo", "delta"});
    sentinel::agent::Allocator forward("alpha");
    sentinel::agent::Allocator reverse("alpha");
    forward.update(input);
    reverse.update(input);
    const auto claim = nearest_bid("charlie", 2, "target", 500);
    const auto bravo = message(relay_state("bravo", 5, claim), "alpha");
    const auto delta = message(relay_state("delta", 5, claim), "alpha");
    auto first = input;
    first.set_tick(1);
    first.add_delivered_messages()->CopyFrom(bravo);
    first.add_delivered_messages()->CopyFrom(delta);
    auto second = input;
    second.set_tick(1);
    second.add_delivered_messages()->CopyFrom(delta);
    second.add_delivered_messages()->CopyFrom(bravo);
    const auto first_result = forward.update(first);
    const auto second_result = reverse.update(second);
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(first_result.state),
              sentinel::protocol::deterministic_bytes(second_result.state));
    EXPECT_EQ(winner(first_result.state, "target").bidder_id(), "charlie");
}

TEST(Allocator, ConflictingRelaysRequireDirectOrigin) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    topology(input, {"alpha", "bravo", "charlie", "delta"}, {"bravo", "delta"});
    sentinel::agent::Allocator allocator("alpha");
    allocator.update(input);

    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(
        message(relay_state("bravo", 5, nearest_bid("charlie", 2, "target", 500)), "alpha"));
    input.add_delivered_messages()->CopyFrom(
        message(relay_state("delta", 5, nearest_bid("charlie", 2, "target", 600)), "alpha"));
    const auto conflicted = allocator.update(input);
    EXPECT_EQ(winner(conflicted.state, "target").bidder_id(), "alpha");

    input.set_tick(2);
    input.add_reachable_peer_ids("charlie");
    input.clear_delivered_messages();
    input.add_delivered_messages()->CopyFrom(message(nearest_state("charlie", 2, "target", 700), "alpha"));
    const auto resolved = allocator.update(input);
    EXPECT_EQ(winner(resolved.state, "target").bidder_id(), "charlie");
    EXPECT_EQ(winner(resolved.state, "target").distance_mm(), 700);
}

TEST(Allocator, RelayedSelfDoesNotRestoreWithdrawnBid) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    input.add_peer_ids("beta");
    sentinel::agent::Allocator allocator("alpha");
    const auto initial = allocator.update(input);
    const auto withdrawn = winner(initial.state, "target");

    input.set_tick(1);
    input.mutable_known_tasks(0)->set_required_capability(sentinel::v1::CAPABILITY_INSPECTION);
    input.add_delivered_messages()->CopyFrom(message(relay_state("beta", 2, withdrawn), "alpha"));
    const auto result = allocator.update(input);
    EXPECT_TRUE(result.state.bids().empty());
    EXPECT_TRUE(result.state.winners().empty());
    EXPECT_TRUE(result.commits.empty());
    EXPECT_TRUE(result.pending);
}

TEST(Allocator, DirectWithdrawalIsVersionWatermark) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    topology(input, {"alpha", "bravo", "charlie", "delta"}, {"bravo", "charlie", "delta"});
    sentinel::agent::Allocator allocator("alpha");
    allocator.update(input);

    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(message(empty_state("charlie", 3), "alpha"));
    input.add_delivered_messages()->CopyFrom(
        message(relay_state("bravo", 4, nearest_bid("charlie", 2, "target", 100)), "alpha"));
    const auto withdrawn = allocator.update(input);
    EXPECT_EQ(winner(withdrawn.state, "target").bidder_id(), "alpha");

    input.set_tick(2);
    input.clear_delivered_messages();
    input.add_delivered_messages()->CopyFrom(message(empty_state("charlie", 3), "alpha"));
    input.add_delivered_messages()->CopyFrom(
        message(relay_state("delta", 4, nearest_bid("charlie", 4, "target", 100)), "alpha"));
    const auto newer = allocator.update(input);
    EXPECT_EQ(winner(newer.state, "target").bidder_id(), "charlie");
    EXPECT_EQ(winner(newer.state, "target").version(), 4U);
}

TEST(Allocator, LocalFailureInvalidatesCachedRelay) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    topology(input, {"alpha", "bravo", "charlie"}, {"bravo"});
    sentinel::agent::Allocator allocator("alpha");
    allocator.update(input);

    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(
        message(relay_state("bravo", 3, nearest_bid("charlie", 2, "target", 100)), "alpha"));
    const auto relayed = allocator.update(input);
    EXPECT_EQ(winner(relayed.state, "target").bidder_id(), "charlie");

    input.set_tick(2);
    input.clear_delivered_messages();
    input.add_failure_detections()->set_failed_agent_id("charlie");
    const auto failed = allocator.update(input);
    EXPECT_EQ(winner(failed.state, "target").bidder_id(), "alpha");
}

TEST(Allocator, RetractsStaleWinnerWithoutRelayEcho) {
    auto tasks = std::vector{task("target", 0)};
    const std::vector<std::string> members{"alpha", "bravo", "charlie", "delta"};
    auto alpha_input = observation("alpha", 8000, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    auto bravo_input = observation("bravo", 4000, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    auto charlie_input = observation("charlie", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    auto delta_input = observation("delta", 9000, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    topology(alpha_input, members, {"bravo", "delta"});
    topology(bravo_input, members, {"alpha", "charlie"});
    topology(charlie_input, members, {"bravo"});
    topology(delta_input, members, {"alpha"});
    sentinel::agent::Allocator alpha("alpha");
    sentinel::agent::Allocator bravo("bravo");
    sentinel::agent::Allocator charlie("charlie");
    sentinel::agent::Allocator delta("delta");
    auto alpha_result = alpha.update(alpha_input);
    auto bravo_result = bravo.update(bravo_input);
    auto charlie_result = charlie.update(charlie_input);
    auto delta_result = delta.update(delta_input);

    const auto advance = [&](std::uint64_t tick) {
        alpha_input.set_tick(tick);
        bravo_input.set_tick(tick);
        charlie_input.set_tick(tick);
        delta_input.set_tick(tick);
        alpha_input.clear_delivered_messages();
        bravo_input.clear_delivered_messages();
        charlie_input.clear_delivered_messages();
        delta_input.clear_delivered_messages();
        alpha_input.add_delivered_messages()->CopyFrom(message_to(bravo_result, "alpha"));
        alpha_input.add_delivered_messages()->CopyFrom(message_to(delta_result, "alpha"));
        bravo_input.add_delivered_messages()->CopyFrom(message_to(alpha_result, "bravo"));
        bravo_input.add_delivered_messages()->CopyFrom(message_to(charlie_result, "bravo"));
        charlie_input.add_delivered_messages()->CopyFrom(message_to(bravo_result, "charlie"));
        delta_input.add_delivered_messages()->CopyFrom(message_to(alpha_result, "delta"));
        alpha_result = alpha.update(alpha_input);
        bravo_result = bravo.update(bravo_input);
        charlie_result = charlie.update(charlie_input);
        delta_result = delta.update(delta_input);
    };
    for (std::uint64_t tick = 1; tick <= 5; ++tick) {
        advance(tick);
    }
    EXPECT_EQ(winner(delta_result.state, "target").bidder_id(), "charlie");

    alpha_input.mutable_known_tasks(0)->mutable_target()->set_x_mm(4000);
    bravo_input.mutable_known_tasks(0)->mutable_target()->set_x_mm(4000);
    charlie_input.mutable_known_tasks(0)->mutable_target()->set_x_mm(4000);
    delta_input.mutable_known_tasks(0)->mutable_target()->set_x_mm(4000);
    for (std::uint64_t tick = 6; tick <= 14; ++tick) {
        advance(tick);
    }
    EXPECT_EQ(winner(alpha_result.state, "target").bidder_id(), "bravo");
    EXPECT_EQ(winner(bravo_result.state, "target").bidder_id(), "bravo");
    EXPECT_EQ(winner(charlie_result.state, "target").bidder_id(), "bravo");
    EXPECT_EQ(winner(delta_result.state, "target").bidder_id(), "bravo");
    EXPECT_FALSE(alpha_result.pending);
    EXPECT_FALSE(bravo_result.pending);
    EXPECT_FALSE(charlie_result.pending);
    EXPECT_FALSE(delta_result.pending);
}

TEST(Allocator, RejectsMalformedRelayProvenance) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    topology(input, {"alpha", "bravo", "charlie", "delta", "echo", "foxtrot"},
             {"bravo", "charlie", "delta", "echo", "foxtrot"});
    sentinel::agent::Allocator allocator("alpha");
    const auto initial = allocator.update(input);

    auto missing = relay_state("bravo", 2, nearest_bid("delta", 1, "target", 100));
    missing.clear_winner_relays();
    auto mismatch = relay_state("charlie", 2, nearest_bid("delta", 1, "target", 100));
    mismatch.mutable_winner_relays(0)->mutable_bid()->set_distance_mm(101);
    auto loop = relay_state("delta", 2, nearest_bid("echo", 1, "target", 100));
    loop.mutable_winner_relays(0)->add_path_agent_ids("delta");
    const auto unknown = relay_state("echo", 2, nearest_bid("zulu", 1, "target", 100));
    auto too_new = nearest_state("foxtrot", 2, "target", 100);
    too_new.mutable_bids(0)->set_version(3);
    too_new.mutable_winners(0)->set_version(3);
    too_new.mutable_winner_relays(0)->mutable_bid()->set_version(3);
    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(message(missing, "alpha"));
    input.add_delivered_messages()->CopyFrom(message(mismatch, "alpha"));
    input.add_delivered_messages()->CopyFrom(message(loop, "alpha"));
    input.add_delivered_messages()->CopyFrom(message(unknown, "alpha"));
    input.add_delivered_messages()->CopyFrom(message(too_new, "alpha"));
    const auto result = allocator.update(input);
    EXPECT_EQ(winner(result.state, "target").bidder_id(), "alpha");
    EXPECT_EQ(result.state.version(), initial.state.version());
    EXPECT_TRUE(result.commits.empty());
}

TEST(Allocator, BadRelayStates) {
    const std::vector tasks{task("target", 2000)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    input.add_peer_ids("beta");
    sentinel::agent::Allocator allocator("alpha");
    const auto initial = allocator.update(input);

    input.set_tick(1);
    auto malformed = nearest_state("beta", 1, "target", 0);
    auto malformed_message = message(malformed, "alpha");
    malformed_message.set_payload("invalid");
    input.add_delivered_messages()->CopyFrom(malformed_message);
    auto forged = nearest_state("alpha", 1, "target", 0);
    input.add_delivered_messages()->CopyFrom(message(forged, "alpha"));
    auto unlisted = nearest_state("gamma", 1, "target", 0);
    input.add_delivered_messages()->CopyFrom(message(unlisted, "alpha"));
    auto stale_map = nearest_state("beta", 2, "target", 0, nullptr, 2);
    input.add_delivered_messages()->CopyFrom(message(stale_map, "alpha"));
    auto conflict_a = nearest_state("beta", 3, "target", 0);
    auto conflict_b = nearest_state("beta", 3, "target", 1);
    input.add_delivered_messages()->CopyFrom(message(conflict_a, "alpha"));
    input.add_delivered_messages()->CopyFrom(message(conflict_b, "alpha"));
    const auto result = allocator.update(input);
    EXPECT_EQ(winner(result.state, "target").bidder_id(), "alpha");
    EXPECT_EQ(result.state.version(), initial.state.version());
    EXPECT_TRUE(result.pending);
    EXPECT_TRUE(result.commits.empty());
}

TEST(Allocator, CbbaDropsLostTaskAndRebuildsBundle) {
    const std::vector tasks{task("first", 2000, 3), task("second", 4000, 2)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, tasks);
    input.add_peer_ids("beta");
    sentinel::agent::Allocator allocator("alpha");
    const auto initial = allocator.update(input);
    ASSERT_EQ(initial.state.bundle_task_ids_size(), 2);
    const auto lost = initial.state.bundle_task_ids(0);
    const auto own = std::find_if(initial.state.bids().begin(), initial.state.bids().end(),
                                  [&lost](const auto& bid) {
                                      return bid.task_id() == lost;
                                  });
    ASSERT_NE(own, initial.state.bids().end());

    sentinel::v1::AllocationState peer;
    peer.set_epoch(1);
    peer.set_version(1);
    peer.set_sender_id("beta");
    peer.set_map_version(1);
    auto* bid = peer.add_bids();
    bid->CopyFrom(*own);
    bid->set_version(1);
    bid->set_bidder_id("beta");
    bid->set_score(std::numeric_limits<std::int64_t>::max());
    bid->set_bundle_position(0);
    peer.add_winners()->CopyFrom(*bid);
    auto* relay = peer.add_winner_relays();
    relay->mutable_bid()->CopyFrom(*bid);
    relay->add_path_agent_ids("beta");
    peer.add_bundle_task_ids(lost);
    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(message(peer, "alpha"));
    const auto result = allocator.update(input);
    EXPECT_EQ(winner(result.state, lost).bidder_id(), "beta");
    EXPECT_EQ(std::find(result.state.bundle_task_ids().begin(), result.state.bundle_task_ids().end(), lost),
              result.state.bundle_task_ids().end());
    EXPECT_EQ(std::find_if(result.state.bids().begin(), result.state.bids().end(),
                           [&lost](const auto& value) {
                               return value.task_id() == lost;
                           }),
              result.state.bids().end());
    std::vector<std::uint32_t> positions;
    for (const auto& id : result.state.bundle_task_ids()) {
        const auto position = std::find_if(result.state.bids().begin(), result.state.bids().end(),
                                           [&id](const auto& value) {
                                               return value.task_id() == id;
                                           });
        ASSERT_NE(position, result.state.bids().end());
        positions.push_back(position->bundle_position());
    }
    std::sort(positions.begin(), positions.end());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        EXPECT_EQ(positions[i], i);
    }
}

TEST(Allocator, CbbaSeparatesBuildOrderFromExecutionOrder) {
    const std::vector tasks{task("far", 8000, 3), task("middle", 4000, 2), task("near", 1000, 1)};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, tasks);
    input.add_peer_ids("beta");
    sentinel::agent::Allocator allocator("alpha");
    const auto initial = allocator.update(input);
    ASSERT_EQ(initial.state.bundle_task_ids_size(), 3);
    EXPECT_EQ(initial.state.bundle_task_ids(0), "far");
    EXPECT_EQ(initial.state.bundle_task_ids(1), "middle");
    EXPECT_EQ(initial.state.bundle_task_ids(2), "near");
    EXPECT_EQ(winner(initial.state, "far").bundle_position(), 2U);
    EXPECT_EQ(winner(initial.state, "near").bundle_position(), 0U);
    EXPECT_EQ(winner(initial.state, "middle").bundle_position(), 1U);
    const auto initial_near = std::find_if(initial.state.bids().begin(), initial.state.bids().end(), [](const auto& bid) {
        return bid.task_id() == "near";
    });
    ASSERT_NE(initial_near, initial.state.bids().end());

    sentinel::v1::AllocationState peer;
    peer.set_epoch(1);
    peer.set_version(1);
    peer.set_sender_id("beta");
    peer.set_map_version(1);
    const auto middle = std::find_if(initial.state.bids().begin(), initial.state.bids().end(), [](const auto& bid) {
        return bid.task_id() == "middle";
    });
    ASSERT_NE(middle, initial.state.bids().end());
    auto* bid = peer.add_bids();
    bid->CopyFrom(*middle);
    bid->set_version(1);
    bid->set_bidder_id("beta");
    bid->set_score(std::numeric_limits<std::int64_t>::max());
    bid->set_bundle_position(0);
    peer.add_winners()->CopyFrom(*bid);
    auto* relay = peer.add_winner_relays();
    relay->mutable_bid()->CopyFrom(*bid);
    relay->add_path_agent_ids("beta");
    peer.add_bundle_task_ids("middle");
    input.set_tick(1);
    input.add_delivered_messages()->CopyFrom(message(peer, "alpha"));
    const auto result = allocator.update(input);
    ASSERT_EQ(result.state.bundle_task_ids_size(), 2);
    EXPECT_EQ(result.state.bundle_task_ids(0), "far");
    EXPECT_EQ(result.state.bundle_task_ids(1), "near");
    EXPECT_EQ(winner(result.state, "middle").bidder_id(), "beta");
    const auto rebuilt_near = std::find_if(result.state.bids().begin(), result.state.bids().end(), [](const auto& value) {
        return value.task_id() == "near";
    });
    ASSERT_NE(rebuilt_near, result.state.bids().end());
    EXPECT_NE(rebuilt_near->score(), initial_near->score());
    EXPECT_GT(rebuilt_near->version(), initial_near->version());
}

TEST(Allocator, AssignedWorkDoesNotMakeConsensusPending) {
    auto assigned = task("assigned", 2000);
    assigned.set_assigned_agent_id("alpha");
    assigned.set_allocation_epoch(1);
    assigned.set_allocation_version(1);
    const std::vector tasks{assigned};
    auto input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    input.add_peer_ids("beta");
    sentinel::agent::Allocator allocator("alpha");
    const auto result = allocator.update(input);
    EXPECT_FALSE(result.pending);
    EXPECT_TRUE(result.commits.empty());
    EXPECT_TRUE(result.state.bids().empty());
    EXPECT_TRUE(result.state.winners().empty());
    ASSERT_EQ(result.state.owners_size(), 1);
    EXPECT_EQ(result.state.owners(0).task_id(), "assigned");
    EXPECT_EQ(result.state.owners(0).agent_id(), "alpha");
}

TEST(Allocator, CbbaAccountsForCommittedExecutionPrefix) {
    auto committed = task("committed", 8000);
    committed.set_assigned_agent_id("alpha");
    committed.set_allocation_epoch(1);
    auto urgent = task("urgent", 1000, 100);
    urgent.set_deadline_tick(5);
    const std::vector tasks{committed, urgent};
    auto cbba_input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA, tasks);
    sentinel::agent::Allocator cbba("alpha");
    const auto constrained = cbba.update(cbba_input);
    EXPECT_TRUE(constrained.state.bids().empty());
    EXPECT_TRUE(constrained.commits.empty());
    EXPECT_TRUE(constrained.pending);

    auto nearest_input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    sentinel::agent::Allocator nearest("alpha");
    const auto direct = nearest.update(nearest_input);
    ASSERT_EQ(direct.commits.size(), 1U);
    EXPECT_EQ(direct.commits.front().task_id(), "urgent");
}

TEST(Allocator, MapChangeResetsConsensusAndRejectsOldState) {
    const std::vector tasks{task("target", 2000)};
    auto first_input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks);
    first_input.add_peer_ids("beta");
    sentinel::agent::Allocator allocator("alpha");
    allocator.update(first_input);
    auto second_input = observation("alpha", 0, sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE, tasks, 1, 2);
    second_input.add_peer_ids("beta");
    second_input.add_delivered_messages()->CopyFrom(message(nearest_state("beta", 1, "target", 0), "alpha"));
    const auto second = allocator.update(second_input);
    EXPECT_EQ(second.state.map_version(), 2U);
    EXPECT_EQ(second.state.version(), 1U);
    EXPECT_EQ(winner(second.state, "target").bidder_id(), "alpha");
    EXPECT_TRUE(second.pending);
}
