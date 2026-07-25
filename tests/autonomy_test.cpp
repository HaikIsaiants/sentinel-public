#include <sentinel/agent/autonomy.hpp>
#include <sentinel/protocol/framing.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

namespace {

sentinel::v1::Envelope observation(std::uint64_t tick, std::uint64_t map_version = 1) {
    sentinel::v1::Envelope result;
    result.set_schema_version(1);
    result.set_sequence(tick + 1);
    result.set_simulation_time_ms(tick * 1000);
    result.set_sender_id("sim");
    result.set_recipient_id("alpha");
    auto* value = result.mutable_observation();
    value->set_tick(tick);
    value->set_step_ms(1000);
    value->set_allocation_epoch(1);
    value->mutable_world()->set_width_mm(4000);
    value->mutable_world()->set_height_mm(4000);
    value->mutable_world()->set_grid_cell_mm(1000);
    value->mutable_world()->set_map_version(map_version);
    auto* self = value->mutable_self();
    self->set_id("alpha");
    self->set_kind("uav");
    self->set_max_speed_mm_s(1000);
    self->set_initial_energy_mj(100000);
    self->set_energy_capacity_mj(100000);
    self->set_energy_mj(100000);
    self->set_energy_cost_mj_per_meter(1000);
    self->set_payload_grams(1000);
    self->set_active(true);
    self->add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    self->add_terrain_access("air");
    return result;
}

sentinel::v1::TaskState task(const char* id, std::int64_t x, std::int64_t y) {
    sentinel::v1::TaskState result;
    result.set_id(id);
    result.set_kind("search");
    result.mutable_target()->set_x_mm(x);
    result.mutable_target()->set_y_mm(y);
    result.set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    result.set_deadline_tick(100);
    result.set_completion_radius_mm(100);
    result.set_status(sentinel::v1::TASK_STATUS_PENDING);
    result.set_service_ticks(1);
    result.set_service_energy_mj_per_tick(100);
    result.set_allocation_epoch(1);
    result.set_allocation_version(1);
    return result;
}

void assign(sentinel::v1::Envelope& envelope, const sentinel::v1::TaskState& value) {
    auto copy = value;
    copy.set_assigned_agent_id("alpha");
    envelope.mutable_observation()->add_assigned_tasks()->CopyFrom(copy);
}

void deliver_state(sentinel::v1::Envelope& envelope, const char* sender, std::uint64_t version = 1) {
    sentinel::v1::AllocationState state;
    state.set_epoch(1);
    state.set_version(version);
    state.set_sender_id(sender);
    state.set_map_version(envelope.observation().world().map_version());
    auto* message = envelope.mutable_observation()->add_delivered_messages();
    message->set_sender_id(sender);
    message->set_recipient_id(envelope.recipient_id());
    message->set_version(version);
    message->set_payload(sentinel::protocol::deterministic_bytes(state));
}

}

TEST(Autonomy, NavigatesThenReportsWork) {
    sentinel::agent::Controller controller("alpha");
    auto first = observation(0);
    assign(first, task("search", 2000, 0));
    const auto navigation = controller.act(first);
    EXPECT_EQ(navigation.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(navigation.action().velocity_x_mm_s(), 1000);
    EXPECT_GT(navigation.action().route_version(), 0U);
    ASSERT_TRUE(navigation.action().has_route_plan());
    EXPECT_EQ(navigation.action().route_plan().version(), navigation.action().route_version());
    EXPECT_EQ(navigation.action().route_plan().map_version(), 1U);
    EXPECT_EQ(navigation.action().route_plan().goal(), "task:search");
    ASSERT_EQ(navigation.action().route_plan().waypoints_size(), 3);
    EXPECT_EQ(navigation.action().route_plan().waypoints(2).x_mm(), 2000);

    auto second = observation(1);
    second.mutable_observation()->mutable_self()->mutable_position()->set_x_mm(2000);
    assign(second, task("search", 2000, 0));
    const auto execution = controller.act(second);
    ASSERT_EQ(execution.action().task_reports_size(), 1);
    EXPECT_EQ(execution.action().task_reports(0).kind(), sentinel::v1::TASK_REPORT_KIND_WORKING);
    EXPECT_EQ(execution.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_EXECUTING);
}

TEST(Autonomy, WaitsForPeerStateBeforeReserving) {
    sentinel::agent::Controller controller("alpha");
    auto first = observation(0);
    first.mutable_observation()->set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    auto* pass = first.mutable_observation()->mutable_world()->add_regions();
    pass->set_id("pass");
    pass->set_kind(sentinel::v1::REGION_KIND_CHOKEPOINT);
    pass->mutable_minimum()->set_x_mm(1000);
    pass->mutable_maximum()->set_x_mm(1000);
    pass->set_energy_multiplier_permille(1000);
    first.mutable_observation()->add_peer_ids("alpha");
    first.mutable_observation()->add_peer_ids("beta");
    first.mutable_observation()->add_peer_ids("gamma");
    first.mutable_observation()->add_reachable_peer_ids("alpha");
    first.mutable_observation()->add_reachable_peer_ids("beta");
    first.mutable_observation()->add_reachable_peer_ids("gamma");
    assign(first, task("search", 2000, 0));
    const auto stale = controller.act(first);
    EXPECT_EQ(stale.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);
    EXPECT_TRUE(stale.action().reservation_proposals().empty());

    auto malformed = first;
    malformed.set_sequence(2);
    malformed.set_simulation_time_ms(1000);
    malformed.mutable_observation()->set_tick(1);
    auto* message = malformed.mutable_observation()->add_delivered_messages();
    message->set_sender_id("beta");
    message->set_recipient_id("alpha");
    const auto rejected = controller.act(malformed);
    EXPECT_TRUE(rejected.action().reservation_proposals().empty());

    auto second = first;
    second.set_sequence(3);
    second.set_simulation_time_ms(2000);
    second.mutable_observation()->set_tick(2);
    deliver_state(second, "beta");
    const auto partial = controller.act(second);
    EXPECT_TRUE(partial.action().reservation_proposals().empty());

    auto third = first;
    third.set_sequence(4);
    third.set_simulation_time_ms(3000);
    third.mutable_observation()->set_tick(3);
    deliver_state(third, "gamma");
    const auto coordinated = controller.act(third);
    ASSERT_EQ(coordinated.action().reservation_proposals_size(), 1);
    EXPECT_EQ(coordinated.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);

    auto fourth = first;
    fourth.set_sequence(5);
    fourth.set_simulation_time_ms(4000);
    fourth.mutable_observation()->set_tick(4);
    fourth.mutable_observation()->clear_reachable_peer_ids();
    fourth.mutable_observation()->add_reachable_peer_ids("alpha");
    fourth.mutable_observation()->add_committed_reservations()->CopyFrom(
        coordinated.action().reservation_proposals(0));
    const auto granted = controller.act(fourth);
    EXPECT_EQ(granted.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(granted.action().velocity_x_mm_s(), 1000);
}

TEST(Autonomy, UnreachablePeerDoesNotBlockNewReservation) {
    sentinel::agent::Controller controller("alpha");
    auto input = observation(0);
    input.mutable_observation()->set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    auto* pass = input.mutable_observation()->mutable_world()->add_regions();
    pass->set_id("pass");
    pass->set_kind(sentinel::v1::REGION_KIND_CHOKEPOINT);
    pass->mutable_minimum()->set_x_mm(1000);
    pass->mutable_maximum()->set_x_mm(1000);
    pass->set_energy_multiplier_permille(1000);
    input.mutable_observation()->add_peer_ids("alpha");
    input.mutable_observation()->add_peer_ids("beta");
    input.mutable_observation()->add_reachable_peer_ids("alpha");
    assign(input, task("search", 2000, 0));
    const auto result = controller.act(input);
    EXPECT_EQ(result.action().reservation_proposals_size(), 1);
    EXPECT_EQ(result.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);
}

TEST(Autonomy, ExchangesAllocationStateAndCommitsAgreedWinner) {
    sentinel::agent::Controller controller("alpha");
    auto first = observation(0);
    first.mutable_observation()->set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    first.mutable_observation()->add_peer_ids("beta");
    first.mutable_observation()->add_known_tasks()->CopyFrom(task("open", 2000, 0));
    const auto first_output = controller.act(first);
    ASSERT_EQ(first_output.action().outgoing_messages_size(), 1);
    ASSERT_EQ(first_output.action().allocation_state().winners_size(), 1);
    EXPECT_EQ(first_output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);

    auto second = observation(1);
    second.mutable_observation()->set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    second.mutable_observation()->add_peer_ids("beta");
    second.mutable_observation()->add_known_tasks()->CopyFrom(task("open", 2000, 0));
    sentinel::v1::AllocationState peer;
    peer.set_epoch(1);
    peer.set_version(1);
    peer.set_sender_id("beta");
    peer.set_map_version(1);
    peer.add_winners()->CopyFrom(first_output.action().allocation_state().winners(0));
    auto* relay = peer.add_winner_relays();
    relay->mutable_bid()->CopyFrom(peer.winners(0));
    relay->add_path_agent_ids("alpha");
    relay->add_path_agent_ids("beta");
    auto* message = second.mutable_observation()->add_delivered_messages();
    message->set_sender_id("beta");
    message->set_recipient_id("alpha");
    message->set_version(1);
    message->set_payload(sentinel::protocol::deterministic_bytes(peer));
    const auto commit_output = controller.act(second);
    ASSERT_EQ(commit_output.action().allocation_commits_size(), 1);
    EXPECT_EQ(commit_output.action().allocation_commits(0).task_id(), "open");
    EXPECT_EQ(commit_output.action().allocation_commits(0).agent_id(), "alpha");
    EXPECT_EQ(commit_output.action().allocation_commits(0).distance_mm(), 2000);
    EXPECT_EQ(commit_output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_IDLE);
}

TEST(Autonomy, ContinuesAssignedWorkDuringAllocationRound) {
    sentinel::agent::Controller controller("alpha");
    auto input = observation(0);
    input.mutable_observation()->set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    auto assigned = task("assigned", 2000, 0);
    assigned.set_assigned_agent_id("alpha");
    assign(input, assigned);
    input.mutable_observation()->add_peer_ids("beta");
    input.mutable_observation()->add_known_tasks()->CopyFrom(assigned);
    input.mutable_observation()->add_known_tasks()->CopyFrom(task("open", 3000, 0));
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 1000);
    EXPECT_TRUE(output.action().task_reports().empty());
}

TEST(Autonomy, PreservesCommittedAllocationEpochOrder) {
    for (const auto policy : {sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE,
                              sentinel::v1::ALLOCATION_POLICY_SENTINEL_CBBA}) {
        sentinel::agent::Controller controller("alpha");
        auto input = observation(0);
        input.mutable_observation()->set_allocation_epoch(2);
        input.mutable_observation()->set_allocation_policy(policy);
        auto older = task("older", 2000, 0);
        older.set_assigned_agent_id("alpha");
        older.set_allocation_epoch(1);
        older.set_bundle_position(1);
        auto newer = task("newer", 0, 0);
        newer.set_assigned_agent_id("alpha");
        newer.set_allocation_epoch(2);
        input.mutable_observation()->add_assigned_tasks()->CopyFrom(newer);
        input.mutable_observation()->add_assigned_tasks()->CopyFrom(older);
        input.mutable_observation()->add_known_tasks()->CopyFrom(newer);
        input.mutable_observation()->add_known_tasks()->CopyFrom(older);
        const auto output = controller.act(input);
        EXPECT_EQ(output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
        EXPECT_EQ(output.action().velocity_x_mm_s(), 1000);
        EXPECT_TRUE(output.action().task_reports().empty());
    }
}

TEST(Autonomy, ChargesWhenTaskReserveIsInsufficient) {
    sentinel::agent::Controller controller("alpha");
    auto input = observation(0);
    input.mutable_observation()->mutable_self()->set_energy_mj(2000);
    auto* charger = input.mutable_observation()->mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(0);
    charger->mutable_position()->set_y_mm(0);
    charger->set_radius_mm(100);
    charger->set_charge_mj_per_tick(10000);
    auto assigned = task("search", 3000, 0);
    assigned.set_service_ticks(5);
    assigned.set_service_energy_mj_per_tick(1000);
    assign(input, assigned);
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().charge_location_id(), "charger");
    EXPECT_EQ(output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_CHARGING);
    EXPECT_TRUE(output.action().task_reports().empty());
}

TEST(Autonomy, UsesChargerWhenDirectRouteExceedsBatteryCapacity) {
    sentinel::agent::Controller controller("alpha");
    auto input = observation(0);
    input.mutable_observation()->mutable_self()->set_initial_energy_mj(3000);
    input.mutable_observation()->mutable_self()->set_energy_capacity_mj(3000);
    input.mutable_observation()->mutable_self()->set_energy_mj(3000);
    auto* charger = input.mutable_observation()->mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(2000);
    charger->set_radius_mm(100);
    charger->set_charge_mj_per_tick(1000);
    assign(input, task("search", 4000, 0));
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 1000);
    EXPECT_TRUE(output.action().task_reports().empty());
}

TEST(Autonomy, RejectsChargerThatCannotMeetDeadline) {
    sentinel::agent::Controller controller("alpha");
    auto input = observation(0);
    input.mutable_observation()->mutable_self()->set_energy_mj(2000);
    auto* charger = input.mutable_observation()->mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->set_radius_mm(100);
    charger->set_charge_mj_per_tick(1000);
    auto assigned = task("search", 3000, 0);
    assigned.set_deadline_tick(4);
    assign(input, assigned);
    const auto output = controller.act(input);
    ASSERT_EQ(output.action().task_reports_size(), 1);
    EXPECT_EQ(output.action().task_reports(0).kind(), sentinel::v1::TASK_REPORT_KIND_REJECTED);
    EXPECT_TRUE(output.action().charge_location_id().empty());
}

TEST(Autonomy, ReplansWhenMapVersionChanges) {
    sentinel::agent::Controller controller("alpha");
    auto first = observation(0);
    assign(first, task("search", 4000, 2000));
    first.mutable_observation()->mutable_self()->mutable_position()->set_y_mm(2000);
    const auto initial = controller.act(first);

    auto second = observation(1);
    second.mutable_observation()->mutable_self()->mutable_position()->set_y_mm(2000);
    assign(second, task("search", 4000, 2000));
    const auto continued = controller.act(second);
    EXPECT_FALSE(continued.action().replanned());
    EXPECT_EQ(continued.action().route_version(), initial.action().route_version());
    EXPECT_EQ(continued.action().velocity_x_mm_s(), initial.action().velocity_x_mm_s());

    auto third = observation(2, 2);
    third.mutable_observation()->mutable_self()->mutable_position()->set_y_mm(2000);
    auto* obstacle = third.mutable_observation()->mutable_world()->add_regions();
    obstacle->set_id("center");
    obstacle->set_kind(sentinel::v1::REGION_KIND_OBSTACLE);
    obstacle->mutable_minimum()->set_x_mm(2000);
    obstacle->mutable_minimum()->set_y_mm(2000);
    obstacle->mutable_maximum()->set_x_mm(2000);
    obstacle->mutable_maximum()->set_y_mm(2000);
    obstacle->set_energy_multiplier_permille(1000);
    assign(third, task("search", 4000, 2000));
    const auto replanned = controller.act(third);
    EXPECT_TRUE(replanned.action().replanned());
    EXPECT_GT(replanned.action().route_version(), initial.action().route_version());
    EXPECT_NE(replanned.action().velocity_y_mm_s(), 0);
}

TEST(Autonomy, RejectsUnreachableAssignedTask) {
    sentinel::agent::Controller controller("alpha");
    auto input = observation(0);
    input.mutable_observation()->mutable_world()->set_width_mm(2000);
    auto* obstacle = input.mutable_observation()->mutable_world()->add_regions();
    obstacle->set_id("wall");
    obstacle->set_kind(sentinel::v1::REGION_KIND_OBSTACLE);
    obstacle->mutable_minimum()->set_x_mm(1000);
    obstacle->mutable_minimum()->set_y_mm(0);
    obstacle->mutable_maximum()->set_x_mm(1000);
    obstacle->mutable_maximum()->set_y_mm(4000);
    obstacle->set_energy_multiplier_permille(1000);
    assign(input, task("blocked", 2000, 0));
    const auto output = controller.act(input);
    ASSERT_EQ(output.action().task_reports_size(), 1);
    EXPECT_EQ(output.action().task_reports(0).kind(), sentinel::v1::TASK_REPORT_KIND_REJECTED);
    EXPECT_EQ(output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);
}

TEST(Autonomy, ReturnsHomeWhenIdle) {
    sentinel::agent::Controller controller("alpha");
    auto input = observation(0);
    input.mutable_observation()->mutable_self()->set_return_location_id("base");
    auto* base = input.mutable_observation()->mutable_world()->add_locations();
    base->set_id("base");
    base->set_kind(sentinel::v1::LOCATION_KIND_RETURN);
    base->mutable_position()->set_x_mm(2000);
    base->mutable_position()->set_y_mm(0);
    base->set_radius_mm(100);
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_RETURNING);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 1000);
}
