#include <sentinel/agent/autonomy.hpp>

#include <gtest/gtest.h>

#include <cstdint>

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

void add_chokepoint(sentinel::v1::Envelope& envelope) {
    auto* region = envelope.mutable_observation()->mutable_world()->add_regions();
    region->set_id("bridge");
    region->set_kind(sentinel::v1::REGION_KIND_CHOKEPOINT);
    region->mutable_minimum()->set_x_mm(1000);
    region->mutable_minimum()->set_y_mm(0);
    region->mutable_maximum()->set_x_mm(2000);
    region->mutable_maximum()->set_y_mm(1000);
    region->set_energy_multiplier_permille(1000);
}

sentinel::v1::FailureDetection detection(const char* failed, std::uint64_t tick) {
    sentinel::v1::FailureDetection result;
    result.set_failed_agent_id(failed);
    result.set_detector_agent_id("alpha");
    result.set_failure_tick(tick - 1);
    result.set_detection_tick(tick);
    return result;
}

}

TEST(RouteCoordination, CopiesEachLocalFailureDetectionOnce) {
    sentinel::agent::Controller controller("alpha");
    auto first = observation(5);
    first.mutable_observation()->add_failure_detections()->CopyFrom(detection("beta", 5));
    first.mutable_observation()->add_failure_detections()->CopyFrom(detection("gamma", 5));
    const auto initial = controller.act(first);
    ASSERT_EQ(initial.action().failure_detections_size(), 2);
    EXPECT_EQ(initial.action().failure_detections(0).failed_agent_id(), "beta");
    EXPECT_EQ(initial.action().failure_detections(1).failed_agent_id(), "gamma");

    auto second = observation(6);
    second.mutable_observation()->add_failure_detections()->CopyFrom(detection("beta", 5));
    second.mutable_observation()->add_failure_detections()->CopyFrom(detection("gamma", 5));
    const auto repeated = controller.act(second);
    EXPECT_TRUE(repeated.action().failure_detections().empty());
}

TEST(RouteCoordination, ReservesChokepointBeforeMovingAndUsesAcceptedGrant) {
    sentinel::agent::Controller controller("alpha");
    auto first = observation(5, 7);
    add_chokepoint(first);
    assign(first, task("search", 3000, 0));
    const auto waiting = controller.act(first);
    ASSERT_EQ(waiting.action().reservation_proposals_size(), 1);
    const auto proposal = waiting.action().reservation_proposals(0);
    EXPECT_EQ(waiting.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);
    EXPECT_EQ(waiting.action().velocity_x_mm_s(), 0);
    EXPECT_EQ(proposal.resource_id(), "bridge");
    EXPECT_EQ(proposal.agent_id(), "alpha");
    EXPECT_GT(proposal.start_tick(), first.observation().tick());
    EXPECT_GE(proposal.end_tick(), proposal.start_tick());
    EXPECT_EQ(proposal.route_version(), waiting.action().route_version());
    EXPECT_EQ(proposal.map_version(), first.observation().world().map_version());

    auto second = observation(proposal.start_tick(), 7);
    add_chokepoint(second);
    assign(second, task("search", 3000, 0));
    second.mutable_observation()->add_committed_reservations()->CopyFrom(proposal);
    const auto moving = controller.act(second);
    EXPECT_EQ(moving.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(moving.action().velocity_x_mm_s(), 1000);
    EXPECT_TRUE(moving.action().reservation_proposals().empty());
}

TEST(RouteCoordination, ReplansAndRetriesAfterMissingReservationGrant) {
    sentinel::agent::Controller controller("alpha");
    auto first = observation(5, 7);
    add_chokepoint(first);
    assign(first, task("search", 3000, 0));
    const auto waiting = controller.act(first);
    ASSERT_EQ(waiting.action().reservation_proposals_size(), 1);
    const auto rejected = waiting.action().reservation_proposals(0);

    auto second = observation(rejected.start_tick(), 7);
    add_chokepoint(second);
    assign(second, task("search", 3000, 0));
    const auto retry = controller.act(second);
    ASSERT_EQ(retry.action().replanning_samples_size(), 1);
    ASSERT_EQ(retry.action().reservation_proposals_size(), 1);
    const auto& sample = retry.action().replanning_samples(0);
    const auto& proposal = retry.action().reservation_proposals(0);
    EXPECT_TRUE(retry.action().replanned());
    EXPECT_EQ(retry.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);
    EXPECT_EQ(sample.agent_id(), "alpha");
    EXPECT_EQ(sample.reason(), sentinel::v1::REPLAN_REASON_RESERVATION);
    EXPECT_EQ(sample.start_tick(), second.observation().tick());
    EXPECT_EQ(sample.end_tick(), second.observation().tick());
    EXPECT_TRUE(sample.wait_plan());
    EXPECT_TRUE(sample.complete());
    EXPECT_EQ(proposal.resource_id(), rejected.resource_id());
    EXPECT_GT(proposal.start_tick(), second.observation().tick());
    EXPECT_GT(proposal.version(), rejected.version());
    EXPECT_EQ(proposal.route_version(), rejected.route_version());
    EXPECT_EQ(proposal.map_version(), rejected.map_version());
}

TEST(RouteCoordination, ReplansForSpeedAndCapacityChanges) {
    for (const auto capacity_change : {false, true}) {
        SCOPED_TRACE(capacity_change);
        sentinel::agent::Controller controller("alpha");
        auto first = observation(0);
        assign(first, task("search", 3000, 0));
        const auto initial = controller.act(first);
        ASSERT_EQ(initial.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);

        auto second = observation(1);
        assign(second, task("search", 3000, 0));
        if (capacity_change) {
            second.mutable_observation()->mutable_self()->set_energy_capacity_mj(50000);
            second.mutable_observation()->mutable_self()->set_energy_mj(50000);
        } else {
            second.mutable_observation()->mutable_self()->set_max_speed_mm_s(500);
        }
        const auto replanned = controller.act(second);
        ASSERT_EQ(replanned.action().replanning_samples_size(), 1);
        const auto& sample = replanned.action().replanning_samples(0);
        EXPECT_TRUE(replanned.action().replanned());
        EXPECT_GT(replanned.action().route_version(), initial.action().route_version());
        EXPECT_EQ(sample.agent_id(), "alpha");
        EXPECT_EQ(sample.reason(), sentinel::v1::REPLAN_REASON_ENDURANCE);
        EXPECT_EQ(sample.start_tick(), second.observation().tick());
        EXPECT_EQ(sample.end_tick(), second.observation().tick());
        EXPECT_FALSE(sample.wait_plan());
        EXPECT_TRUE(sample.complete());
    }
}
