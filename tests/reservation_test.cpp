#include <sentinel/planning/planner.hpp>

#include <gtest/gtest.h>

namespace {

sentinel::planning::Reservation reservation(const char* resource, const char* agent,
                                            std::uint64_t start, std::uint64_t end,
                                            std::uint64_t version = 1, std::uint64_t route_version = 1,
                                            std::uint64_t map_version = 1) {
    return sentinel::planning::Reservation{resource, agent, start, end, version, route_version,
                                           map_version};
}

}

TEST(ReservationTable, UsesInclusiveIntervalsAndUniqueProposals) {
    sentinel::planning::ReservationTable table;
    const auto first = reservation("bridge", "alpha", 10, 12);
    EXPECT_EQ(table.reserve(first), sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(table.reserve(first), sentinel::planning::ReservationResult::unchanged);
    EXPECT_EQ(table.reserve(reservation("bridge", "alpha", 20, 22)),
              sentinel::planning::ReservationResult::stale);
    EXPECT_EQ(table.reserve(reservation("bridge", "beta", 12, 14)),
              sentinel::planning::ReservationResult::rejected);
    EXPECT_EQ(table.reserve(reservation("bridge", "beta", 12, 14)),
              sentinel::planning::ReservationResult::unchanged);
    EXPECT_EQ(table.conflicts(), 1U);
    EXPECT_EQ(table.reserve(reservation("bridge", "beta", 13, 14, 2)),
              sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(table.reserve(reservation("tunnel", "gamma", 12, 14)),
              sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(table.committed().size(), 3U);
    EXPECT_THROW(table.reserve(reservation("bridge", "delta", 20, 19)), std::invalid_argument);
}

TEST(ReservationTable, ArbitrationIsStableAcrossProposalOrder) {
    const auto alpha = reservation("bridge", "alpha", 10, 12);
    const auto beta = reservation("bridge", "beta", 10, 12);
    sentinel::planning::ReservationTable first;
    EXPECT_EQ(first.reserve(beta), sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(first.reserve(alpha), sentinel::planning::ReservationResult::committed);
    sentinel::planning::ReservationTable second;
    EXPECT_EQ(second.reserve(alpha), sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(second.reserve(beta), sentinel::planning::ReservationResult::rejected);
    ASSERT_EQ(first.committed().size(), 1U);
    ASSERT_EQ(second.committed().size(), 1U);
    EXPECT_EQ(first.committed().front().agent_id, "alpha");
    EXPECT_EQ(second.committed().front().agent_id, "alpha");
    EXPECT_EQ(first.conflicts(), 1U);
    EXPECT_EQ(second.conflicts(), 1U);

    first.release_before(12);
    EXPECT_EQ(first.committed().size(), 1U);
    first.release_before(13);
    EXPECT_TRUE(first.committed().empty());
}

TEST(ReservationTable, RejectsStalePerAgentVersions) {
    sentinel::planning::ReservationTable table;
    const auto newest = reservation("bridge", "alpha", 10, 12, 3, 4, 2);
    EXPECT_EQ(table.reserve(newest), sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(table.reserve(newest), sentinel::planning::ReservationResult::unchanged);
    EXPECT_EQ(table.reserve(reservation("bridge", "alpha", 13, 15, 2, 4, 2)),
              sentinel::planning::ReservationResult::stale);
    EXPECT_EQ(table.reserve(reservation("bridge", "alpha", 13, 15, 3, 4, 2)),
              sentinel::planning::ReservationResult::stale);
    ASSERT_EQ(table.seen().size(), 1U);
    EXPECT_EQ(table.seen().front().version, 3U);
}

TEST(ReservationTable, ReleasesAgentsRoutesAndExpiredIntervals) {
    sentinel::planning::ReservationTable table;
    EXPECT_EQ(table.reserve(reservation("bridge", "alpha", 10, 12, 1, 4)),
              sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(table.reserve(reservation("tunnel", "alpha", 20, 22, 2, 5)),
              sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(table.reserve(reservation("bridge", "beta", 13, 15)),
              sentinel::planning::ReservationResult::committed);
    EXPECT_EQ(table.reserve(reservation("ridge", "gamma", 30, 32, 1, 1, 2)),
              sentinel::planning::ReservationResult::committed);

    table.release_route("alpha", 4);
    ASSERT_EQ(table.committed().size(), 3U);
    EXPECT_EQ(table.committed().front().agent_id, "beta");
    table.release_agent("beta");
    ASSERT_EQ(table.committed().size(), 2U);
    table.release_map(1);
    ASSERT_EQ(table.committed().size(), 1U);
    EXPECT_EQ(table.committed().front().route_version, 5U);
    table.release_before(23);
    EXPECT_TRUE(table.committed().empty());
    EXPECT_EQ(table.seen().size(), 4U);
}

TEST(ReservationTable, ExtractsCanonicalContestedResource) {
    sentinel::v1::World world;
    auto add = [&world](const char* id, sentinel::v1::RegionKind kind, std::int64_t minimum,
                        std::int64_t maximum) {
        auto* region = world.add_regions();
        region->set_id(id);
        region->set_kind(kind);
        region->mutable_minimum()->set_x_mm(minimum);
        region->mutable_minimum()->set_y_mm(0);
        region->mutable_maximum()->set_x_mm(maximum);
        region->mutable_maximum()->set_y_mm(1000);
    };
    add("zulu", sentinel::v1::REGION_KIND_CHOKEPOINT, 2000, 3000);
    add("terrain", sentinel::v1::REGION_KIND_TERRAIN, 500, 1500);
    add("alpha", sentinel::v1::REGION_KIND_CHOKEPOINT, 1000, 2000);
    sentinel::planning::Coordinate from;
    sentinel::planning::Coordinate to;
    from << 0, 500;
    to << 2500, 500;
    EXPECT_EQ(sentinel::planning::contested_resource(world, from, to), "alpha");
    from << 0, 2000;
    to << 3000, 2000;
    EXPECT_FALSE(sentinel::planning::contested_resource(world, from, to));
}
