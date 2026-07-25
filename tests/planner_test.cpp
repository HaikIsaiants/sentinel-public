#include <sentinel/planning/planner.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace {

sentinel::planning::Coordinate coordinate(std::int64_t x, std::int64_t y) {
    sentinel::planning::Coordinate result;
    result << x, y;
    return result;
}

sentinel::v1::World world(std::int64_t width = 4000, std::int64_t height = 4000) {
    sentinel::v1::World result;
    result.set_width_mm(width);
    result.set_height_mm(height);
    result.set_grid_cell_mm(1000);
    result.set_map_version(1);
    return result;
}

sentinel::v1::Region* add_region(sentinel::v1::World& world, std::string id,
                                 sentinel::v1::RegionKind kind, std::int64_t minimum_x,
                                 std::int64_t minimum_y, std::int64_t maximum_x,
                                 std::int64_t maximum_y) {
    auto* region = world.add_regions();
    region->set_id(std::move(id));
    region->set_kind(kind);
    region->mutable_minimum()->set_x_mm(minimum_x);
    region->mutable_minimum()->set_y_mm(minimum_y);
    region->mutable_maximum()->set_x_mm(maximum_x);
    region->mutable_maximum()->set_y_mm(maximum_y);
    region->set_energy_multiplier_permille(1000);
    return region;
}

sentinel::v1::VehicleState vehicle(std::string id, std::int64_t x, std::int64_t y) {
    sentinel::v1::VehicleState result;
    result.set_id(std::move(id));
    result.set_active(true);
    result.mutable_position()->set_x_mm(x);
    result.mutable_position()->set_y_mm(y);
    result.set_max_speed_mm_s(1000);
    result.set_initial_energy_mj(100000);
    result.set_energy_mj(100000);
    result.set_energy_cost_mj_per_meter(1000);
    result.set_payload_grams(1000);
    return result;
}

sentinel::v1::TaskState task(std::int64_t x, std::int64_t y) {
    sentinel::v1::TaskState result;
    result.set_id("task");
    result.set_kind("search");
    result.mutable_target()->set_x_mm(x);
    result.mutable_target()->set_y_mm(y);
    result.set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    result.set_status(sentinel::v1::TASK_STATUS_PENDING);
    result.set_deadline_tick(100);
    result.set_priority(50);
    return result;
}

void expect_coordinate(const sentinel::planning::Coordinate& value, std::int64_t x, std::int64_t y) {
    EXPECT_EQ(value.x(), x);
    EXPECT_EQ(value.y(), y);
}

}

TEST(Planner, CanonicalPathIsStableAcrossRegionOrder) {
    auto first_world = world();
    add_region(first_world, "center", sentinel::v1::REGION_KIND_OBSTACLE, 2000, 2000, 2000, 2000);
    add_region(first_world, "corner", sentinel::v1::REGION_KIND_CHOKEPOINT, 4000, 4000, 4000, 4000);
    auto second_world = first_world;
    std::reverse(second_world.mutable_regions()->begin(), second_world.mutable_regions()->end());
    const auto current = vehicle("agent", 0, 2000);
    const auto start = coordinate(0, 2000);
    const auto goal = coordinate(4000, 2000);
    const auto first = sentinel::planning::astar(first_world, current, start, goal, 1000);
    const auto second = sentinel::planning::astar(second_world, current, start, goal, 1000);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    ASSERT_EQ(first->points.size(), 7U);
    ASSERT_EQ(second->points.size(), first->points.size());
    const std::vector<sentinel::planning::Coordinate> expected{
        coordinate(0, 2000), coordinate(0, 1000), coordinate(1000, 1000), coordinate(2000, 1000),
        coordinate(3000, 1000), coordinate(4000, 1000), coordinate(4000, 2000)};
    for (std::size_t i = 0; i < expected.size(); ++i) {
        expect_coordinate(first->points[i], expected[i].x(), expected[i].y());
        expect_coordinate(second->points[i], expected[i].x(), expected[i].y());
    }
    EXPECT_EQ(first->distance_mm, 6000);
    EXPECT_EQ(first->energy_mj, 6000);
    EXPECT_EQ(first->travel_ticks, 6U);
    EXPECT_EQ(second->distance_mm, first->distance_mm);
    EXPECT_EQ(second->energy_mj, first->energy_mj);
    EXPECT_EQ(second->travel_ticks, first->travel_ticks);
}

TEST(Planner, AppliesRegionAndTerrainRules) {
    const auto from = coordinate(0, 0);
    const auto to = coordinate(1000, 0);
    auto current = vehicle("agent", 0, 0);
    for (const auto kind : {sentinel::v1::REGION_KIND_OBSTACLE, sentinel::v1::REGION_KIND_RESTRICTED}) {
        auto blocked = world(1000, 1000);
        add_region(blocked, "blocked", kind, 1000, 0, 1000, 0);
        EXPECT_FALSE(sentinel::planning::segment_allowed(blocked, current, from, to));
    }
    auto chokepoint = world(1000, 1000);
    auto* passage = add_region(chokepoint, "passage", sentinel::v1::REGION_KIND_CHOKEPOINT, 1000, 0, 1000, 0);
    EXPECT_TRUE(sentinel::planning::segment_allowed(chokepoint, current, from, to));
    passage->set_closed(true);
    EXPECT_FALSE(sentinel::planning::segment_allowed(chokepoint, current, from, to));

    auto terrain_world = world(4000, 1000);
    auto* mud = add_region(terrain_world, "mud", sentinel::v1::REGION_KIND_TERRAIN, 2000, 0, 2000, 1000);
    mud->set_terrain("mud");
    mud->set_energy_multiplier_permille(1500);
    EXPECT_FALSE(sentinel::planning::astar(terrain_world, current, coordinate(0, 0), coordinate(4000, 0), 1000));
    current.add_terrain_access("mud");
    const auto route = sentinel::planning::astar(terrain_world, current, coordinate(0, 0), coordinate(4000, 0), 1000);
    ASSERT_TRUE(route.has_value());
    EXPECT_EQ(route->distance_mm, 4000);
    EXPECT_EQ(route->energy_mj, 5000);
    EXPECT_EQ(sentinel::planning::terrain_multiplier(terrain_world, coordinate(1000, 0), coordinate(2000, 0)),
              1500U);
    EXPECT_EQ(sentinel::planning::motion_energy(current, 1000, 1500), 1500);
}

TEST(Planner, DeadlineAndReserveEdges) {
    const auto current_world = world(2000, 1000);
    auto current = vehicle("agent", 0, 0);
    current.set_initial_energy_mj(10000);
    current.set_energy_mj(4000);
    current.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    auto current_task = task(2000, 0);
    current_task.set_service_ticks(2);
    current_task.set_service_energy_mj_per_tick(500);
    current_task.set_deadline_tick(9);
    EXPECT_EQ(sentinel::planning::reserve_energy(current), 1000);
    const auto exact = sentinel::planning::feasible_route(current_world, current, current_task, 5, 1000);
    ASSERT_TRUE(exact.has_value());
    EXPECT_EQ(exact->distance_mm, 2000);
    EXPECT_EQ(exact->energy_mj, 2000);
    EXPECT_EQ(exact->travel_ticks, 2U);

    current_task.set_deadline_tick(8);
    EXPECT_FALSE(sentinel::planning::feasible_route(current_world, current, current_task, 5, 1000));
    current_task.set_deadline_tick(9);
    current.set_energy_mj(3999);
    EXPECT_FALSE(sentinel::planning::feasible_route(current_world, current, current_task, 5, 1000));
    current.set_initial_energy_mj(10001);
    EXPECT_EQ(sentinel::planning::reserve_energy(current), 1001);
}

TEST(Planner, NearestCapableFiltersAndUsesAgentIdTieBreak) {
    const auto current_world = world(4000, 1000);
    auto current_task = task(4000, 0);
    current_task.set_payload_required_grams(100);
    std::vector<sentinel::v1::VehicleState> vehicles;

    auto incapable = vehicle("a-incapable", 3000, 0);
    vehicles.push_back(incapable);
    auto overloaded = vehicle("b-overloaded", 2000, 0);
    overloaded.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    overloaded.set_payload_grams(99);
    vehicles.push_back(overloaded);
    auto depleted = vehicle("c-depleted", 1000, 0);
    depleted.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    depleted.set_energy_mj(1);
    vehicles.push_back(depleted);
    auto later = vehicle("z-feasible", 0, 0);
    later.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    vehicles.push_back(later);
    auto winner = vehicle("m-feasible", 0, 0);
    winner.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    vehicles.push_back(winner);

    const auto first = sentinel::planning::nearest_capable(current_world, vehicles, current_task, 0, 1000, 3, 7);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->agent_id(), "m-feasible");
    EXPECT_EQ(first->distance_mm(), 4000);
    EXPECT_EQ(first->epoch(), 3U);
    EXPECT_EQ(first->version(), 7U);
    EXPECT_FALSE(sentinel::planning::make_claim(current_world, incapable, current_task, 0, 1000, 3, 7).feasible());

    std::reverse(vehicles.begin(), vehicles.end());
    const auto second = sentinel::planning::nearest_capable(current_world, vehicles, current_task, 0, 1000, 3, 7);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->agent_id(), first->agent_id());
    EXPECT_EQ(second->distance_mm(), first->distance_mm());
}

TEST(Planner, RoutesDeterministicallyFromAnOffGridPosition) {
    const auto current_world = world(4000, 2000);
    const auto current = vehicle("agent", 500, 1000);
    const auto route = sentinel::planning::route_from(current_world, current, coordinate(500, 1000),
                                                       coordinate(3000, 1000), 1000);
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->points.size(), 4U);
    expect_coordinate(route->points[0], 500, 1000);
    expect_coordinate(route->points[1], 1000, 1000);
    expect_coordinate(route->points[2], 2000, 1000);
    expect_coordinate(route->points[3], 3000, 1000);
    EXPECT_EQ(route->distance_mm, 2500);
    EXPECT_EQ(route->energy_mj, 2500);
    EXPECT_EQ(route->travel_ticks, 3U);
    EXPECT_FALSE(sentinel::planning::route_from(current_world, current, coordinate(500, 500),
                                                 coordinate(3000, 1000), 1000));
}

TEST(Planner, EvaluatesOrderedBundleConstraintsCumulatively) {
    const auto current_world = world(6000, 1000);
    auto current = vehicle("agent", 0, 0);
    current.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    current.add_capabilities(sentinel::v1::CAPABILITY_DELIVERY);
    auto first = task(2000, 0);
    first.set_id("first");
    first.set_service_ticks(1);
    first.set_service_energy_mj_per_tick(100);
    first.set_deadline_tick(4);
    auto second = task(4000, 0);
    second.set_id("second");
    second.set_required_capability(sentinel::v1::CAPABILITY_DELIVERY);
    second.set_payload_required_grams(600);
    second.set_service_ticks(2);
    second.set_service_energy_mj_per_tick(200);
    second.set_deadline_tick(8);
    const auto evaluated = sentinel::planning::evaluate_bundle(current_world, current, {first, second}, 0, 1000);
    ASSERT_TRUE(evaluated.has_value());
    EXPECT_EQ(evaluated->distance_mm, 4000);
    EXPECT_EQ(evaluated->energy_mj, 4500);
    EXPECT_EQ(evaluated->completion_tick, 7U);
    EXPECT_EQ(evaluated->task_completion_ticks, (std::vector<std::uint64_t>{3, 7}));

    auto third = second;
    third.set_id("third");
    EXPECT_FALSE(sentinel::planning::evaluate_bundle(current_world, current, {first, second, third}, 0, 1000));
    auto energy_limited = current;
    energy_limited.set_initial_energy_mj(10000);
    energy_limited.set_energy_mj(5450);
    EXPECT_TRUE(sentinel::planning::evaluate_bundle(current_world, energy_limited, {second}, 0, 1000));
    EXPECT_FALSE(sentinel::planning::evaluate_bundle(current_world, energy_limited, {first, second}, 0, 1000));
    auto incapable = current;
    incapable.clear_capabilities();
    incapable.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    EXPECT_FALSE(sentinel::planning::evaluate_bundle(current_world, incapable, {first, second}, 0, 1000));
    second.set_deadline_tick(6);
    EXPECT_FALSE(sentinel::planning::evaluate_bundle(current_world, current, {first, second}, 0, 1000));
}

TEST(Planner, FindsBestStableBundleInsertionAndScoresPriority) {
    const auto current_world = world(6000, 1000);
    auto current = vehicle("agent", 0, 0);
    current.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    auto existing = task(4000, 0);
    existing.set_id("existing");
    auto candidate = task(1000, 0);
    candidate.set_id("candidate");
    candidate.set_priority(70);
    const auto insertion = sentinel::planning::best_insertion(current_world, current, {existing}, candidate,
                                                               0, 1000);
    ASSERT_TRUE(insertion.has_value());
    EXPECT_EQ(insertion->index, 0U);
    EXPECT_EQ(insertion->distance_mm, 0);
    EXPECT_EQ(insertion->energy_mj, 0);
    EXPECT_EQ(insertion->completion_tick, 1U);
    const auto appended = sentinel::planning::best_insertion(current_world, current, {existing}, candidate,
                                                               0, 1000, 1);
    ASSERT_TRUE(appended.has_value());
    EXPECT_EQ(appended->index, 1U);
    EXPECT_FALSE(sentinel::planning::best_insertion(current_world, current, {existing}, candidate,
                                                     0, 1000, 2));

    auto lower_priority = candidate;
    lower_priority.set_priority(60);
    const auto lower = sentinel::planning::best_insertion(current_world, current, {existing}, lower_priority,
                                                           0, 1000);
    ASSERT_TRUE(lower.has_value());
    EXPECT_EQ(insertion->score - lower->score, 10'000'000'000'000LL);

    auto tied = task(4000, 0);
    tied.set_id("tied");
    const auto stable = sentinel::planning::best_insertion(current_world, current, {existing}, tied, 0, 1000);
    ASSERT_TRUE(stable.has_value());
    EXPECT_EQ(stable->index, 0U);
    EXPECT_FALSE(sentinel::planning::best_insertion(current_world, current, {existing}, existing, 0, 1000));
}
