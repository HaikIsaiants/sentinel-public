#include <sentinel/core/rng.hpp>

#include <gtest/gtest.h>

TEST(RngStreams, RepeatsAndIgnoresAccessOrder) {
    sentinel::core::RngStreams first(42);
    sentinel::core::RngStreams second(42);
    const auto first_network = first.stream("network").next();
    const auto first_environment = first.stream("environment").next();
    const auto second_environment = second.stream("environment").next();
    const auto second_network = second.stream("network").next();
    EXPECT_EQ(first_network, second_network);
    EXPECT_EQ(first_environment, second_environment);
    EXPECT_NE(first_network, first_environment);
    EXPECT_EQ(first.states(), second.states());
}

TEST(RngStreams, ProducesBoundedValues) {
    sentinel::core::DeterministicRng rng(9);
    for (int i = 0; i < 1000; ++i) {
        const auto value = rng.uniform(-7, 13);
        EXPECT_GE(value, -7);
        EXPECT_LE(value, 13);
    }
    EXPECT_THROW(rng.uniform(2, 1), std::invalid_argument);
}
