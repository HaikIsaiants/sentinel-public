#include <sentinel/protocol/framing.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <gtest/gtest.h>

#include <sstream>

TEST(Framing, RoundTripsDeterministicProtobuf) {
    sentinel::v1::Envelope expected;
    expected.set_schema_version(1);
    expected.set_sequence(4);
    expected.set_sender_id("agent-a");
    expected.set_recipient_id("sim");
    expected.mutable_action()->set_tick(3);
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    sentinel::protocol::write_frame(stream, expected);
    sentinel::v1::Envelope actual;
    EXPECT_TRUE(sentinel::protocol::read_frame(stream, actual));
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(expected), sentinel::protocol::deterministic_bytes(actual));
    EXPECT_FALSE(sentinel::protocol::read_frame(stream, actual));
}

TEST(Framing, RejectsTruncatedFrames) {
    std::stringstream stream(std::string("\0\0\0", 3), std::ios::in | std::ios::binary);
    sentinel::v1::Envelope message;
    EXPECT_THROW(sentinel::protocol::read_frame(stream, message), std::runtime_error);
}
