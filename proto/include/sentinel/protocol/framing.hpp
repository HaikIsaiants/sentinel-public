#pragma once

#include <google/protobuf/message_lite.h>

#include <iosfwd>
#include <string>

namespace sentinel::protocol {

void configure_binary_stdio();
std::string deterministic_bytes(const google::protobuf::MessageLite& message);
bool read_frame(std::istream& stream, google::protobuf::MessageLite& message);
void write_frame(std::ostream& stream, const google::protobuf::MessageLite& message, bool flush = true);

}
