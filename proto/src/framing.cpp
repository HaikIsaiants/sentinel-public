#include <sentinel/protocol/framing.hpp>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace sentinel::protocol {

void configure_binary_stdio() {
#ifdef _WIN32
    if (_setmode(_fileno(stdin), _O_BINARY) == -1 || _setmode(_fileno(stdout), _O_BINARY) == -1) {
        throw std::runtime_error("failed to configure binary standard streams");
    }
#endif
}

std::string deterministic_bytes(const google::protobuf::MessageLite& message) {
    std::string output;
    google::protobuf::io::StringOutputStream raw(&output);
    google::protobuf::io::CodedOutputStream coded(&raw);
    coded.SetSerializationDeterministic(true);
    if (!message.SerializeToCodedStream(&coded)) {
        throw std::runtime_error("failed to serialize protobuf message");
    }
    coded.Trim();
    return output;
}

bool read_frame(std::istream& stream, google::protobuf::MessageLite& message) {
    std::array<unsigned char, 4> header{};
    stream.read(reinterpret_cast<char*>(header.data()), header.size());
    if (stream.gcount() == 0 && stream.eof()) {
        return false;
    }
    if (stream.gcount() != static_cast<std::streamsize>(header.size())) {
        throw std::runtime_error("truncated frame header");
    }
    // 32-bit big-endian size prefix.
    const std::uint32_t size = (static_cast<std::uint32_t>(header[0]) << 24U)
                               | (static_cast<std::uint32_t>(header[1]) << 16U)
                               | (static_cast<std::uint32_t>(header[2]) << 8U)
                               | static_cast<std::uint32_t>(header[3]);
    if (size > 16U * 1024U * 1024U) {
        throw std::runtime_error("protobuf frame is too large");
    }
    std::string payload(size, '\0');
    stream.read(payload.data(), static_cast<std::streamsize>(payload.size()));
    if (stream.gcount() != static_cast<std::streamsize>(payload.size())) {
        throw std::runtime_error("truncated protobuf frame");
    }
    if (!message.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
        throw std::runtime_error("invalid protobuf frame");
    }
    return true;
}

void write_frame(std::ostream& stream, const google::protobuf::MessageLite& message, bool flush) {
    const std::string payload = deterministic_bytes(message);
    if (payload.size() > 16U * 1024U * 1024U) {
        throw std::runtime_error("protobuf frame is too large");
    }
    const auto size = static_cast<std::uint32_t>(payload.size());
    const std::array<unsigned char, 4> header{
        static_cast<unsigned char>((size >> 24U) & 0xffU),
        static_cast<unsigned char>((size >> 16U) & 0xffU),
        static_cast<unsigned char>((size >> 8U) & 0xffU),
        static_cast<unsigned char>(size & 0xffU)
    };
    stream.write(reinterpret_cast<const char*>(header.data()), header.size());
    stream.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    if (flush) {
        stream.flush();
    }
    if (!stream) {
        throw std::runtime_error("failed to write protobuf frame");
    }
}

}
