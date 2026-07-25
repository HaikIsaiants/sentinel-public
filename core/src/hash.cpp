#include <sentinel/core/hash.hpp>

#include <array>
#include <iomanip>
#include <sstream>

namespace sentinel::core {

void HashBuilder::bytes(const void* data, std::size_t size) {
    const auto* input = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        state_ ^= input[i];
        state_ *= 1099511628211ULL;
    }
}

void HashBuilder::unsigned_integer(std::uint64_t value) {
    std::array<unsigned char, 8> data{};
    for (std::size_t byte = 0; byte < data.size(); ++byte) {
        data[byte] = static_cast<unsigned char>((value >> (byte * 8U)) & 0xffU);
    }
    bytes(data.data(), data.size());
}

void HashBuilder::signed_integer(std::int64_t value) {
    unsigned_integer(static_cast<std::uint64_t>(value));
}

void HashBuilder::boolean(bool value) {
    const unsigned char data = value ? 1U : 0U;
    bytes(&data, 1);
}

void HashBuilder::text(std::string_view value) {
    unsigned_integer(value.size());
    bytes(value.data(), value.size());
}

std::string HashBuilder::finish() const {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(16) << state_;
    return stream.str();
}

std::string hash_bytes(std::string_view value) {
    HashBuilder hash;
    hash.bytes(value.data(), value.size());
    return hash.finish();
}

}
