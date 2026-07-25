#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace sentinel::core {

class HashBuilder {
public:
    void bytes(const void* data, std::size_t size);
    void unsigned_integer(std::uint64_t value);
    void signed_integer(std::int64_t value);
    void boolean(bool value);
    void text(std::string_view value);
    std::string finish() const;

private:
    std::uint64_t state_{14695981039346656037ULL};
};

std::string hash_bytes(std::string_view value);

}
