#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace sentinel::protocol {

inline std::string required_option(int argc, char* const argv[], std::string_view name, int first = 1) {
    for (int i = first; i + 1 < argc; ++i) {
        if (argv[i] == name) {
            return argv[i + 1];
        }
    }
    throw std::invalid_argument("missing required option");
}

}
