#include <sentinel/agent/autonomy.hpp>
#include <sentinel/protocol/arguments.hpp>
#include <sentinel/protocol/framing.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    try {
        sentinel::agent::Controller controller(sentinel::protocol::required_option(argc, argv, "--id"));
        sentinel::protocol::configure_binary_stdio();
        sentinel::v1::Envelope input;
        while (sentinel::protocol::read_frame(std::cin, input)) {
            sentinel::protocol::write_frame(std::cout, controller.act(input));
            input.Clear();
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
