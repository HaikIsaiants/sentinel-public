#include <sentinel/protocol/arguments.hpp>
#include <sentinel/protocol/framing.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#ifdef __linux__
#include <sys/prctl.h>
#endif

namespace {

constexpr std::uint32_t max_frame_bytes = 16U * 1024U * 1024U;
volatile std::sig_atomic_t stop_requested{};

void request_stop(int) {
    stop_requested = 1;
}

void check_stop() {
    if (stop_requested != 0) {
        throw std::runtime_error("supervisor terminated");
    }
}

int numeric_option(int argc, char** argv, std::string_view name, int fallback) {
    for (int idx = 1; idx + 1 < argc; ++idx) {
        if (argv[idx] == name) {
            const auto value = std::stoi(argv[idx + 1]);
            if (value < 1) {
                throw std::invalid_argument("invalid timeout");
            }
            return value;
        }
    }
    return fallback;
}

void close_fd(int& descriptor) {
    if (descriptor >= 0) {
        ::close(descriptor);
        descriptor = -1;
    }
}

std::array<int, 2> make_pipe() {
    std::array<int, 2> descriptors{};
    if (::pipe(descriptors.data()) != 0) {
        throw std::runtime_error(std::strerror(errno));
    }
    for (const int descriptor : descriptors) {
        if (::fcntl(descriptor, F_SETFD, FD_CLOEXEC) < 0) {
            close_fd(descriptors[0]);
            close_fd(descriptors[1]);
            throw std::runtime_error(std::strerror(errno));
        }
    }
    return descriptors;
}

class Process {
public:
    explicit Process(std::vector<std::string> arguments) {
        auto input = make_pipe();
        std::array<int, 2> output{};
        try {
            output = make_pipe();
        } catch (...) {
            close_fd(input[0]);
            close_fd(input[1]);
            throw;
        }
        std::vector<char*> values;
        values.reserve(arguments.size() + 1);
        for (auto& argument : arguments) {
            values.push_back(argument.data());
        }
        values.push_back(nullptr);
        const auto parent = ::getpid();
        pid_ = ::fork();
        if (pid_ == 0) {
#ifdef __linux__
            // Linux kills the worker if its supervisor disappears.
            if (::prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || ::getppid() != parent) {
                _exit(127);
            }
#endif
            ::dup2(input[0], STDIN_FILENO);
            ::dup2(output[1], STDOUT_FILENO);
            close_fd(input[0]);
            close_fd(input[1]);
            close_fd(output[0]);
            close_fd(output[1]);
            ::execvp(values[0], values.data());
            std::cerr << std::strerror(errno) << '\n';
            _exit(127);
        }
        close_fd(input[0]);
        close_fd(output[1]);
        if (pid_ < 0) {
            close_fd(input[1]);
            close_fd(output[0]);
            throw std::runtime_error(std::strerror(errno));
        }
        input_ = input[1];
        output_ = output[0];
        if (::fcntl(input_, F_SETFL, ::fcntl(input_, F_GETFL) | O_NONBLOCK) < 0
            || ::fcntl(output_, F_SETFL, ::fcntl(output_, F_GETFL) | O_NONBLOCK) < 0) {
            finish(true, 100);
            throw std::runtime_error(std::strerror(errno));
        }
    }

    Process(const Process&) = delete;
    Process& operator=(const Process&) = delete;

    Process(Process&& other) noexcept
        : pid_(other.pid_), input_(other.input_), output_(other.output_) {
        other.pid_ = -1;
        other.input_ = -1;
        other.output_ = -1;
    }

    Process& operator=(Process&&) = delete;

    ~Process() {
        try {
            finish(true, 100);
        } catch (...) {
        }
    }

    int input() const {
        return input_;
    }

    int output() const {
        return output_;
    }

    void terminate() {
        close_fd(input_);
        if (pid_ > 0) {
            ::kill(pid_, SIGTERM);
        }
    }

    int finish(bool force, int timeout_ms) {
        close_fd(input_);
        if (pid_ <= 0) {
            close_fd(output_);
            return 0;
        }
        if (force) {
            ::kill(pid_, SIGTERM);
        }
        auto status = wait(timeout_ms);
        if (!status && !force) {
            ::kill(pid_, SIGTERM);
            status = wait(timeout_ms);
        }
        if (!status) {
            ::kill(pid_, SIGKILL);
            int value{};
            while (::waitpid(pid_, &value, 0) < 0 && errno == EINTR) {
            }
            status = value;
        }
        pid_ = -1;
        close_fd(output_);
        if (WIFEXITED(*status)) {
            return WEXITSTATUS(*status);
        }
        return WIFSIGNALED(*status) ? 128 + WTERMSIG(*status) : 1;
    }

private:
    std::optional<int> wait(int timeout_ms) {
        int waited = 0;
        while (true) {
            int status{};
            const auto result = ::waitpid(pid_, &status, WNOHANG);
            if (result == pid_) {
                return status;
            }
            if (result < 0 && errno != EINTR) {
                throw std::runtime_error(std::strerror(errno));
            }
            if (waited >= timeout_ms) {
                return std::nullopt;
            }
            const int interval = std::min(10, timeout_ms - waited);
            ::poll(nullptr, 0, interval);
            waited += interval;
        }
    }

    pid_t pid_{-1};
    int input_{-1};
    int output_{-1};
};

void await(int descriptor, short events, int timeout_ms, std::string_view error) {
    pollfd current{descriptor, events, 0};
    while (true) {
        check_stop();
        const int result = ::poll(&current, 1, timeout_ms);
        if (result > 0 && (current.revents & events) != 0) {
            return;
        }
        if (result < 0 && errno == EINTR) {
            continue;
        }
        throw std::runtime_error(std::string(error));
    }
}

void read_exact(int descriptor, char* output, std::size_t size, int timeout_ms) {
    std::size_t offset = 0;
    while (offset < size) {
        await(descriptor, POLLIN, timeout_ms, "process response timed out");
        const auto count = ::read(descriptor, output + offset, size - offset);
        if (count > 0) {
            offset += static_cast<std::size_t>(count);
        } else if (count == 0) {
            throw std::runtime_error("process closed its protocol stream");
        } else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
            throw std::runtime_error(std::strerror(errno));
        }
    }
}

void write_exact(int descriptor, const char* input, std::size_t size, int timeout_ms) {
    std::size_t offset = 0;
    while (offset < size) {
        await(descriptor, POLLOUT, timeout_ms, "process request timed out");
        const auto count = ::write(descriptor, input + offset, size - offset);
        if (count > 0) {
            offset += static_cast<std::size_t>(count);
        } else if (count == 0) {
            throw std::runtime_error("process closed its protocol stream");
        } else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
            throw std::runtime_error(std::strerror(errno));
        }
    }
}

template <typename Message>
Message read_frame(int descriptor, int timeout_ms) {
    std::array<unsigned char, 4> header{};
    read_exact(descriptor, reinterpret_cast<char*>(header.data()), header.size(), timeout_ms);
    const auto size = (static_cast<std::uint32_t>(header[0]) << 24U)
                      | (static_cast<std::uint32_t>(header[1]) << 16U)
                      | (static_cast<std::uint32_t>(header[2]) << 8U)
                      | static_cast<std::uint32_t>(header[3]);
    if (size > max_frame_bytes) {
        throw std::runtime_error("process returned an oversized protocol frame");
    }
    std::string payload(size, '\0');
    read_exact(descriptor, payload.data(), payload.size(), timeout_ms);
    Message message;
    if (!message.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
        throw std::runtime_error("process returned an invalid protocol frame");
    }
    return message;
}

void write_frame(int descriptor, const google::protobuf::MessageLite& message, int timeout_ms) {
    const auto payload = sentinel::protocol::deterministic_bytes(message);
    if (payload.size() > max_frame_bytes) {
        throw std::runtime_error("protocol frame is too large");
    }
    const auto size = static_cast<std::uint32_t>(payload.size());
    const std::array<unsigned char, 4> header{
        static_cast<unsigned char>(size >> 24U),
        static_cast<unsigned char>(size >> 16U),
        static_cast<unsigned char>(size >> 8U),
        static_cast<unsigned char>(size)
    };
    write_exact(descriptor, reinterpret_cast<const char*>(header.data()), header.size(), timeout_ms);
    write_exact(descriptor, payload.data(), payload.size(), timeout_ms);
}

void validate_observation(const sentinel::v1::Envelope& envelope, std::uint64_t tick) {
    const auto& id = envelope.recipient_id();
    if (id.empty() || envelope.schema_version() != 1 || envelope.sender_id() != "sim"
        || !envelope.has_observation() || envelope.observation().self().id() != id
        || envelope.observation().tick() != tick
        || std::any_of(envelope.observation().assigned_tasks().begin(),
                       envelope.observation().assigned_tasks().end(),
                       [&id](const auto& task) { return task.assigned_agent_id() != id; })) {
        throw std::runtime_error("observation crossed an agent boundary");
    }
}

void validate_action(const sentinel::v1::Envelope& envelope, const sentinel::v1::Envelope& observation) {
    if (envelope.schema_version() != 1 || !envelope.has_action()
        || envelope.sender_id() != observation.recipient_id() || envelope.recipient_id() != "sim"
        || envelope.sequence() != observation.sequence()
        || envelope.simulation_time_ms() != observation.simulation_time_ms()
        || envelope.action().tick() != observation.observation().tick()) {
        throw std::runtime_error("agent returned an invalid action envelope");
    }
}

sentinel::v1::SimulationFrame run(int argc, char** argv) {
    const auto simulator_path = sentinel::protocol::required_option(argc, argv, "--sim");
    const auto agent_path = sentinel::protocol::required_option(argc, argv, "--agent");
    const auto scenario_path = sentinel::protocol::required_option(argc, argv, "--scenario");
    const auto log_path = sentinel::protocol::required_option(argc, argv, "--log");
    const auto summary_path = sentinel::protocol::required_option(argc, argv, "--summary");
    const int timeout_ms = numeric_option(argc, argv, "--timeout-ms", 60000);
    const int shutdown_timeout_ms = numeric_option(argc, argv, "--shutdown-timeout-ms", 5000);
    Process simulation({simulator_path, "run", "--scenario", scenario_path, "--log", log_path,
                        "--summary", summary_path});
    std::map<std::string, Process> agents;
    sentinel::v1::SimulationFrame terminal;
    while (true) {
        check_stop();
        auto frame = read_frame<sentinel::v1::SimulationFrame>(simulation.output(), timeout_ms);
        if (!frame.has_observations()) {
            throw std::runtime_error("simulator returned an invalid frame");
        }
        const auto& observations = frame.observations();
        if (observations.finished()) {
            if (!observations.has_summary()) {
                throw std::runtime_error("simulator returned an invalid terminal frame");
            }
            terminal = std::move(frame);
            break;
        }
        if (observations.observations_size() < 3 || observations.observations_size() > 5) {
            throw std::runtime_error("mission does not contain three to five agents");
        }
        std::vector<const sentinel::v1::Envelope*> ordered;
        ordered.reserve(static_cast<std::size_t>(observations.observations_size()));
        for (const auto& envelope : observations.observations()) {
            validate_observation(envelope, observations.tick());
            ordered.push_back(&envelope);
        }
        // agent id order keeps the lockstep exchange reproducible
        std::sort(ordered.begin(), ordered.end(), [](const auto* left, const auto* right) {
            return left->recipient_id() < right->recipient_id();
        });
        if (std::adjacent_find(ordered.begin(), ordered.end(), [](const auto* left, const auto* right) {
                return left->recipient_id() == right->recipient_id();
            }) != ordered.end()) {
            throw std::runtime_error("duplicate agent observation");
        }
        const bool initialize = agents.empty();
        for (const auto* envelope : ordered) {
            const auto& id = envelope->recipient_id();
            if (initialize) {
                agents.emplace(id, Process({agent_path, "--id", id}));
            } else if (!agents.contains(id)) {
                throw std::runtime_error("observed vehicle set changed");
            }
        }
        if (agents.size() != ordered.size()) {
            throw std::runtime_error("observed vehicle set changed");
        }
        for (const auto* envelope : ordered) {
            write_frame(agents.at(envelope->recipient_id()).input(), *envelope, timeout_ms);
        }
        sentinel::v1::SimulationFrame request;
        auto* actions = request.mutable_actions();
        actions->set_tick(observations.tick());
        for (const auto* observation : ordered) {
            auto action = read_frame<sentinel::v1::Envelope>(agents.at(observation->recipient_id()).output(), timeout_ms);
            validate_action(action, *observation);
            actions->add_actions()->CopyFrom(action);
        }
        write_frame(simulation.input(), request, timeout_ms);
    }
    for (auto& [id, agent] : agents) {
        if (agent.finish(false, shutdown_timeout_ms) != 0) {
            throw std::runtime_error("agent process failed: " + id);
        }
    }
    if (simulation.finish(false, shutdown_timeout_ms) != 0) {
        throw std::runtime_error("simulator process failed");
    }
    check_stop();
    return terminal;
}

}

int main(int argc, char** argv) {
    try {
        std::signal(SIGPIPE, SIG_IGN);
        std::signal(SIGINT, request_stop);
        std::signal(SIGTERM, request_stop);
#ifdef __linux__
        const auto parent = ::getppid();
        if (parent <= 1 || ::prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || ::getppid() != parent) {
            throw std::runtime_error("failed to bind supervisor lifetime");
        }
#endif
        sentinel::protocol::configure_binary_stdio();
        sentinel::protocol::write_frame(std::cout, run(argc, argv));
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
