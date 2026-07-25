#pragma once

#include <sentinel/v1/sentinel.pb.h>

#include <memory>
#include <string>
#include <vector>

namespace sentinel::agent {

struct AllocationResult {
    v1::AllocationState state;
    std::vector<v1::NetworkMessage> outgoing_messages;
    std::vector<v1::AllocationCommit> commits;
    bool pending{};
    bool coordinated{};
};

class Allocator {
public:
    explicit Allocator(std::string agent_id);
    ~Allocator();
    Allocator(const Allocator&) = delete;
    Allocator& operator=(const Allocator&) = delete;
    AllocationResult update(const v1::AgentObservation& observation);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}
