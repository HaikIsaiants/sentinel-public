#pragma once

#include <sentinel/v1/sentinel.pb.h>

#include <memory>
#include <string>

namespace sentinel::agent {

class Controller {
public:
    explicit Controller(std::string agent_id);
    ~Controller();
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    sentinel::v1::Envelope act(const sentinel::v1::Envelope& input);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}
