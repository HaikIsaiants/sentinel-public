#include <sentinel/protocol/framing.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using namespace std::chrono_literals;
using DeleteEntity = ros_gz_interfaces::srv::DeleteEntity;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

class Adapter : public rclcpp::Node {
public:
    Adapter()
        : Node("sentinel_gazebo_adapter"),
          trace_(declare_parameter<std::string>("trace"), std::ios::binary),
          playback_rate_(declare_parameter<double>("playback_rate", 1.0)),
          world_(declare_parameter<std::string>("world", "sentinel")),
          pose_client_(create_client<SetEntityPose>("/world/" + world_ + "/set_pose")),
          delete_client_(create_client<DeleteEntity>("/world/" + world_ + "/remove")) {
        if (!trace_) {
            throw std::runtime_error("failed to open visualization trace");
        }
        if (!(playback_rate_ > 0.0)) {
            throw std::invalid_argument("playback rate must be positive");
        }
    }

    void play() {
        if (!pose_client_->wait_for_service(20s) || !delete_client_->wait_for_service(20s)) {
            throw std::runtime_error("Gazebo entity services are unavailable");
        }
        bool terminal{};
        std::optional<std::uint64_t> previous_tick;
        sentinel::v1::SimulationFrame frame;
        while (sentinel::protocol::read_frame(trace_, frame)) {
            if (terminal || !frame.has_observations()) {
                throw std::runtime_error("invalid visualization trace");
            }
            const auto& batch = frame.observations();
            if (batch.observations_size() == 0
                || (previous_tick && batch.tick() != *previous_tick + 1)) {
                throw std::runtime_error("invalid visualization tick sequence");
            }
            apply(batch);
            previous_tick = batch.tick();
            terminal = batch.finished();
            if (!terminal) {
                const auto step_ms = batch.observations(0).observation().step_ms();
                if (step_ms == 0) {
                    throw std::runtime_error("invalid visualization step");
                }
                rclcpp::WallRate(1000.0 * playback_rate_ / static_cast<double>(step_ms)).sleep();
            }
            frame.Clear();
        }
        if (!terminal) {
            throw std::runtime_error("visualization trace has no terminal frame");
        }
    }

private:
    void await(rclcpp::Client<SetEntityPose>::SharedFuture future) {
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, 10s)
                != rclcpp::FutureReturnCode::SUCCESS
            || !future.get()->success) {
            throw std::runtime_error("Gazebo rejected an entity pose");
        }
    }

    void await(rclcpp::Client<DeleteEntity>::SharedFuture future) {
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, 10s)
                != rclcpp::FutureReturnCode::SUCCESS
            || !future.get()->success) {
            throw std::runtime_error("Gazebo rejected an entity removal");
        }
    }

    void set_pose(const std::string& id, double x, double y, double z, double yaw = 0.0) {
        // TODO: batch these updates; service round trips get choppy on long traces.
        auto request = std::make_shared<SetEntityPose::Request>();
        request->entity.name = id;
        request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
        request->pose.position.x = x;
        request->pose.position.y = y;
        request->pose.position.z = z;
        request->pose.orientation.z = std::sin(yaw / 2.0);
        request->pose.orientation.w = std::cos(yaw / 2.0);
        await(pose_client_->async_send_request(request).future.share());
    }

    void remove(const std::string& id) {
        if (!removed_.insert(id).second) {
            return;
        }
        auto request = std::make_shared<DeleteEntity::Request>();
        request->entity.name = id;
        request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
        await(delete_client_->async_send_request(request).future.share());
    }

    void apply_vehicle(const sentinel::v1::VehicleState& vehicle) {
        if (!vehicle.active()) {
            remove(vehicle.id());
            return;
        }
        auto& yaw = yaw_[vehicle.id()];
        if (vehicle.velocity_x_mm_s() != 0 || vehicle.velocity_y_mm_s() != 0) {
            yaw = std::atan2(static_cast<double>(vehicle.velocity_y_mm_s()),
                             static_cast<double>(vehicle.velocity_x_mm_s()));
        }
        const auto z = vehicle.kind() == "uav" ? 0.8 : 0.2;
        set_pose(vehicle.id(), static_cast<double>(vehicle.position().x_mm()) / 1000.0,
                 static_cast<double>(vehicle.position().y_mm()) / 1000.0, z, yaw);
    }

    void apply_tasks(const sentinel::v1::AgentObservation& observation) {
        std::set<std::string> active;
        for (const auto& task : observation.known_tasks()) {
            active.insert(task.id());
            task_positions_[task.id()] = task.target();
            if (!visible_tasks_.contains(task.id())) {
                const auto z = task.kind() == "inspection" ? 0.24 : 0.12;
                set_pose(task.id(), static_cast<double>(task.target().x_mm()) / 1000.0,
                         static_cast<double>(task.target().y_mm()) / 1000.0, z);
            }
            auto owner = owners_.find(task.id());
            if (owner != owners_.end() && owner->second != task.assigned_agent_id()) {
                RCLCPP_INFO(get_logger(), "%s reassigned from %s to %s", task.id().c_str(),
                            owner->second.c_str(), task.assigned_agent_id().c_str());
            }
            owners_[task.id()] = task.assigned_agent_id();
        }
        for (const auto& id : visible_tasks_) {
            if (active.contains(id)) {
                continue;
            }
            const auto& position = task_positions_.at(id);
            set_pose(id, static_cast<double>(position.x_mm()) / 1000.0,
                     static_cast<double>(position.y_mm()) / 1000.0, -1.0);
        }
        visible_tasks_ = std::move(active);
    }

    void apply_regions(const sentinel::v1::AgentObservation& observation) {
        for (const auto& region : observation.world().regions()) {
            if (region.kind() != sentinel::v1::REGION_KIND_CHOKEPOINT) {
                continue;
            }
            auto state = region_states_.find(region.id());
            if (state != region_states_.end() && state->second == region.closed()) {
                continue;
            }
            const auto x = static_cast<double>(region.minimum().x_mm() + region.maximum().x_mm()) / 2000.0;
            const auto y = static_cast<double>(region.minimum().y_mm() + region.maximum().y_mm()) / 2000.0;
            set_pose(region.id() + "-block", x, y, region.closed() ? 0.3 : -1.0);
            region_states_[region.id()] = region.closed();
        }
    }

    void apply_partition(const sentinel::v1::ObservationBatch& batch) {
        // TODO(partition UI): stop assuming uav-alpha is present
        for (const auto& envelope : batch.observations()) {
            const auto& observation = envelope.observation();
            if (observation.self().id() != "uav-alpha") {
                continue;
            }
            const auto partitioned = observation.reachable_peer_ids_size() < observation.peer_ids_size();
            if (partitioned_ && *partitioned_ == partitioned) {
                return;
            }
            const auto x = static_cast<double>(observation.self().position().x_mm()) / 1000.0;
            const auto y = static_cast<double>(observation.self().position().y_mm()) / 1000.0;
            set_pose("partition-indicator", x, y, partitioned ? 1.5 : -1.0);
            partitioned_ = partitioned;
            return;
        }
    }

    void apply(const sentinel::v1::ObservationBatch& batch) {
        for (const auto& envelope : batch.observations()) {
            if (!envelope.has_observation()
                || envelope.observation().self().id() != envelope.recipient_id()) {
                throw std::runtime_error("invalid visualization observation");
            }
            apply_vehicle(envelope.observation().self());
        }
        const auto& observation = batch.observations(0).observation();
        apply_tasks(observation);
        apply_regions(observation);
        apply_partition(batch);
        if (batch.finished()) {
            RCLCPP_INFO(get_logger(), "terminal hash %s", batch.summary().terminal_hash().c_str());
        }
    }

    std::ifstream trace_;
    double playback_rate_;
    std::string world_;
    rclcpp::Client<SetEntityPose>::SharedPtr pose_client_;
    rclcpp::Client<DeleteEntity>::SharedPtr delete_client_;
    std::map<std::string, double> yaw_;
    std::map<std::string, sentinel::v1::Point> task_positions_;
    std::map<std::string, std::string> owners_;
    std::map<std::string, bool> region_states_;
    std::set<std::string> visible_tasks_;
    std::set<std::string> removed_;
    std::optional<bool> partitioned_;
};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto adapter = std::make_shared<Adapter>();
        adapter->play();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
