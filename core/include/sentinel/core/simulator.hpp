#pragma once

#include <sentinel/core/network.hpp>
#include <sentinel/core/rng.hpp>
#include <sentinel/planning/planner.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sentinel::core {

struct StepOutcome {
    sentinel::v1::ActionBatch actions;
    std::vector<sentinel::v1::AppliedEvent> applied_events;
    std::vector<sentinel::v1::NetworkOutcome> network_outcomes;
    sentinel::v1::ObservationBatch observations;
};

class Simulator {
public:
    explicit Simulator(sentinel::v1::Scenario scenario);
    sentinel::v1::ObservationBatch observe() const;
    StepOutcome step(const sentinel::v1::ActionBatch& actions);
    StepOutcome replay_step(const sentinel::v1::ActionBatch& actions,
                            const google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent>& applied_events);
    sentinel::v1::SimulationSummary summary() const;
    std::string state_hash() const;
    bool finished() const;
    std::uint64_t tick() const;
    const sentinel::v1::Scenario& scenario() const;

private:
    struct Vehicle {
        sentinel::v1::VehicleSpec spec;
        std::int64_t x_mm{};
        std::int64_t y_mm{};
        std::int64_t velocity_x_mm_s{};
        std::int64_t velocity_y_mm_s{};
        std::int64_t energy_mj{};
        std::int64_t energy_capacity_mj{};
        std::int64_t max_speed_mm_s{};
        std::int64_t payload_grams{};
        bool active{true};
        std::optional<std::uint64_t> failure_tick;
        bool failure_announced{};
        sentinel::v1::BehaviorMode behavior_mode{sentinel::v1::BEHAVIOR_MODE_IDLE};
        std::uint64_t route_version{};
    };

    struct FailureMonitor {
        std::string detector_id;
        std::string failed_id;
        std::uint64_t failure_tick{};
        std::uint64_t missed_ticks{};
        std::optional<std::uint64_t> detection_tick;
    };

    struct Task {
        sentinel::v1::TaskSpec spec;
        bool released{};
        sentinel::v1::TaskStatus status{sentinel::v1::TASK_STATUS_PENDING};
        std::string assigned_agent_id;
        std::uint64_t allocation_epoch{};
        std::uint64_t allocation_version{};
        std::uint64_t progress_ticks{};
        std::uint64_t completion_tick{};
        std::int64_t allocation_score{};
        std::uint32_t bundle_position{};
    };

    sentinel::v1::ActionBatch canonicalize_actions(const sentinel::v1::ActionBatch& actions) const;
    StepOutcome step_internal(const sentinel::v1::ActionBatch& actions,
                              const google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent>* recorded_events);
    std::string compute_state_hash() const;
    std::vector<sentinel::v1::AppliedEvent> apply_events(
        const google::protobuf::RepeatedPtrField<sentinel::v1::AppliedEvent>* recorded_events,
        std::vector<sentinel::v1::NetworkOutcome>& network_outcomes);
    void apply_actions(const sentinel::v1::ActionBatch& actions,
                       std::vector<sentinel::v1::NetworkOutcome>& network_outcomes,
                       bool coordination_fence);
    void apply_failure_detections(const sentinel::v1::ActionBatch& actions);
    void apply_reservations(const sentinel::v1::ActionBatch& actions, bool coordination_fence);
    void apply_commits(const sentinel::v1::ActionBatch& actions, bool coordination_fence);
    void apply_rejections(const sentinel::v1::ActionBatch& actions);
    void update_tasks(const sentinel::v1::ActionBatch& actions);
    void update_failure_monitors();
    bool valid_commit(const sentinel::v1::AllocationCommit& commit, const sentinel::v1::Envelope& envelope) const;
    sentinel::v1::NetworkProfile network_profile(std::string_view id) const;
    void begin_convergence();
    void update_convergence();
    bool allocation_agrees() const;
    std::string allocation_signature() const;
    sentinel::v1::VehicleState vehicle_state(const Vehicle& vehicle) const;
    sentinel::v1::TaskState task_state(const Task& task) const;
    Vehicle& vehicle(std::string_view id);
    const Vehicle& vehicle(std::string_view id) const;
    Task& task(std::string_view id);
    const Task& task(std::string_view id) const;
    const sentinel::v1::ServiceLocation& location(std::string_view id) const;
    bool has_capability(const Vehicle& vehicle, sentinel::v1::Capability capability) const;
    bool valid_detection(const sentinel::v1::FailureDetection& detection,
                         const sentinel::v1::Envelope& envelope) const;
    bool valid_reservation(const sentinel::v1::SpaceTimeReservation& reservation,
                           const sentinel::v1::Envelope& envelope) const;
    bool has_reservation(std::string_view agent_id, std::string_view resource_id,
                         std::uint64_t route_version) const;
    bool occupied(std::string_view resource_id, std::string_view agent_id) const;
    void record_energy_invariant(const Vehicle& vehicle);
    void record_reservation_invariant();
    void record_commit_invariants(const Task& task, const Vehicle& vehicle);

    sentinel::v1::Scenario scenario_;
    std::vector<Vehicle> vehicles_;
    std::vector<Task> tasks_;
    RngStreams rng_streams_;
    NetworkEmulator network_;
    planning::ReservationTable reservations_;
    std::string network_profile_;
    std::vector<sentinel::v1::NetworkMessage> delivered_messages_;
    std::vector<sentinel::v1::NetworkOutcome> network_outcomes_;
    std::vector<sentinel::v1::AllocationState> allocation_views_;
    std::vector<sentinel::v1::AllocationConvergence> allocation_convergence_;
    std::vector<FailureMonitor> failure_monitors_;
    std::vector<sentinel::v1::FailureDetection> failure_detections_;
    std::vector<sentinel::v1::TaskReassignment> task_reassignments_;
    std::vector<sentinel::v1::ReplanningSample> replanning_samples_;
    std::uint64_t allocation_epoch_{1};
    std::uint64_t convergence_start_tick_{};
    std::uint64_t convergence_candidate_tick_{};
    std::uint32_t convergence_stable_ticks_{};
    std::string convergence_signature_;
    bool convergence_open_{};
    std::uint64_t tick_{};
    std::size_t next_event_{};
    std::string state_hash_;
    std::vector<std::string> tick_hashes_;
    std::uint64_t wait_ticks_{};
    std::uint64_t replan_count_{};
    std::uint64_t recharge_ticks_{};
    std::uint64_t return_ticks_{};
    std::uint64_t travel_distance_mm_{};
    std::uint64_t energy_consumed_mj_{};
    std::uint64_t rejected_commits_{};
    std::uint64_t agent_energy_below_zero_violations_{};
    std::uint64_t committed_reservation_overlap_violations_{};
    std::uint64_t completed_task_reassignment_violations_{};
    std::uint64_t incapable_agent_commit_violations_{};
};

}
