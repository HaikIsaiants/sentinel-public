#include <sentinel/agent/allocation.hpp>

#include <sentinel/planning/planner.hpp>
#include <sentinel/protocol/framing.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace sentinel::agent {
namespace {

using BidMap = std::map<std::string, v1::AllocationBid>;
using TaskMap = std::map<std::string, v1::TaskState>;

constexpr std::uint64_t state_retransmit_ms = 500;

bool same_bid(const v1::AllocationBid& left, const v1::AllocationBid& right) {
    return std::tuple{left.epoch(), left.version(), left.task_id(), left.bidder_id(), left.score(),
                      left.distance_mm(), left.energy_mj(), left.completion_tick(), left.bundle_position()}
           == std::tuple{right.epoch(), right.version(), right.task_id(), right.bidder_id(), right.score(),
                         right.distance_mm(), right.energy_mj(), right.completion_tick(), right.bundle_position()};
}

bool same_relay(const v1::AllocationRelay& left, const v1::AllocationRelay& right) {
    return same_bid(left.bid(), right.bid())
           && std::equal(left.path_agent_ids().begin(), left.path_agent_ids().end(),
                         right.path_agent_ids().begin(), right.path_agent_ids().end());
}

bool same_bid_content(const v1::AllocationBid& left, const v1::AllocationBid& right) {
    return std::tuple{left.task_id(), left.bidder_id(), left.score(), left.distance_mm(), left.energy_mj(),
                      left.completion_tick(), left.bundle_position()}
           == std::tuple{right.task_id(), right.bidder_id(), right.score(), right.distance_mm(), right.energy_mj(),
                         right.completion_tick(), right.bundle_position()};
}

bool same_owner(const v1::AllocationCommit& left, const v1::AllocationCommit& right) {
    return std::tuple{left.epoch(), left.version(), left.task_id(), left.agent_id(), left.distance_mm(),
                      left.score(), left.bundle_position()}
           == std::tuple{right.epoch(), right.version(), right.task_id(), right.agent_id(), right.distance_mm(),
                         right.score(), right.bundle_position()};
}

bool better(const v1::AllocationBid& left, const v1::AllocationBid& right, v1::AllocationPolicy policy) {
    if (policy == v1::ALLOCATION_POLICY_SENTINEL_CBBA) {
        return left.score() != right.score() ? left.score() > right.score()
                                             : left.bidder_id() < right.bidder_id();
    }
    return std::tuple{left.distance_mm(), left.bidder_id()}
           < std::tuple{right.distance_mm(), right.bidder_id()};
}

bool same_state(const v1::AllocationState& left, const v1::AllocationState& right) {
    if (left.epoch() != right.epoch() || left.sender_id() != right.sender_id()
        || left.map_version() != right.map_version() || left.bids_size() != right.bids_size()
        || left.winners_size() != right.winners_size()
        || left.winner_relays_size() != right.winner_relays_size()
        || left.bundle_task_ids_size() != right.bundle_task_ids_size()
        || left.owners_size() != right.owners_size()) {
        return false;
    }
    for (int i = 0; i < left.bids_size(); ++i) {
        if (!same_bid(left.bids(i), right.bids(i))) {
            return false;
        }
    }
    for (int i = 0; i < left.winners_size(); ++i) {
        if (!same_bid(left.winners(i), right.winners(i))) {
            return false;
        }
    }
    for (int i = 0; i < left.winner_relays_size(); ++i) {
        if (!same_relay(left.winner_relays(i), right.winner_relays(i))) {
            return false;
        }
    }
    for (int i = 0; i < left.owners_size(); ++i) {
        if (!same_owner(left.owners(i), right.owners(i))) {
            return false;
        }
    }
    return std::equal(left.bundle_task_ids().begin(), left.bundle_task_ids().end(),
                      right.bundle_task_ids().begin());
}

std::optional<v1::AllocationBid> winner_for(const std::string& task_id, const std::string& agent_id,
                                             const BidMap& own,
                                             const std::map<std::string, v1::AllocationState>& peers,
                                             v1::AllocationPolicy policy) {
    struct Claims {
        std::optional<v1::AllocationBid> origin;
        std::uint64_t watermark{};
        bool direct{};
        std::vector<v1::AllocationBid> relays;
    };
    // Direct reports set the watermark used for relayed claims.
    std::map<std::string, Claims> claims;
    const auto own_bid = own.find(task_id);
    if (own_bid != own.end()) {
        claims[agent_id].origin = own_bid->second;
    }
    for (const auto& [peer, state] : peers) {
        auto& direct = claims[peer];
        direct.watermark = state.version();
        direct.direct = true;
        for (const auto& bid : state.bids()) {
            if (bid.task_id() == task_id) {
                direct.origin = bid;
            }
        }
        for (int index = 0; index < state.winners_size(); ++index) {
            const auto& bid = state.winners(index);
            const auto& relay = state.winner_relays(index);
            if (bid.task_id() == task_id && bid.bidder_id() != agent_id
                && std::find(relay.path_agent_ids().begin(), relay.path_agent_ids().end(), agent_id)
                       == relay.path_agent_ids().end()) {
                claims[bid.bidder_id()].relays.push_back(bid);
            }
        }
    }
    std::optional<v1::AllocationBid> winner;
    for (auto& [bidder, values] : claims) {
        std::optional<v1::AllocationBid> candidate;
        if (bidder == agent_id) {
            candidate = values.origin;
        } else {
            std::uint64_t newest{};
            for (const auto& relay : values.relays) {
                if (!values.direct || relay.version() > values.watermark) {
                    newest = std::max(newest, relay.version());
                }
            }
            if (newest != 0) {
                for (const auto& relay : values.relays) {
                    if (relay.version() != newest) {
                        continue;
                    }
                    if (!candidate) {
                        candidate = relay;
                    } else if (!same_bid(*candidate, relay)) {
                        candidate.reset();
                        break;
                    }
                }
            } else {
                candidate = values.origin;
            }
        }
        if (candidate && (!winner || better(*candidate, *winner, policy))) {
            winner = *candidate;
        }
    }
    return winner;
}

BidMap winners_for(const TaskMap& tasks, const std::string& agent_id, const BidMap& own,
                    const std::map<std::string, v1::AllocationState>& peers, v1::AllocationPolicy policy) {
    BidMap winners;
    for (const auto& [id, task] : tasks) {
        static_cast<void>(task);
        if (const auto winner = winner_for(id, agent_id, own, peers, policy)) {
            winners.emplace(id, *winner);
        }
    }
    return winners;
}

const v1::AllocationBid* winner_in(const v1::AllocationState& state, const std::string& task_id) {
    const auto position = std::lower_bound(state.winners().begin(), state.winners().end(), task_id,
                                           [](const auto& bid, const auto& id) {
                                               return bid.task_id() < id;
                                           });
    return position != state.winners().end() && position->task_id() == task_id ? &*position : nullptr;
}

std::vector<std::string> winner_path(const v1::AllocationBid& winner, const std::string& agent_id,
                                     const std::map<std::string, v1::AllocationState>& peers) {
    if (winner.bidder_id() == agent_id) {
        return {agent_id};
    }
    const auto origin = peers.find(winner.bidder_id());
    if (origin != peers.end()) {
        const auto bid = std::lower_bound(origin->second.bids().begin(), origin->second.bids().end(),
                                          winner.task_id(), [](const auto& value, const auto& id) {
                                              return value.task_id() < id;
                                          });
        if (bid != origin->second.bids().end() && bid->task_id() == winner.task_id()
            && same_bid(*bid, winner)) {
            return {winner.bidder_id(), agent_id};
        }
    }
    std::vector<std::string> selected;
    for (const auto& [peer, state] : peers) {
        static_cast<void>(peer);
        for (const auto& relay : state.winner_relays()) {
            if (!same_bid(relay.bid(), winner)
                || std::find(relay.path_agent_ids().begin(), relay.path_agent_ids().end(), agent_id)
                       != relay.path_agent_ids().end()) {
                continue;
            }
            std::vector<std::string> path(relay.path_agent_ids().begin(), relay.path_agent_ids().end());
            path.push_back(agent_id);
            if (selected.empty() || path.size() < selected.size()
                || (path.size() == selected.size() && path < selected)) {
                selected = std::move(path);
            }
        }
    }
    if (selected.empty()) {
        throw std::logic_error("winner path missing");
    }
    return selected;
}

std::vector<v1::TaskState> ordered_tasks(const std::vector<std::string>& bundle, const TaskMap& tasks) {
    std::vector<v1::TaskState> result;
    result.reserve(bundle.size());
    for (const auto& id : bundle) {
        result.push_back(tasks.at(id));
    }
    return result;
}

std::vector<std::string> ordered_path(const std::vector<std::string>& build_order, const BidMap& bids) {
    auto path = build_order;
    std::sort(path.begin(), path.end(), [&bids](const auto& left, const auto& right) {
        return std::tuple{bids.at(left).bundle_position(), left}
               < std::tuple{bids.at(right).bundle_position(), right};
    });
    return path;
}

void set_positions(BidMap& bids, const std::vector<std::string>& bundle) {
    for (std::size_t i = 0; i < bundle.size(); ++i) {
        bids.at(bundle[i]).set_bundle_position(static_cast<std::uint32_t>(i));
    }
}

bool valid_policy(v1::AllocationPolicy policy) {
    return policy == v1::ALLOCATION_POLICY_NEAREST_CAPABLE
           || policy == v1::ALLOCATION_POLICY_SENTINEL_CBBA;
}

}

class Allocator::Impl {
public:
    explicit Impl(std::string agent_id) : agent_id_(std::move(agent_id)) {
        if (agent_id_.empty()) {
            throw std::invalid_argument("empty allocator agent ID");
        }
    }

    AllocationResult update(const v1::AgentObservation& observation) {
        validate_observation(observation);
        synchronize(observation);
        auto tasks = task_map(observation);
        const auto members = peer_ids(observation.peer_ids());
        const auto peers = observation.reachable_peer_ids_size() == 0
                               ? members
                               : peer_ids(observation.reachable_peer_ids());
        auto bidders = members;
        bidders.push_back(agent_id_);
        for (const auto& detection : observation.failure_detections()) {
            std::erase(bidders, detection.failed_agent_id());
        }
        std::sort(bidders.begin(), bidders.end());
        bidders.erase(std::unique(bidders.begin(), bidders.end()), bidders.end());
        accept_messages(observation, tasks, peers, bidders, members);
        for (auto position = peer_states_.begin(); position != peer_states_.end();) {
            if (!std::binary_search(peers.begin(), peers.end(), position->first)
                || !validate_state(position->second, tasks, peers, bidders, members)) {
                position = peer_states_.erase(position);
            } else {
                ++position;
            }
        }
        merge_owners(tasks, observation);
        const auto active = unassigned_tasks(tasks);
        const auto committed = committed_tasks(tasks);

        auto generated = policy_ == v1::ALLOCATION_POLICY_NEAREST_CAPABLE
                             ? nearest_bids(active)
                             : cbba_bids(active, committed);
        const auto next_version = state_.version() + 1;
        for (auto& [id, bid] : generated) {
            const auto previous = own_bids_.find(id);
            bid.set_epoch(epoch_);
            bid.set_version(previous != own_bids_.end() && same_bid_content(bid, previous->second)
                                ? previous->second.version()
                                : next_version);
        }
        own_bids_ = generated;
        const auto winners = winners_for(active, agent_id_, own_bids_, peer_states_, policy_);
        revise_state(winners, tasks);

        AllocationResult result;
        result.state = state_;
        result.coordinated = peer_states_.size() == peers.size();
        if (serialized_version_ != state_.version()) {
            serialized_state_ = protocol::deterministic_bytes(state_);
            serialized_version_ = state_.version();
        }
        // periodic resend repairs dropped state broadcasts
        const auto interval = std::max<std::uint64_t>(
            1, state_retransmit_ms / step_ms_ + (state_retransmit_ms % step_ms_ != 0));
        if (broadcast_version_ != state_.version() || observation.tick() % interval == 0) {
            for (const auto& peer : members) {
                auto message = v1::NetworkMessage{};
                message.set_sender_id(agent_id_);
                message.set_recipient_id(peer);
                message.set_version(state_.version());
                message.set_payload(serialized_state_);
                result.outgoing_messages.push_back(std::move(message));
            }
            broadcast_version_ = state_.version();
        }
        for (const auto& [id, task] : active) {
            const auto winner = winners.find(id);
            const auto agreed = winner != winners.end() && peers_agree(id, winner->second, peers);
            result.pending = result.pending || !agreed;
            if (!agreed || winner->second.bidder_id() != agent_id_) {
                continue;
            }
            const auto claim = planning::make_claim(observation.world(), observation.self(), task,
                                                    observation.tick(), observation.step_ms(), epoch_,
                                                    winner->second.version());
            if (!claim.feasible()) {
                result.pending = true;
                continue;
            }
            auto commit = v1::AllocationCommit{};
            commit.set_epoch(epoch_);
            commit.set_version(winner->second.version());
            commit.set_task_id(id);
            commit.set_agent_id(agent_id_);
            commit.set_distance_mm(claim.distance_mm());
            commit.set_score(winner->second.score());
            commit.set_bundle_position(winner->second.bundle_position());
            result.commits.push_back(std::move(commit));
        }
        return result;
    }

private:
    void validate_observation(const v1::AgentObservation& observation) const {
        if (observation.self().id() != agent_id_ || observation.allocation_epoch() == 0
            || observation.world().map_version() == 0 || observation.step_ms() == 0
            || !valid_policy(observation.allocation_policy())) {
            throw std::invalid_argument("invalid allocator observation");
        }
    }

    void synchronize(const v1::AgentObservation& observation) {
        if (epoch_ == observation.allocation_epoch() && map_version_ == observation.world().map_version()
            && policy_ == observation.allocation_policy()) {
            return;
        }
        epoch_ = observation.allocation_epoch();
        map_version_ = observation.world().map_version();
        policy_ = observation.allocation_policy();
        basis_tick_ = observation.tick();
        step_ms_ = observation.step_ms();
        basis_self_ = observation.self();
        basis_world_ = observation.world();
        evaluator_ = policy_ == v1::ALLOCATION_POLICY_SENTINEL_CBBA
                         ? std::make_unique<planning::BundleEvaluator>(basis_world_, basis_self_, basis_tick_, step_ms_)
                         : nullptr;
        state_.Clear();
        serialized_state_.clear();
        serialized_version_ = 0;
        broadcast_version_ = 0;
        own_bids_.clear();
        peer_states_.clear();
        build_order_.clear();
    }

    TaskMap task_map(const v1::AgentObservation& observation) const {
        TaskMap tasks;
        for (const auto& task : observation.known_tasks()) {
            if (task.status() == v1::TASK_STATUS_PENDING) {
                tasks.emplace(task.id(), task);
            }
        }
        return tasks;
    }

    TaskMap unassigned_tasks(const TaskMap& tasks) const {
        TaskMap result;
        for (const auto& [id, task] : tasks) {
            if (task.assigned_agent_id().empty()) {
                result.emplace(id, task);
            }
        }
        return result;
    }

    std::vector<v1::TaskState> committed_tasks(const TaskMap& source) const {
        std::vector<v1::TaskState> tasks;
        for (const auto& [id, task] : source) {
            static_cast<void>(id);
            if (task.status() == v1::TASK_STATUS_PENDING && task.assigned_agent_id() == agent_id_) {
                tasks.push_back(task);
            }
        }
        std::sort(tasks.begin(), tasks.end(), [](const auto& left, const auto& right) {
            return std::tuple{left.allocation_epoch(), left.bundle_position(), left.deadline_tick(), left.id()}
                   < std::tuple{right.allocation_epoch(), right.bundle_position(), right.deadline_tick(), right.id()};
        });
        return tasks;
    }

    template <typename Range>
    std::vector<std::string> peer_ids(const Range& values) const {
        std::vector<std::string> peers;
        for (const auto& peer : values) {
            if (peer != agent_id_) {
                peers.push_back(peer);
            }
        }
        std::sort(peers.begin(), peers.end());
        peers.erase(std::unique(peers.begin(), peers.end()), peers.end());
        return peers;
    }

    bool validate_bid(const v1::AllocationBid& bid, const TaskMap& tasks,
                      const std::vector<std::string>& bidders) const {
        const auto task = tasks.find(bid.task_id());
        return bid.epoch() == epoch_ && bid.version() != 0 && task != tasks.end()
               && std::binary_search(bidders.begin(), bidders.end(), bid.bidder_id())
               && bid.distance_mm() >= 0 && bid.energy_mj() >= 0
               && bid.completion_tick() <= task->second.deadline_tick()
               && bid.bundle_position() < tasks.size();
    }

    bool validate_state(v1::AllocationState& state, const TaskMap& tasks,
                        const std::vector<std::string>& peers,
                        const std::vector<std::string>& bidders,
                        const std::vector<std::string>& members) const {
        if (state.epoch() != epoch_ || state.version() == 0 || state.map_version() != map_version_
            || state.sender_id() == agent_id_
            || !std::binary_search(peers.begin(), peers.end(), state.sender_id())
            || !std::binary_search(bidders.begin(), bidders.end(), state.sender_id())) {
            return false;
        }
        std::sort(state.mutable_bids()->begin(), state.mutable_bids()->end(), [](const auto& left, const auto& right) {
            return left.task_id() < right.task_id();
        });
        std::sort(state.mutable_winners()->begin(), state.mutable_winners()->end(),
                  [](const auto& left, const auto& right) {
                      return left.task_id() < right.task_id();
                  });
        std::sort(state.mutable_winner_relays()->begin(), state.mutable_winner_relays()->end(),
                  [](const auto& left, const auto& right) {
                      return left.bid().task_id() < right.bid().task_id();
                  });
        std::sort(state.mutable_owners()->begin(), state.mutable_owners()->end(), [](const auto& left, const auto& right) {
            return left.task_id() < right.task_id();
        });
        for (int index = 0; index < state.bids_size(); ++index) {
            const auto& bid = state.bids(index);
            if (!validate_bid(bid, tasks, bidders) || bid.bidder_id() != state.sender_id()
                || bid.version() > state.version()
                || (policy_ == v1::ALLOCATION_POLICY_NEAREST_CAPABLE
                    && (bid.score() != 0 || bid.bundle_position() != 0))
                || (index != 0 && state.bids(index - 1).task_id() == bid.task_id())) {
                return false;
            }
        }
        if (state.winner_relays_size() != state.winners_size()) {
            return false;
        }
        for (int index = 0; index < state.winners_size(); ++index) {
            const auto& bid = state.winners(index);
            const auto& relay = state.winner_relays(index);
            if (!validate_bid(bid, tasks, bidders)
                || !same_bid(relay.bid(), bid) || relay.path_agent_ids().empty()
                || relay.path_agent_ids(0) != bid.bidder_id()
                || relay.path_agent_ids(relay.path_agent_ids_size() - 1) != state.sender_id()
                || (policy_ == v1::ALLOCATION_POLICY_NEAREST_CAPABLE
                    && (bid.score() != 0 || bid.bundle_position() != 0))
                || (index != 0 && state.winners(index - 1).task_id() == bid.task_id())) {
                return false;
            }
            std::vector<std::string> path(relay.path_agent_ids().begin(), relay.path_agent_ids().end());
            auto unique_path = path;
            std::sort(unique_path.begin(), unique_path.end());
            if (std::adjacent_find(unique_path.begin(), unique_path.end()) != unique_path.end()
                || !std::all_of(path.begin(), path.end(), [&bidders](const auto& agent) {
                       return std::binary_search(bidders.begin(), bidders.end(), agent);
                   })) {
                return false;
            }
            const auto origin = std::lower_bound(state.bids().begin(), state.bids().end(), bid.task_id(),
                                                 [](const auto& value, const auto& id) {
                                                     return value.task_id() < id;
                                                 });
            if (bid.bidder_id() == state.sender_id()
                && (origin == state.bids().end() || origin->task_id() != bid.task_id()
                    || !same_bid(*origin, bid) || path.size() != 1)) {
                return false;
            }
        }
        auto owners = members;
        owners.push_back(agent_id_);
        std::sort(owners.begin(), owners.end());
        for (int index = 0; index < state.owners_size(); ++index) {
            const auto& owner = state.owners(index);
            if (!tasks.contains(owner.task_id())
                || !std::binary_search(owners.begin(), owners.end(), owner.agent_id())
                || owner.epoch() == 0 || owner.epoch() > epoch_ || owner.version() == 0
                || owner.distance_mm() != 0
                || (index != 0 && state.owners(index - 1).task_id() == owner.task_id())) {
                return false;
            }
        }
        std::vector<std::string> bundle(state.bundle_task_ids().begin(), state.bundle_task_ids().end());
        auto unique = bundle;
        std::sort(unique.begin(), unique.end());
        if (std::adjacent_find(unique.begin(), unique.end()) != unique.end()
            || (policy_ == v1::ALLOCATION_POLICY_NEAREST_CAPABLE && !bundle.empty())
            || (policy_ == v1::ALLOCATION_POLICY_SENTINEL_CBBA
                && bundle.size() != static_cast<std::size_t>(state.bids_size()))) {
            return false;
        }
        std::vector<std::uint32_t> positions;
        for (const auto& id : bundle) {
            const auto bid = std::lower_bound(state.bids().begin(), state.bids().end(), id,
                                              [](const auto& value, const auto& id) {
                                                  return value.task_id() < id;
                                              });
            if (bid == state.bids().end() || bid->task_id() != id
                || bid->bundle_position() >= bundle.size()) {
                return false;
            }
            positions.push_back(bid->bundle_position());
        }
        std::sort(positions.begin(), positions.end());
        for (std::size_t idx = 0; idx < positions.size(); ++idx) {
            if (positions[idx] != idx) {
                return false;
            }
        }
        return true;
    }

    void accept_messages(const v1::AgentObservation& observation, const TaskMap& tasks,
                         const std::vector<std::string>& peers,
                         const std::vector<std::string>& bidders,
                         const std::vector<std::string>& members) {
        struct Candidate {
            v1::AllocationState state;
            std::string bytes;
            bool conflict{};
        };
        // conflicting bytes invalidate this sender for the tick.
        std::map<std::string, Candidate> candidates;
        for (const auto& message : observation.delivered_messages()) {
            v1::AllocationState incoming;
            if (message.recipient_id() != agent_id_ || message.sender_id() == agent_id_
                || !std::binary_search(peers.begin(), peers.end(), message.sender_id())
                || !incoming.ParseFromString(message.payload()) || incoming.sender_id() != message.sender_id()
                || incoming.version() != message.version()
                || !validate_state(incoming, tasks, peers, bidders, members)) {
                continue;
            }
            const auto bytes = protocol::deterministic_bytes(incoming);
            auto position = candidates.find(message.sender_id());
            if (position == candidates.end() || incoming.version() > position->second.state.version()) {
                candidates[message.sender_id()] = Candidate{std::move(incoming), bytes, false};
            } else if (incoming.version() == position->second.state.version()
                       && bytes != position->second.bytes) {
                position->second.conflict = true;
            }
        }
        for (auto& [sender, candidate] : candidates) {
            if (candidate.conflict) {
                continue;
            }
            const auto current = peer_states_.find(sender);
            if (current == peer_states_.end() || candidate.state.version() > current->second.version()) {
                peer_states_[sender] = std::move(candidate.state);
            }
        }
    }

    void merge_owners(TaskMap& tasks, const v1::AgentObservation& observation) const {
        struct Candidate {
            v1::AllocationCommit owner;
            bool direct{};
        };
        std::vector<Candidate> candidates;
        std::set<std::pair<std::string, std::string>> unavailable;
        for (const auto& [id, task] : tasks) {
            if (task.assigned_agent_id().empty()) {
                unavailable.emplace(id, agent_id_);
                continue;
            }
            auto owner = v1::AllocationCommit{};
            owner.set_epoch(task.allocation_epoch());
            owner.set_version(task.allocation_version());
            owner.set_task_id(id);
            owner.set_agent_id(task.assigned_agent_id());
            owner.set_score(task.allocation_score());
            owner.set_bundle_position(task.bundle_position());
            candidates.push_back({std::move(owner), task.assigned_agent_id() == agent_id_});
            if (task.assigned_agent_id() != agent_id_) {
                unavailable.emplace(id, agent_id_);
            }
        }
        for (const auto& detection : observation.failure_detections()) {
            for (const auto& [id, task] : tasks) {
                static_cast<void>(task);
                unavailable.emplace(id, detection.failed_agent_id());
            }
        }
        for (const auto& [peer, state] : peer_states_) {
            for (const auto& [id, task] : tasks) {
                static_cast<void>(task);
                const auto owned = std::any_of(state.owners().begin(), state.owners().end(),
                                               [&peer, &id](const auto& owner) {
                                                   return owner.task_id() == id && owner.agent_id() == peer;
                                               });
                if (!owned) {
                    unavailable.emplace(id, peer);
                }
            }
            for (const auto& owner : state.owners()) {
                candidates.push_back({owner, owner.agent_id() == peer});
            }
        }
        std::map<std::string, Candidate> selected;
        for (auto& candidate : candidates) {
            if (unavailable.contains({candidate.owner.task_id(), candidate.owner.agent_id()})) {
                continue;
            }
            const auto current = selected.find(candidate.owner.task_id());
            if (current == selected.end()) {
                selected.emplace(candidate.owner.task_id(), std::move(candidate));
                continue;
            }
            const auto& value = current->second;
            const auto replace = candidate.direct != value.direct ? candidate.direct
                                 : candidate.owner.epoch() != value.owner.epoch()
                                     ? candidate.owner.epoch() > value.owner.epoch()
                                 : candidate.owner.version() != value.owner.version()
                                     ? candidate.owner.version() > value.owner.version()
                                 : candidate.owner.score() != value.owner.score()
                                     ? candidate.owner.score() > value.owner.score()
                                 : candidate.owner.agent_id() != value.owner.agent_id()
                                     ? candidate.owner.agent_id() < value.owner.agent_id()
                                     : candidate.owner.bundle_position() < value.owner.bundle_position();
            if (replace) {
                current->second = std::move(candidate);
            }
        }
        for (auto& [id, task] : tasks) {
            const auto progress = task.assigned_agent_id() == agent_id_ ? task.progress_ticks() : 0;
            task.clear_assigned_agent_id();
            task.set_allocation_epoch(0);
            task.set_allocation_version(0);
            task.set_allocation_score(0);
            task.set_bundle_position(0);
            task.set_progress_ticks(0);
            const auto owner = selected.find(id);
            if (owner == selected.end()) {
                continue;
            }
            task.set_assigned_agent_id(owner->second.owner.agent_id());
            task.set_allocation_epoch(owner->second.owner.epoch());
            task.set_allocation_version(owner->second.owner.version());
            task.set_allocation_score(owner->second.owner.score());
            task.set_bundle_position(owner->second.owner.bundle_position());
            task.set_progress_ticks(owner->second.owner.agent_id() == agent_id_ ? progress : 0);
        }
    }

    BidMap nearest_bids(const TaskMap& tasks) const {
        BidMap bids;
        for (const auto& [id, task] : tasks) {
            const auto route = planning::feasible_route(basis_world_, basis_self_, task, basis_tick_, step_ms_);
            if (!route) {
                continue;
            }
            auto bid = v1::AllocationBid{};
            bid.set_task_id(id);
            bid.set_bidder_id(agent_id_);
            bid.set_distance_mm(route->distance_mm);
            bid.set_energy_mj(route->energy_mj + static_cast<std::int64_t>(task.service_ticks())
                                                     * task.service_energy_mj_per_tick());
            bid.set_completion_tick(basis_tick_ + route->travel_ticks + task.service_ticks());
            bids.emplace(id, std::move(bid));
        }
        return bids;
    }

    BidMap cbba_bids(const TaskMap& tasks, const std::vector<v1::TaskState>& committed) {
        std::vector<std::string> build_order;
        BidMap bids;
        for (const auto& id : build_order_) {
            const auto task = tasks.find(id);
            const auto bid = own_bids_.find(id);
            if (task == tasks.end() || bid == own_bids_.end()) {
                break;
            }
            build_order.push_back(id);
            bids.emplace(id, bid->second);
        }
        const auto current_winners = winners_for(tasks, agent_id_, bids, peer_states_, policy_);
        std::size_t retained{};
        // CBBA drops the dependent suffix after a lost bundle item.
        while (retained < build_order.size()) {
            const auto winner = current_winners.find(build_order[retained]);
            if (winner == current_winners.end() || winner->second.bidder_id() != agent_id_) {
                break;
            }
            ++retained;
        }
        while (build_order.size() > retained) {
            bids.erase(build_order.back());
            build_order.pop_back();
        }
        auto path = ordered_path(build_order, bids);
        set_positions(bids, path);
        while (build_order.size() < tasks.size()) {
            struct Choice {
                std::string task_id;
                planning::BundleInsertion insertion;
            };
            std::optional<Choice> choice;
            auto ordered = committed;
            const auto auction = ordered_tasks(path, tasks);
            ordered.insert(ordered.end(), auction.begin(), auction.end());
            for (const auto& [id, task] : tasks) {
                if (bids.contains(id)) {
                    continue;
                }
                const auto insertion = evaluator_->best_insertion(ordered, task, committed.size());
                if (!insertion) {
                    continue;
                }
                v1::AllocationBid candidate;
                candidate.set_task_id(id);
                candidate.set_bidder_id(agent_id_);
                candidate.set_score(insertion->score);
                candidate.set_distance_mm(insertion->distance_mm);
                candidate.set_energy_mj(insertion->energy_mj);
                candidate.set_completion_tick(insertion->completion_tick);
                candidate.set_bundle_position(
                    static_cast<std::uint32_t>(insertion->index - committed.size()));
                const auto remote = winner_for(id, agent_id_, {}, peer_states_, policy_);
                if (remote && !better(candidate, *remote, policy_)) {
                    continue;
                }
                if (!choice) {
                    choice = Choice{id, *insertion};
                } else {
                    const auto& current = choice->insertion;
                    if (insertion->score > current.score
                        || (insertion->score == current.score
                            && std::tuple{insertion->distance_mm, insertion->energy_mj,
                                          insertion->completion_tick, id, insertion->index}
                                   < std::tuple{current.distance_mm, current.energy_mj,
                                                current.completion_tick, choice->task_id, current.index})) {
                        choice = Choice{id, *insertion};
                    }
                }
            }
            if (!choice) {
                break;
            }
            auto bid = v1::AllocationBid{};
            bid.set_task_id(choice->task_id);
            bid.set_bidder_id(agent_id_);
            bid.set_score(choice->insertion.score);
            bid.set_distance_mm(choice->insertion.distance_mm);
            bid.set_energy_mj(choice->insertion.energy_mj);
            bid.set_completion_tick(choice->insertion.completion_tick);
            const auto index = choice->insertion.index - committed.size();
            bid.set_bundle_position(static_cast<std::uint32_t>(index));
            bids.emplace(choice->task_id, std::move(bid));
            path.insert(path.begin() + static_cast<std::ptrdiff_t>(index), choice->task_id);
            build_order.push_back(choice->task_id);
            set_positions(bids, path);
        }
        build_order_ = std::move(build_order);
        return bids;
    }

    void revise_state(const BidMap& winners, const TaskMap& tasks) {
        v1::AllocationState next;
        next.set_epoch(epoch_);
        next.set_sender_id(agent_id_);
        next.set_map_version(map_version_);
        for (const auto& [id, bid] : own_bids_) {
            static_cast<void>(id);
            next.add_bids()->CopyFrom(bid);
        }
        for (const auto& [id, bid] : winners) {
            static_cast<void>(id);
            next.add_winners()->CopyFrom(bid);
            auto* relay = next.add_winner_relays();
            relay->mutable_bid()->CopyFrom(bid);
            for (const auto& agent : winner_path(bid, agent_id_, peer_states_)) {
                relay->add_path_agent_ids(agent);
            }
        }
        for (const auto& id : build_order_) {
            next.add_bundle_task_ids(id);
        }
        for (const auto& [id, task] : tasks) {
            static_cast<void>(id);
            if (task.assigned_agent_id().empty()) {
                continue;
            }
            auto* owner = next.add_owners();
            owner->set_epoch(task.allocation_epoch());
            owner->set_version(task.allocation_version());
            owner->set_task_id(task.id());
            owner->set_agent_id(task.assigned_agent_id());
            owner->set_score(task.allocation_score());
            owner->set_bundle_position(task.bundle_position());
        }
        next.set_version(same_state(next, state_) ? state_.version() : state_.version() + 1);
        state_ = std::move(next);
    }

    bool peers_agree(const std::string& task_id, const v1::AllocationBid& winner,
                     const std::vector<std::string>& peers) const {
        for (const auto& peer : peers) {
            const auto state = peer_states_.find(peer);
            if (state == peer_states_.end()) {
                return false;
            }
            const auto peer_winner = winner_in(state->second, task_id);
            if (!peer_winner || !same_bid(*peer_winner, winner)) {
                return false;
            }
        }
        return true;
    }

    std::string agent_id_;
    std::uint64_t epoch_{};
    std::uint64_t map_version_{};
    std::uint64_t basis_tick_{};
    std::uint64_t step_ms_{};
    v1::AllocationPolicy policy_{v1::ALLOCATION_POLICY_UNSPECIFIED};
    v1::VehicleState basis_self_;
    v1::World basis_world_;
    std::unique_ptr<planning::BundleEvaluator> evaluator_;
    v1::AllocationState state_;
    std::string serialized_state_;
    std::uint64_t serialized_version_{};
    std::uint64_t broadcast_version_{};
    BidMap own_bids_;
    std::map<std::string, v1::AllocationState> peer_states_;
    std::vector<std::string> build_order_;
};

Allocator::Allocator(std::string agent_id) : impl_(std::make_unique<Impl>(std::move(agent_id))) {}

Allocator::~Allocator() = default;

AllocationResult Allocator::update(const v1::AgentObservation& observation) {
    return impl_->update(observation);
}

}
