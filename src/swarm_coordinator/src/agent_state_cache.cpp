#include "swarm_coordinator/agent_state_cache.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "swarm_core/state_acceptance.hpp"

namespace swarm_coordinator
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;

[[nodiscard]] bool has_valid_nanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  return stamp.nanosec < static_cast<std::uint32_t>(kNanosecondsPerSecond);
}

[[nodiscard]] std::int64_t to_nanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

UpdateResult AgentStateCache::update(
  const std::string & expected_agent_id,
  const swarm_interfaces::msg::AgentState & message,
  const SteadyTimePoint received_at)
{
  if (expected_agent_id.empty() || message.agent_id != expected_agent_id) {
    return {UpdateStatus::kRejectedIdentityMismatch, 0};
  }

  if (!has_valid_nanoseconds(message.header.stamp)) {
    return {UpdateStatus::kRejectedInvalidTimestamp, 0};
  }

  const std::int64_t timestamp_ns = to_nanoseconds(message.header.stamp);
  // Build the immutable replacement before taking the lock. Allocation and the
  // potentially expensive ROS-message copy therefore cannot extend lock hold
  // time. If construction throws, the cache remains unchanged.
  const auto candidate = std::make_shared<const CachedAgentState>(
    CachedAgentState{message, received_at, timestamp_ns});

  std::lock_guard<std::mutex> lock(mutex_);
  const auto previous = states_.find(expected_agent_id);

  if (previous == states_.end()) {
    states_.emplace(expected_agent_id, candidate);
    return {UpdateStatus::kAcceptedFirst, 0};
  }

  if (!swarm_core::should_accept(
      timestamp_ns,
      std::optional<std::int64_t>{previous->second->source_timestamp_ns}))
  {
    return {UpdateStatus::kRejectedTimestamp, 0};
  }

  if (message.sequence <= previous->second->message.sequence) {
    return {UpdateStatus::kRejectedSequence, 0};
  }

  const std::uint64_t missing_samples =
    message.sequence - previous->second->message.sequence - 1;

  // shared_ptr assignment is noexcept: readers holding the previous immutable
  // record remain valid while the map atomically moves to the new record.
  previous->second = candidate;
  return {UpdateStatus::kAccepted, missing_samples};
}

std::vector<CachedAgentSnapshot> AgentStateCache::snapshot(
  const std::vector<std::string> & agent_ids,
  const SteadyTimePoint now,
  const std::chrono::milliseconds stale_timeout) const
{
  if (stale_timeout < std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("stale_timeout must not be negative");
  }

  // Allocate and copy configured IDs before locking. Inside the critical
  // section we only acquire immutable shared_ptr snapshots and calculate age.
  std::vector<CachedAgentSnapshot> snapshots;
  snapshots.reserve(agent_ids.size());
  for (const auto & agent_id : agent_ids) {
    snapshots.push_back(
      CachedAgentSnapshot{agent_id, Freshness::kNeverSeen, nullptr, std::nullopt});
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & snapshot : snapshots) {
      const auto state = states_.find(snapshot.agent_id);
      if (state == states_.end()) {
        continue;
      }

      snapshot.state = state->second;

      // A summary can capture 'now' immediately before a concurrent update
      // records its receipt time. Clamp that valid concurrency edge case to zero.
      const auto age = now >= snapshot.state->received_at ?
        std::chrono::duration_cast<std::chrono::milliseconds>(
          now - snapshot.state->received_at) :
        std::chrono::milliseconds::zero();

      snapshot.age = age;
      snapshot.freshness = age > stale_timeout ?
        Freshness::kStale : Freshness::kFresh;
    }
  }

  return snapshots;
}

}  // namespace swarm_coordinator
