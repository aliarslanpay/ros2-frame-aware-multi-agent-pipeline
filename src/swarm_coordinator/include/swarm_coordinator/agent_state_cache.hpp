#ifndef SWARM_COORDINATOR__AGENT_STATE_CACHE_HPP_
#define SWARM_COORDINATOR__AGENT_STATE_CACHE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarm_interfaces/msg/agent_state.hpp"

namespace swarm_coordinator
{

using SteadyTimePoint = std::chrono::steady_clock::time_point;

enum class UpdateStatus
{
  kAcceptedFirst,
  kAccepted,
  kRejectedIdentityMismatch,
  kRejectedInvalidTimestamp,
  kRejectedTimestamp,
  kRejectedSequence
};

struct UpdateResult
{
  UpdateStatus status;
  std::uint64_t missing_samples{0};

  [[nodiscard]] bool accepted() const noexcept
  {
    return status == UpdateStatus::kAcceptedFirst || status == UpdateStatus::kAccepted;
  }
};

struct CachedAgentState
{
  swarm_interfaces::msg::AgentState message;
  SteadyTimePoint received_at;
  std::int64_t source_timestamp_ns;
};

enum class Freshness
{
  kNeverSeen,
  kFresh,
  kStale
};

struct CachedAgentSnapshot
{
  std::string agent_id;
  Freshness freshness{Freshness::kNeverSeen};
  std::shared_ptr<const CachedAgentState> state;
  std::optional<std::chrono::milliseconds> age;
};

// Thread-safety contract:
// - update() and snapshot() may be called concurrently from any thread;
// - mutex_ protects the map and all pointer replacements;
// - cached records are immutable and published through shared_ptr;
// - formatting, logging, and full message copies happen outside mutex_;
// - ordering validation and pointer replacement form one atomic critical section.
class AgentStateCache
{
public:
  [[nodiscard]] UpdateResult update(
    const std::string & expected_agent_id,
    const swarm_interfaces::msg::AgentState & message,
    SteadyTimePoint received_at);

  [[nodiscard]] std::vector<CachedAgentSnapshot> snapshot(
    const std::vector<std::string> & agent_ids,
    SteadyTimePoint now,
    std::chrono::milliseconds stale_timeout) const;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<const CachedAgentState>> states_;
};

}  // namespace swarm_coordinator

#endif  // SWARM_COORDINATOR__AGENT_STATE_CACHE_HPP_
