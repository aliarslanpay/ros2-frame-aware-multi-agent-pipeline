#include "swarm_coordinator/snapshot_message_builder.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "builtin_interfaces/msg/duration.hpp"
#include "swarm_interfaces/msg/agent_snapshot.hpp"

namespace swarm_coordinator
{
namespace
{

constexpr std::int64_t kMillisecondsPerSecond = 1000;
constexpr std::uint32_t kNanosecondsPerMillisecond = 1'000'000U;
constexpr std::uint32_t kMaximumNanoseconds = 999'999'999U;

[[nodiscard]] builtin_interfaces::msg::Duration to_duration_message(
  const std::chrono::milliseconds age) noexcept
{
  builtin_interfaces::msg::Duration result;
  const std::int64_t milliseconds = age.count();
  if (milliseconds <= 0) {
    return result;
  }

  constexpr std::int64_t maximum_seconds =
    std::numeric_limits<std::int32_t>::max();
  constexpr std::int64_t maximum_milliseconds =
    maximum_seconds * kMillisecondsPerSecond + (kMillisecondsPerSecond - 1);

  if (milliseconds > maximum_milliseconds) {
    result.sec = std::numeric_limits<std::int32_t>::max();
    result.nanosec = kMaximumNanoseconds;
    return result;
  }

  result.sec = static_cast<std::int32_t>(milliseconds / kMillisecondsPerSecond);
  result.nanosec = static_cast<std::uint32_t>(
    milliseconds % kMillisecondsPerSecond) * kNanosecondsPerMillisecond;
  return result;
}

[[nodiscard]] std::uint8_t to_message_freshness(const Freshness freshness)
{
  switch (freshness) {
    case Freshness::kFresh:
      return swarm_interfaces::msg::AgentSnapshot::FRESH;
    case Freshness::kStale:
      return swarm_interfaces::msg::AgentSnapshot::STALE;
    case Freshness::kNeverSeen:
      throw std::logic_error("state-bearing snapshot cannot be NEVER_SEEN");
  }

  throw std::logic_error("unknown internal freshness value");
}

[[nodiscard]] swarm_interfaces::msg::AgentSnapshot make_agent_snapshot_message(
  const CachedAgentSnapshot & snapshot)
{
  swarm_interfaces::msg::AgentSnapshot result;
  result.agent_id = snapshot.agent_id;

  if (!snapshot.state) {
    result.has_state = false;
    result.freshness = swarm_interfaces::msg::AgentSnapshot::NEVER_SEEN;
    return result;
  }

  if (!snapshot.age.has_value()) {
    throw std::logic_error("state-bearing snapshot must have an age");
  }

  result.has_state = true;
  result.freshness = to_message_freshness(snapshot.freshness);
  result.state = snapshot.state->message;
  result.age = to_duration_message(*snapshot.age);
  return result;
}

}  // namespace

swarm_interfaces::msg::SwarmSnapshot make_swarm_snapshot_message(
  const std::vector<CachedAgentSnapshot> & snapshots,
  const builtin_interfaces::msg::Time & creation_time,
  const std::string & target_frame,
  const std::uint64_t sequence)
{
  swarm_interfaces::msg::SwarmSnapshot result;
  result.header.stamp = creation_time;
  result.header.frame_id = target_frame;
  result.sequence = sequence;
  result.agents.reserve(snapshots.size());

  for (const auto & snapshot : snapshots) {
    result.agents.push_back(make_agent_snapshot_message(snapshot));
  }

  return result;
}

}  // namespace swarm_coordinator
