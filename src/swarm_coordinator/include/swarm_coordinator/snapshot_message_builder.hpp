#ifndef SWARM_COORDINATOR__SNAPSHOT_MESSAGE_BUILDER_HPP_
#define SWARM_COORDINATOR__SNAPSHOT_MESSAGE_BUILDER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "swarm_coordinator/agent_state_cache.hpp"
#include "swarm_interfaces/msg/swarm_snapshot.hpp"

namespace swarm_coordinator
{

// Converts one immutable, coherent cache view into the consumer-facing ROS
// contract. No cache lock is held by this function.
[[nodiscard]] swarm_interfaces::msg::SwarmSnapshot make_swarm_snapshot_message(
  const std::vector<CachedAgentSnapshot> & snapshots,
  const builtin_interfaces::msg::Time & creation_time,
  const std::string & target_frame,
  std::uint64_t sequence);

}  // namespace swarm_coordinator

#endif  // SWARM_COORDINATOR__SNAPSHOT_MESSAGE_BUILDER_HPP_
