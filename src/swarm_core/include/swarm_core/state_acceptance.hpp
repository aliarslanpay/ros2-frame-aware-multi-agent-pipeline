#ifndef SWARM_CORE__STATE_ACCEPTANCE_HPP_
#define SWARM_CORE__STATE_ACCEPTANCE_HPP_

#include <cstdint>
#include <optional>

namespace swarm_core
{

// Accept the first state and then only states with a strictly newer source
// timestamp. Equality is a duplicate; a lower timestamp is out of order.
[[nodiscard]] bool should_accept(
  std::int64_t incoming_timestamp_ns,
  std::optional<std::int64_t> last_timestamp_ns) noexcept;

}  // namespace swarm_core

#endif  // SWARM_CORE__STATE_ACCEPTANCE_HPP_

