#include "swarm_core/state_acceptance.hpp"

namespace swarm_core
{

bool should_accept(
  const std::int64_t incoming_timestamp_ns,
  const std::optional<std::int64_t> last_timestamp_ns) noexcept
{
  return !last_timestamp_ns.has_value() ||
         incoming_timestamp_ns > last_timestamp_ns.value();
}

}  // namespace swarm_core

