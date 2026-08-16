#include "swarm_coordinator/frame_aware_state_processor.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace swarm_coordinator
{

FrameAwareStateProcessor::FrameAwareStateProcessor(
  std::string target_frame,
  TransformLookup transform_lookup)
: normalizer_(std::move(target_frame), std::move(transform_lookup))
{
}

ProcessResult FrameAwareStateProcessor::process(
  const std::string & expected_agent_id,
  const swarm_interfaces::msg::AgentState & source,
  const SteadyTimePoint received_at)
{
  auto normalization = normalizer_.normalize(expected_agent_id, source);
  if (!normalization.success()) {
    return {std::move(normalization), std::nullopt};
  }

  const UpdateResult update = cache_.update(
    expected_agent_id, *normalization.state, received_at);
  return {std::move(normalization), update};
}

std::vector<AgentSnapshot> FrameAwareStateProcessor::snapshot(
  const std::vector<std::string> & agent_ids,
  const SteadyTimePoint now,
  const std::chrono::milliseconds stale_timeout) const
{
  return cache_.snapshot(agent_ids, now, stale_timeout);
}

}  // namespace swarm_coordinator
