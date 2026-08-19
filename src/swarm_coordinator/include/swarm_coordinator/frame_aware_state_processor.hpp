#ifndef SWARM_COORDINATOR__FRAME_AWARE_STATE_PROCESSOR_HPP_
#define SWARM_COORDINATOR__FRAME_AWARE_STATE_PROCESSOR_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "swarm_coordinator/agent_state_cache.hpp"
#include "swarm_coordinator/state_frame_normalizer.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"

namespace swarm_coordinator
{

struct ProcessResult
{
  NormalizationResult normalization;
  std::optional<UpdateResult> update;

  [[nodiscard]] bool accepted() const noexcept
  {
    return update.has_value() && update->accepted();
  }
};

// This class makes the critical processing invariant structural: normalization must
// succeed before AgentStateCache::update() can run. A transform rejection can
// therefore never replace state or refresh receipt time.
class FrameAwareStateProcessor
{
public:
  FrameAwareStateProcessor(std::string target_frame, TransformLookup transform_lookup);

  [[nodiscard]] ProcessResult process(
    const std::string & expected_agent_id,
    const swarm_interfaces::msg::AgentState & source,
    SteadyTimePoint received_at);

  [[nodiscard]] std::vector<CachedAgentSnapshot> snapshot(
    const std::vector<std::string> & agent_ids,
    SteadyTimePoint now,
    std::chrono::milliseconds stale_timeout) const;

private:
  StateFrameNormalizer normalizer_;
  AgentStateCache cache_;
};

}  // namespace swarm_coordinator

#endif  // SWARM_COORDINATOR__FRAME_AWARE_STATE_PROCESSOR_HPP_
