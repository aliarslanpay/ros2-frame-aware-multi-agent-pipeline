#ifndef SWARM_COORDINATOR__STATE_FRAME_NORMALIZER_HPP_
#define SWARM_COORDINATOR__STATE_FRAME_NORMALIZER_HPP_

#include <functional>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"

namespace swarm_coordinator
{

struct TransformLookupResult
{
  std::optional<geometry_msgs::msg::TransformStamped> transform;
  std::string detail;
};

using TransformLookup = std::function<TransformLookupResult(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & source_stamp)>;

enum class NormalizationStatus
{
  kSuccess,
  kEmptySourceFrame,
  kUnexpectedSourceFrame,
  kEmptyChildFrame,
  kUnexpectedChildFrame,
  kInvalidSourceTimestamp,
  kInvalidPoseQuaternion,
  kTransformUnavailable,
  kTransformFrameMismatch,
  kInvalidTransformQuaternion,
  kTransformApplicationFailed
};

struct NormalizationResult
{
  NormalizationStatus status;
  std::optional<swarm_interfaces::msg::AgentState> state;
  std::string detail;

  [[nodiscard]] bool success() const noexcept
  {
    return status == NormalizationStatus::kSuccess && state.has_value();
  }
};

[[nodiscard]] const char * to_string(NormalizationStatus status) noexcept;

// Immutable, concurrency-safe frame-normalization policy. The supplied lookup
// callback must itself be safe for concurrent calls; tf2_ros::Buffer satisfies
// that requirement. The normalizer never queues messages and never mutates the
// coordinator cache.
class StateFrameNormalizer
{
public:
  StateFrameNormalizer(std::string target_frame, TransformLookup transform_lookup);

  [[nodiscard]] NormalizationResult normalize(
    const std::string & expected_agent_id,
    const swarm_interfaces::msg::AgentState & source) const;

  [[nodiscard]] const std::string & target_frame() const noexcept
  {
    return target_frame_;
  }

private:
  std::string target_frame_;
  TransformLookup transform_lookup_;
};

}  // namespace swarm_coordinator

#endif  // SWARM_COORDINATOR__STATE_FRAME_NORMALIZER_HPP_
