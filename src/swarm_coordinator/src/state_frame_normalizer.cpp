#include "swarm_coordinator/state_frame_normalizer.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace swarm_coordinator
{
namespace
{

constexpr std::uint32_t kNanosecondsPerSecond = 1'000'000'000U;
constexpr double kMinimumQuaternionNormSquared = 1.0e-12;

[[nodiscard]] bool valid_source_timestamp(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  if (stamp.nanosec >= kNanosecondsPerSecond) {
    return false;
  }

  // In tf2, time zero means "latest". Reject it so callers cannot
  // accidentally bypass the source-timestamp contract.
  return stamp.sec != 0 || stamp.nanosec != 0U;
}

[[nodiscard]] bool valid_quaternion(
  const geometry_msgs::msg::Quaternion & quaternion) noexcept
{
  const bool finite =
    std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
    std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
  if (!finite) {
    return false;
  }

  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return norm_squared > kMinimumQuaternionNormSquared;
}

[[nodiscard]] geometry_msgs::msg::Vector3 rotate_vector(
  const geometry_msgs::msg::Vector3 & source,
  const tf2::Quaternion & target_from_source_rotation)
{
  const tf2::Vector3 input{source.x, source.y, source.z};
  const tf2::Vector3 output = tf2::quatRotate(target_from_source_rotation, input);

  geometry_msgs::msg::Vector3 result;
  result.x = output.x();
  result.y = output.y();
  result.z = output.z();
  return result;
}

[[nodiscard]] NormalizationResult reject(
  const NormalizationStatus status,
  std::string detail)
{
  return {status, std::nullopt, std::move(detail)};
}

}  // namespace

StateFrameNormalizer::StateFrameNormalizer(
  std::string target_frame,
  TransformLookup transform_lookup)
: target_frame_(std::move(target_frame)),
  transform_lookup_(std::move(transform_lookup))
{
  if (target_frame_.empty()) {
    throw std::invalid_argument("target_frame must not be empty");
  }
  if (!transform_lookup_) {
    throw std::invalid_argument("transform_lookup callback must be set");
  }
}

NormalizationResult StateFrameNormalizer::normalize(
  const std::string & expected_agent_id,
  const swarm_interfaces::msg::AgentState & source) const
{
  const std::string expected_source_frame = expected_agent_id + "/odom";
  const std::string expected_child_frame = expected_agent_id + "/base_link";

  if (source.header.frame_id.empty()) {
    return reject(NormalizationStatus::kEmptySourceFrame, "source frame is empty");
  }
  if (source.header.frame_id != expected_source_frame) {
    return reject(
      NormalizationStatus::kUnexpectedSourceFrame,
      "expected '" + expected_source_frame + "' but received '" +
      source.header.frame_id + "'");
  }
  if (source.child_frame_id.empty()) {
    return reject(NormalizationStatus::kEmptyChildFrame, "child frame is empty");
  }
  if (source.child_frame_id != expected_child_frame) {
    return reject(
      NormalizationStatus::kUnexpectedChildFrame,
      "expected '" + expected_child_frame + "' but received '" +
      source.child_frame_id + "'");
  }
  if (!valid_source_timestamp(source.header.stamp)) {
    return reject(
      NormalizationStatus::kInvalidSourceTimestamp,
      "timestamp is invalid or zero/latest");
  }
  if (!valid_quaternion(source.pose.orientation)) {
    return reject(
      NormalizationStatus::kInvalidPoseQuaternion,
      "source pose quaternion is non-finite or has near-zero norm");
  }

  const TransformLookupResult lookup = transform_lookup_(
    target_frame_, source.header.frame_id, source.header.stamp);
  if (!lookup.transform.has_value()) {
    return reject(
      NormalizationStatus::kTransformUnavailable,
      lookup.detail.empty() ? "transform is unavailable" : lookup.detail);
  }

  geometry_msgs::msg::TransformStamped target_from_source = *lookup.transform;
  if (target_from_source.header.frame_id != target_frame_ ||
    target_from_source.child_frame_id != source.header.frame_id)
  {
    return reject(
      NormalizationStatus::kTransformFrameMismatch,
      "lookup returned a transform with mismatched target/source frames");
  }
  if (!valid_quaternion(target_from_source.transform.rotation)) {
    return reject(
      NormalizationStatus::kInvalidTransformQuaternion,
      "transform quaternion is non-finite or has near-zero norm");
  }

  try {
    tf2::Quaternion source_orientation;
    tf2::fromMsg(source.pose.orientation, source_orientation);
    source_orientation.normalize();

    tf2::Quaternion target_from_source_rotation;
    tf2::fromMsg(target_from_source.transform.rotation, target_from_source_rotation);
    target_from_source_rotation.normalize();
    target_from_source.transform.rotation = tf2::toMsg(target_from_source_rotation);

    geometry_msgs::msg::PoseStamped source_pose;
    source_pose.header = source.header;
    source_pose.pose = source.pose;
    source_pose.pose.orientation = tf2::toMsg(source_orientation);

    geometry_msgs::msg::PoseStamped normalized_pose;
    tf2::doTransform(source_pose, normalized_pose, target_from_source);

    tf2::Quaternion normalized_orientation;
    tf2::fromMsg(normalized_pose.pose.orientation, normalized_orientation);
    normalized_orientation.normalize();
    normalized_pose.pose.orientation = tf2::toMsg(normalized_orientation);

    auto normalized = source;
    normalized.header.frame_id = target_frame_;
    normalized.header.stamp = source.header.stamp;
    normalized.pose = normalized_pose.pose;

    // Pose receives rotation and translation. Linear and angular velocity are
    // free vectors expressed at the agent origin, so only frame rotation is
    // applied; translating a vector would be physically incorrect here.
    normalized.velocity.linear = rotate_vector(
      source.velocity.linear, target_from_source_rotation);
    normalized.velocity.angular = rotate_vector(
      source.velocity.angular, target_from_source_rotation);

    return {NormalizationStatus::kSuccess, std::move(normalized), {}};
  } catch (const std::exception & exception) {
    return reject(NormalizationStatus::kTransformApplicationFailed, exception.what());
  }
}

const char * to_string(const NormalizationStatus status) noexcept
{
  switch (status) {
    case NormalizationStatus::kSuccess:
      return "success";
    case NormalizationStatus::kEmptySourceFrame:
      return "empty_source_frame";
    case NormalizationStatus::kUnexpectedSourceFrame:
      return "unexpected_source_frame";
    case NormalizationStatus::kEmptyChildFrame:
      return "empty_child_frame";
    case NormalizationStatus::kUnexpectedChildFrame:
      return "unexpected_child_frame";
    case NormalizationStatus::kInvalidSourceTimestamp:
      return "invalid_source_timestamp";
    case NormalizationStatus::kInvalidPoseQuaternion:
      return "invalid_pose_quaternion";
    case NormalizationStatus::kTransformUnavailable:
      return "transform_unavailable";
    case NormalizationStatus::kTransformFrameMismatch:
      return "transform_frame_mismatch";
    case NormalizationStatus::kInvalidTransformQuaternion:
      return "invalid_transform_quaternion";
    case NormalizationStatus::kTransformApplicationFailed:
      return "transform_application_failed";
  }
  return "unknown";
}

}  // namespace swarm_coordinator
