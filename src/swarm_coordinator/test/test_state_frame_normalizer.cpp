#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "swarm_coordinator/state_frame_normalizer.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace swarm_coordinator
{
namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kHalfPi = 1.5707963267948966;

[[nodiscard]] geometry_msgs::msg::Quaternion yaw_quaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

[[nodiscard]] swarm_interfaces::msg::AgentState make_state()
{
  swarm_interfaces::msg::AgentState state;
  state.header.stamp.sec = 10;
  state.header.stamp.nanosec = 100U;
  state.header.frame_id = "agent_1/odom";
  state.agent_id = "agent_1";
  state.child_frame_id = "agent_1/base_link";
  state.sequence = 7;
  state.pose.orientation = yaw_quaternion(0.0);
  state.health = swarm_interfaces::msg::AgentState::HEALTH_NOMINAL;
  return state;
}

[[nodiscard]] geometry_msgs::msg::TransformStamped make_transform(
  const double x,
  const double y,
  const double z,
  const double yaw)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "agent_1/odom";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  transform.transform.rotation = yaw_quaternion(yaw);
  return transform;
}

[[nodiscard]] StateFrameNormalizer normalizer_with(
  const geometry_msgs::msg::TransformStamped & transform)
{
  return StateFrameNormalizer{
    "map",
    [transform](const std::string &, const std::string &, const auto &)
    {
      return TransformLookupResult{transform, {}};
    }};
}

TEST(StateFrameNormalizer, AppliesTranslationAndPreservesSourceTimestamp)
{
  auto state = make_state();
  state.pose.position.x = 1.0;
  state.pose.position.y = 2.0;
  state.pose.position.z = 3.0;
  state.velocity.linear.x = 4.0;

  const auto normalizer = normalizer_with(make_transform(10.0, -2.0, 1.0, 0.0));
  const auto result = normalizer.normalize("agent_1", state);

  ASSERT_TRUE(result.success());
  EXPECT_EQ(result.state->header.frame_id, "map");
  EXPECT_EQ(result.state->header.stamp.sec, state.header.stamp.sec);
  EXPECT_EQ(result.state->header.stamp.nanosec, state.header.stamp.nanosec);
  EXPECT_EQ(result.state->child_frame_id, "agent_1/base_link");
  EXPECT_NEAR(result.state->pose.position.x, 11.0, kTolerance);
  EXPECT_NEAR(result.state->pose.position.y, 0.0, kTolerance);
  EXPECT_NEAR(result.state->pose.position.z, 4.0, kTolerance);
  EXPECT_NEAR(result.state->velocity.linear.x, 4.0, kTolerance);
}

TEST(StateFrameNormalizer, RotatesPoseAndVelocityWithoutTranslatingVectors)
{
  auto state = make_state();
  state.pose.position.x = 1.0;
  state.velocity.linear.x = 2.0;
  state.velocity.linear.z = 3.0;
  state.velocity.angular.x = 1.0;
  state.velocity.angular.z = 4.0;

  const auto normalizer = normalizer_with(make_transform(10.0, 20.0, 0.0, kHalfPi));
  const auto result = normalizer.normalize("agent_1", state);

  ASSERT_TRUE(result.success());
  EXPECT_NEAR(result.state->pose.position.x, 10.0, kTolerance);
  EXPECT_NEAR(result.state->pose.position.y, 21.0, kTolerance);
  EXPECT_NEAR(result.state->velocity.linear.x, 0.0, kTolerance);
  EXPECT_NEAR(result.state->velocity.linear.y, 2.0, kTolerance);
  EXPECT_NEAR(result.state->velocity.linear.z, 3.0, kTolerance);
  EXPECT_NEAR(result.state->velocity.angular.x, 0.0, kTolerance);
  EXPECT_NEAR(result.state->velocity.angular.y, 1.0, kTolerance);
  EXPECT_NEAR(result.state->velocity.angular.z, 4.0, kTolerance);
  EXPECT_NEAR(result.state->pose.orientation.z, std::sqrt(0.5), kTolerance);
  EXPECT_NEAR(result.state->pose.orientation.w, std::sqrt(0.5), kTolerance);
}

TEST(StateFrameNormalizer, RejectsUnavailableTransform)
{
  const StateFrameNormalizer normalizer{
    "map",
    [](const std::string &, const std::string &, const auto &)
    {
      return TransformLookupResult{std::nullopt, "missing map to odom transform"};
    }};

  const auto result = normalizer.normalize("agent_1", make_state());

  EXPECT_EQ(result.status, NormalizationStatus::kTransformUnavailable);
  EXPECT_FALSE(result.state.has_value());
  EXPECT_EQ(result.detail, "missing map to odom transform");
}

TEST(StateFrameNormalizer, RejectsUnexpectedSourceFrameWithoutLookup)
{
  bool lookup_called = false;
  const StateFrameNormalizer normalizer{
    "map",
    [&lookup_called](const std::string &, const std::string &, const auto &)
    {
      lookup_called = true;
      return TransformLookupResult{make_transform(0.0, 0.0, 0.0, 0.0), {}};
    }};
  auto state = make_state();
  state.header.frame_id = "wrong/odom";

  const auto result = normalizer.normalize("agent_1", state);

  EXPECT_EQ(result.status, NormalizationStatus::kUnexpectedSourceFrame);
  EXPECT_FALSE(lookup_called);
}

TEST(StateFrameNormalizer, RejectsUnexpectedChildFrame)
{
  const auto normalizer = normalizer_with(make_transform(0.0, 0.0, 0.0, 0.0));
  auto state = make_state();
  state.child_frame_id = "agent_2/base_link";

  const auto result = normalizer.normalize("agent_1", state);

  EXPECT_EQ(result.status, NormalizationStatus::kUnexpectedChildFrame);
  EXPECT_FALSE(result.state.has_value());
}

TEST(StateFrameNormalizer, RejectsZeroTimestampInsteadOfUsingLatestTransform)
{
  const auto normalizer = normalizer_with(make_transform(0.0, 0.0, 0.0, 0.0));
  auto state = make_state();
  state.header.stamp.sec = 0;
  state.header.stamp.nanosec = 0U;

  const auto result = normalizer.normalize("agent_1", state);

  EXPECT_EQ(result.status, NormalizationStatus::kInvalidSourceTimestamp);
  EXPECT_FALSE(result.state.has_value());
}

TEST(StateFrameNormalizer, RejectsInvalidPoseQuaternion)
{
  const auto normalizer = normalizer_with(make_transform(0.0, 0.0, 0.0, 0.0));
  auto state = make_state();
  state.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();

  const auto result = normalizer.normalize("agent_1", state);

  EXPECT_EQ(result.status, NormalizationStatus::kInvalidPoseQuaternion);
  EXPECT_FALSE(result.state.has_value());
}

}  // namespace
}  // namespace swarm_coordinator
