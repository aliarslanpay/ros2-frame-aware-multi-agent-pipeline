#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "swarm_coordinator/frame_aware_state_processor.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"

namespace swarm_coordinator
{
namespace
{

using namespace std::chrono_literals;

[[nodiscard]] swarm_interfaces::msg::AgentState make_state(
  const std::uint64_t sequence,
  const std::int32_t stamp_seconds)
{
  swarm_interfaces::msg::AgentState state;
  state.header.stamp.sec = stamp_seconds;
  state.header.frame_id = "agent_1/odom";
  state.agent_id = "agent_1";
  state.child_frame_id = "agent_1/base_link";
  state.sequence = sequence;
  state.pose.orientation.w = 1.0;
  state.health = swarm_interfaces::msg::AgentState::HEALTH_NOMINAL;
  return state;
}

[[nodiscard]] geometry_msgs::msg::TransformStamped identity_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "agent_1/odom";
  transform.transform.rotation.w = 1.0;
  return transform;
}

TEST(FrameAwareStateProcessor, StoresOnlyNormalizedTargetFrameState)
{
  FrameAwareStateProcessor processor{
    "map",
    [](const std::string &, const std::string &, const auto &)
    {
      return TransformLookupResult{identity_transform(), {}};
    }};

  const auto result = processor.process(
    "agent_1", make_state(1, 10), SteadyTimePoint{});

  ASSERT_TRUE(result.accepted());
  const auto snapshots = processor.snapshot(
    std::vector<std::string>{"agent_1"}, SteadyTimePoint{}, 1s);
  ASSERT_EQ(snapshots.size(), 1U);
  ASSERT_TRUE(snapshots.front().state);
  EXPECT_EQ(snapshots.front().state->message.header.frame_id, "map");
  EXPECT_EQ(snapshots.front().state->message.sequence, 1U);
}

TEST(FrameAwareStateProcessor, TransformRejectionCannotRefreshFreshness)
{
  const auto transform_available = std::make_shared<bool>(true);
  FrameAwareStateProcessor processor{
    "map",
    [transform_available](const std::string &, const std::string &, const auto &)
    {
      if (!*transform_available) {
        return TransformLookupResult{std::nullopt, "delayed transform"};
      }
      return TransformLookupResult{identity_transform(), {}};
    }};

  ASSERT_TRUE(processor.process(
    "agent_1", make_state(1, 10), SteadyTimePoint{}).accepted());

  *transform_available = false;
  const auto rejected = processor.process(
    "agent_1", make_state(2, 11), SteadyTimePoint{} + 900ms);
  EXPECT_FALSE(rejected.accepted());
  EXPECT_EQ(
    rejected.normalization.status,
    NormalizationStatus::kTransformUnavailable);
  EXPECT_FALSE(rejected.update.has_value());

  const auto snapshots = processor.snapshot(
    std::vector<std::string>{"agent_1"}, SteadyTimePoint{} + 1501ms, 1500ms);
  ASSERT_EQ(snapshots.size(), 1U);
  ASSERT_TRUE(snapshots.front().state);
  EXPECT_EQ(snapshots.front().state->message.sequence, 1U);
  EXPECT_EQ(snapshots.front().freshness, Freshness::kStale);
  ASSERT_TRUE(snapshots.front().age.has_value());
  EXPECT_EQ(*snapshots.front().age, 1501ms);
}

}  // namespace
}  // namespace swarm_coordinator
