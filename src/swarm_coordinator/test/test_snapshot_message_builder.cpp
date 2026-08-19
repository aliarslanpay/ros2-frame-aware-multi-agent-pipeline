#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "swarm_coordinator/snapshot_message_builder.hpp"
#include "swarm_interfaces/msg/agent_snapshot.hpp"

namespace swarm_coordinator
{
namespace
{

using namespace std::chrono_literals;

[[nodiscard]] CachedAgentSnapshot make_cached_snapshot(
  const std::string & agent_id,
  const Freshness freshness,
  const std::chrono::milliseconds age,
  const std::uint64_t state_sequence = 7,
  const std::int32_t source_seconds = 42,
  const std::uint32_t source_nanoseconds = 123'000'000U)
{
  swarm_interfaces::msg::AgentState state;
  state.header.stamp.sec = source_seconds;
  state.header.stamp.nanosec = source_nanoseconds;
  state.header.frame_id = "map";
  state.agent_id = agent_id;
  state.child_frame_id = agent_id + "/base_link";
  state.sequence = state_sequence;
  state.pose.position.x = 12.5;
  state.pose.orientation.w = 1.0;
  state.health = swarm_interfaces::msg::AgentState::HEALTH_NOMINAL;

  auto cached_state = std::make_shared<const CachedAgentState>(
    CachedAgentState{state, SteadyTimePoint{}, 0});
  return CachedAgentSnapshot{agent_id, freshness, std::move(cached_state), age};
}

[[nodiscard]] builtin_interfaces::msg::Time creation_time()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100;
  stamp.nanosec = 456'000'000U;
  return stamp;
}

TEST(SnapshotMessageBuilder, NeverSeenUsesExplicitAbsentStateContract)
{
  const std::vector<CachedAgentSnapshot> snapshots{
    CachedAgentSnapshot{"agent_1", Freshness::kNeverSeen, nullptr, std::nullopt}};

  const auto message = make_swarm_snapshot_message(
    snapshots, creation_time(), "map", 10);

  ASSERT_EQ(message.agents.size(), 1U);
  const auto & agent = message.agents.front();
  EXPECT_EQ(agent.freshness, swarm_interfaces::msg::AgentSnapshot::NEVER_SEEN);
  EXPECT_FALSE(agent.has_state);
  EXPECT_EQ(agent.age.sec, 0);
  EXPECT_EQ(agent.age.nanosec, 0U);
  EXPECT_TRUE(agent.state.agent_id.empty());
  EXPECT_TRUE(agent.state.header.frame_id.empty());
  EXPECT_EQ(agent.state.sequence, 0U);
}

TEST(SnapshotMessageBuilder, FreshCopiesAcceptedStateAndReceiptAge)
{
  const auto message = make_swarm_snapshot_message(
    {make_cached_snapshot("agent_1", Freshness::kFresh, 1234ms)},
    creation_time(), "map", 11);

  ASSERT_EQ(message.agents.size(), 1U);
  const auto & agent = message.agents.front();
  EXPECT_EQ(agent.freshness, swarm_interfaces::msg::AgentSnapshot::FRESH);
  EXPECT_TRUE(agent.has_state);
  EXPECT_EQ(agent.state.agent_id, "agent_1");
  EXPECT_EQ(agent.state.sequence, 7U);
  EXPECT_DOUBLE_EQ(agent.state.pose.position.x, 12.5);
  EXPECT_EQ(agent.age.sec, 1);
  EXPECT_EQ(agent.age.nanosec, 234'000'000U);
}

TEST(SnapshotMessageBuilder, StaleUsesStaleConstantAndReceiptAge)
{
  const auto message = make_swarm_snapshot_message(
    {make_cached_snapshot("agent_2", Freshness::kStale, 2500ms)},
    creation_time(), "map", 12);

  ASSERT_EQ(message.agents.size(), 1U);
  const auto & agent = message.agents.front();
  EXPECT_EQ(agent.freshness, swarm_interfaces::msg::AgentSnapshot::STALE);
  EXPECT_TRUE(agent.has_state);
  EXPECT_EQ(agent.age.sec, 2);
  EXPECT_EQ(agent.age.nanosec, 500'000'000U);
}

TEST(SnapshotMessageBuilder, PreservesConfiguredAgentOrder)
{
  const std::vector<CachedAgentSnapshot> snapshots{
    make_cached_snapshot("agent_3", Freshness::kFresh, 1ms),
    CachedAgentSnapshot{"agent_1", Freshness::kNeverSeen, nullptr, std::nullopt},
    make_cached_snapshot("agent_2", Freshness::kStale, 2s)};

  const auto message = make_swarm_snapshot_message(
    snapshots, creation_time(), "map", 13);

  ASSERT_EQ(message.agents.size(), 3U);
  EXPECT_EQ(message.agents[0].agent_id, "agent_3");
  EXPECT_EQ(message.agents[1].agent_id, "agent_1");
  EXPECT_EQ(message.agents[2].agent_id, "agent_2");
}

TEST(SnapshotMessageBuilder, CopiesCreationMetadataAndSnapshotSequence)
{
  const auto message = make_swarm_snapshot_message(
    {}, creation_time(), "world", 9876);

  EXPECT_EQ(message.header.stamp.sec, 100);
  EXPECT_EQ(message.header.stamp.nanosec, 456'000'000U);
  EXPECT_EQ(message.header.frame_id, "world");
  EXPECT_EQ(message.sequence, 9876U);
}

TEST(SnapshotMessageBuilder, PreservesNestedSourceTimestampAndTargetFrame)
{
  const auto message = make_swarm_snapshot_message(
    {make_cached_snapshot(
      "agent_1", Freshness::kFresh, 10ms, 5, 77, 88U)},
    creation_time(), "map", 14);

  ASSERT_EQ(message.agents.size(), 1U);
  const auto & state = message.agents.front().state;
  EXPECT_EQ(state.header.stamp.sec, 77);
  EXPECT_EQ(state.header.stamp.nanosec, 88U);
  EXPECT_EQ(state.header.frame_id, "map");
  EXPECT_NE(state.header.stamp.sec, message.header.stamp.sec);
}

TEST(SnapshotMessageBuilder, ClampsNegativeAgeToZero)
{
  const auto message = make_swarm_snapshot_message(
    {make_cached_snapshot("agent_1", Freshness::kFresh, -1ms)},
    creation_time(), "map", 15);

  ASSERT_EQ(message.agents.size(), 1U);
  EXPECT_EQ(message.agents.front().age.sec, 0);
  EXPECT_EQ(message.agents.front().age.nanosec, 0U);
}

TEST(SnapshotMessageBuilder, SaturatesAgeAboveDurationRange)
{
  using MillisecondsRep = std::chrono::milliseconds::rep;
  const auto message = make_swarm_snapshot_message(
    {make_cached_snapshot(
      "agent_1", Freshness::kStale,
      std::chrono::milliseconds{std::numeric_limits<MillisecondsRep>::max()})},
    creation_time(), "map", 16);

  ASSERT_EQ(message.agents.size(), 1U);
  EXPECT_EQ(
    message.agents.front().age.sec,
    std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(message.agents.front().age.nanosec, 999'999'999U);
}

}  // namespace
}  // namespace swarm_coordinator
