#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "swarm_coordinator/agent_state_cache.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"

namespace swarm_coordinator
{
namespace
{

using namespace std::chrono_literals;

swarm_interfaces::msg::AgentState make_state(
  const std::string & agent_id,
  const std::uint64_t sequence,
  const std::int32_t seconds,
  const std::uint32_t nanoseconds = 0)
{
  swarm_interfaces::msg::AgentState message;
  message.header.stamp.sec = seconds;
  message.header.stamp.nanosec = nanoseconds;
  message.header.frame_id = "map";
  message.agent_id = agent_id;
  message.sequence = sequence;
  message.pose.position.x = static_cast<double>(sequence);
  message.pose.orientation.w = 1.0;
  message.health = swarm_interfaces::msg::AgentState::HEALTH_NOMINAL;
  return message;
}

AgentSnapshot snapshot_for(
  const AgentStateCache & cache,
  const std::string & agent_id,
  const SteadyTimePoint now,
  const std::chrono::milliseconds stale_timeout)
{
  auto snapshots = cache.snapshot({agent_id}, now, stale_timeout);
  if (snapshots.size() != 1U) {
    throw std::logic_error("single-agent snapshot returned an unexpected size");
  }
  return std::move(snapshots.front());
}

TEST(AgentStateCache, AcceptsAndStoresFirstState)
{
  AgentStateCache cache;
  const SteadyTimePoint received_at{};

  const auto result = cache.update("agent_1", make_state("agent_1", 0, 10), received_at);
  const auto snapshot = snapshot_for(cache, "agent_1", received_at, 1s);

  EXPECT_EQ(result.status, UpdateStatus::kAcceptedFirst);
  ASSERT_TRUE(snapshot.state);
  EXPECT_EQ(snapshot.state->message.sequence, 0U);
  EXPECT_EQ(snapshot.freshness, Freshness::kFresh);
}

TEST(AgentStateCache, DetectsSequenceGaps)
{
  AgentStateCache cache;
  const SteadyTimePoint start{};
  ASSERT_TRUE(cache.update("agent_1", make_state("agent_1", 10, 10), start).accepted());

  const auto result = cache.update("agent_1", make_state("agent_1", 14, 11), start + 1s);

  EXPECT_TRUE(result.accepted());
  EXPECT_EQ(result.missing_samples, 3U);
}

TEST(AgentStateCache, RejectsIdentityMismatchWithoutUpdatingCache)
{
  AgentStateCache cache;

  const auto result = cache.update(
    "agent_1", make_state("wrong_agent", 0, 10), SteadyTimePoint{});
  const auto snapshot = snapshot_for(cache, "agent_1", SteadyTimePoint{}, 1s);

  EXPECT_EQ(result.status, UpdateStatus::kRejectedIdentityMismatch);
  EXPECT_FALSE(snapshot.state);
  EXPECT_EQ(snapshot.freshness, Freshness::kNeverSeen);
}

TEST(AgentStateCache, RejectsDuplicateAndOutOfOrderTimestamp)
{
  AgentStateCache cache;
  const SteadyTimePoint start{};
  ASSERT_TRUE(cache.update("agent_1", make_state("agent_1", 10, 10), start).accepted());

  EXPECT_EQ(
    cache.update("agent_1", make_state("agent_1", 11, 10), start + 1s).status,
    UpdateStatus::kRejectedTimestamp);
  EXPECT_EQ(
    cache.update("agent_1", make_state("agent_1", 12, 9), start + 2s).status,
    UpdateStatus::kRejectedTimestamp);
}

TEST(AgentStateCache, RejectsNonIncreasingSequence)
{
  AgentStateCache cache;
  const SteadyTimePoint start{};
  ASSERT_TRUE(cache.update("agent_1", make_state("agent_1", 10, 10), start).accepted());

  EXPECT_EQ(
    cache.update("agent_1", make_state("agent_1", 10, 11), start + 1s).status,
    UpdateStatus::kRejectedSequence);
  EXPECT_EQ(
    cache.update("agent_1", make_state("agent_1", 9, 12), start + 2s).status,
    UpdateStatus::kRejectedSequence);
}

TEST(AgentStateCache, RejectsInvalidNanosecondField)
{
  AgentStateCache cache;

  const auto result = cache.update(
    "agent_1", make_state("agent_1", 0, 10, 1'000'000'000U), SteadyTimePoint{});

  EXPECT_EQ(result.status, UpdateStatus::kRejectedInvalidTimestamp);
}

TEST(AgentStateCache, ReportsNeverSeenFreshAndStaleFromOneSnapshotApi)
{
  AgentStateCache cache;
  const SteadyTimePoint start{};

  EXPECT_EQ(
    snapshot_for(cache, "agent_1", start, 1000ms).freshness,
    Freshness::kNeverSeen);

  ASSERT_TRUE(cache.update("agent_1", make_state("agent_1", 0, 10), start).accepted());
  EXPECT_EQ(
    snapshot_for(cache, "agent_1", start + 1000ms, 1000ms).freshness,
    Freshness::kFresh);
  EXPECT_EQ(
    snapshot_for(cache, "agent_1", start + 1001ms, 1000ms).freshness,
    Freshness::kStale);
}

TEST(AgentStateCache, RejectedStateDoesNotRefreshArrivalTime)
{
  AgentStateCache cache;
  const SteadyTimePoint start{};
  ASSERT_TRUE(cache.update("agent_1", make_state("agent_1", 10, 10), start).accepted());

  EXPECT_FALSE(
    cache.update("agent_1", make_state("agent_1", 10, 11), start + 900ms).accepted());

  const auto snapshot = snapshot_for(cache, "agent_1", start + 1001ms, 1000ms);
  EXPECT_EQ(snapshot.freshness, Freshness::kStale);
  ASSERT_TRUE(snapshot.age.has_value());
  EXPECT_EQ(*snapshot.age, 1001ms);
}

TEST(AgentStateCache, ReturnsOneCoherentMultiAgentSnapshot)
{
  AgentStateCache cache;
  const SteadyTimePoint start{};
  ASSERT_TRUE(cache.update("agent_1", make_state("agent_1", 1, 10), start).accepted());
  ASSERT_TRUE(cache.update("agent_2", make_state("agent_2", 2, 20), start + 500ms).accepted());

  const auto snapshots = cache.snapshot(
    {"agent_1", "agent_2", "agent_3"}, start + 2s, 1500ms);

  ASSERT_EQ(snapshots.size(), 3U);
  EXPECT_EQ(snapshots[0].freshness, Freshness::kStale);
  EXPECT_EQ(snapshots[1].freshness, Freshness::kFresh);
  EXPECT_EQ(snapshots[2].freshness, Freshness::kNeverSeen);
  ASSERT_TRUE(snapshots[0].state);
  ASSERT_TRUE(snapshots[1].state);
  EXPECT_EQ(snapshots[0].state->message.sequence, 1U);
  EXPECT_EQ(snapshots[1].state->message.sequence, 2U);
}

TEST(AgentStateCache, ClampsConcurrentFutureReceiptAgeToZero)
{
  AgentStateCache cache;
  const SteadyTimePoint start{};
  ASSERT_TRUE(
    cache.update("agent_1", make_state("agent_1", 1, 10), start + 1ms).accepted());

  const auto snapshot = snapshot_for(cache, "agent_1", start, 1000ms);

  EXPECT_EQ(snapshot.freshness, Freshness::kFresh);
  ASSERT_TRUE(snapshot.age.has_value());
  EXPECT_EQ(*snapshot.age, 0ms);
}

TEST(AgentStateCache, RejectsNegativeStaleTimeout)
{
  const AgentStateCache cache;
  EXPECT_THROW(cache.snapshot({"agent_1"}, SteadyTimePoint{}, -1ms), std::invalid_argument);
}

}  // namespace
}  // namespace swarm_coordinator
