#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "swarm_coordinator/agent_state_cache.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"

namespace swarm_coordinator
{
namespace
{

using namespace std::chrono_literals;

class StartGate final
{
public:
  explicit StartGate(const std::size_t participant_count)
  : participant_count_(participant_count)
  {
  }

  void arrive_and_wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++arrived_;
    if (arrived_ == participant_count_) {
      open_ = true;
      condition_.notify_all();
      return;
    }
    condition_.wait(lock, [this]() { return open_; });
  }

private:
  const std::size_t participant_count_;
  std::size_t arrived_{0};
  bool open_{false};
  std::mutex mutex_;
  std::condition_variable condition_;
};

swarm_interfaces::msg::AgentState make_state(
  const std::string & agent_id,
  const std::uint64_t sequence)
{
  swarm_interfaces::msg::AgentState message;
  message.header.stamp.sec = static_cast<std::int32_t>(sequence + 1U);
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
  const SteadyTimePoint now = SteadyTimePoint{} + 24h)
{
  return cache.snapshot({agent_id}, now, 48h).front();
}

TEST(AgentStateCacheConcurrency, UpdatesDifferentAgentsConcurrently)
{
  constexpr std::size_t kAgentCount = 4;
  constexpr std::uint64_t kIterations = 2000;

  AgentStateCache cache;
  StartGate start_gate{kAgentCount};
  std::atomic<bool> failed{false};
  std::vector<std::thread> workers;
  workers.reserve(kAgentCount);

  for (std::size_t agent_index = 0; agent_index < kAgentCount; ++agent_index) {
    workers.emplace_back([&, agent_index]() {
      const std::string agent_id = "agent_" + std::to_string(agent_index);
      start_gate.arrive_and_wait();
      for (std::uint64_t sequence = 0; sequence < kIterations; ++sequence) {
        if (!cache.update(
            agent_id, make_state(agent_id, sequence),
            SteadyTimePoint{} + std::chrono::nanoseconds{sequence}).accepted())
        {
          failed.store(true, std::memory_order_relaxed);
        }
      }
    });
  }

  for (auto & worker : workers) {
    worker.join();
  }

  EXPECT_FALSE(failed.load(std::memory_order_relaxed));
  for (std::size_t agent_index = 0; agent_index < kAgentCount; ++agent_index) {
    const auto snapshot = snapshot_for(cache, "agent_" + std::to_string(agent_index));
    ASSERT_TRUE(snapshot.state);
    EXPECT_EQ(snapshot.state->message.sequence, kIterations - 1U);
  }
}

TEST(AgentStateCacheConcurrency, SnapshotRemainsConsistentDuringUpdates)
{
  constexpr std::uint64_t kIterations = 5000;
  const std::string agent_id = "agent_1";
  AgentStateCache cache;
  StartGate start_gate{2};
  std::atomic<bool> writer_done{false};
  std::atomic<bool> failed{false};

  std::thread writer([&]() {
    start_gate.arrive_and_wait();
    for (std::uint64_t sequence = 0; sequence < kIterations; ++sequence) {
      if (!cache.update(
          agent_id, make_state(agent_id, sequence),
          SteadyTimePoint{} + std::chrono::nanoseconds{sequence}).accepted())
      {
        failed.store(true, std::memory_order_relaxed);
      }
      if (sequence % 32U == 0U) {
        std::this_thread::yield();
      }
    }
    writer_done.store(true, std::memory_order_release);
  });

  std::thread reader([&]() {
    start_gate.arrive_and_wait();
    std::optional<std::uint64_t> previous_sequence;
    do {
      const auto snapshot = snapshot_for(cache, agent_id);
      if (!snapshot.state) {
        std::this_thread::yield();
        continue;
      }

      const auto sequence = snapshot.state->message.sequence;
      const auto expected_timestamp_ns =
        static_cast<std::int64_t>(sequence + 1U) * 1'000'000'000LL;
      if (snapshot.state->message.pose.position.x != static_cast<double>(sequence) ||
        snapshot.state->source_timestamp_ns != expected_timestamp_ns ||
        (previous_sequence.has_value() && sequence < *previous_sequence))
      {
        failed.store(true, std::memory_order_relaxed);
      }
      previous_sequence = sequence;
    } while (!writer_done.load(std::memory_order_acquire));
  });

  writer.join();
  reader.join();

  EXPECT_FALSE(failed.load(std::memory_order_relaxed));
  const auto final_snapshot = snapshot_for(cache, agent_id);
  ASSERT_TRUE(final_snapshot.state);
  EXPECT_EQ(final_snapshot.state->message.sequence, kIterations - 1U);
}

TEST(AgentStateCacheConcurrency, RejectsConcurrentDuplicateAndOutOfOrderMessages)
{
  constexpr std::size_t kThreadCount = 8;
  constexpr std::size_t kIterations = 500;
  const std::string agent_id = "agent_1";
  AgentStateCache cache;
  ASSERT_TRUE(cache.update(agent_id, make_state(agent_id, 100), SteadyTimePoint{}).accepted());

  StartGate start_gate{kThreadCount};
  std::atomic<std::size_t> rejected{0};
  std::atomic<bool> unexpected_status{false};
  std::vector<std::thread> workers;

  for (std::size_t thread_index = 0; thread_index < kThreadCount; ++thread_index) {
    workers.emplace_back([&, thread_index]() {
      start_gate.arrive_and_wait();
      for (std::size_t iteration = 0; iteration < kIterations; ++iteration) {
        auto message = make_state(agent_id, 100);
        if ((thread_index + iteration) % 2U != 0U) {
          message.sequence = 200;
          message.header.stamp.sec = 100;  // Older than cached stamp 101.
        } else {
          message.header.stamp.sec = 102;  // New stamp, duplicate sequence.
        }

        const auto result = cache.update(agent_id, message, SteadyTimePoint{} + 1h);
        if (result.status != UpdateStatus::kRejectedTimestamp &&
          result.status != UpdateStatus::kRejectedSequence)
        {
          unexpected_status.store(true, std::memory_order_relaxed);
        } else {
          rejected.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }

  for (auto & worker : workers) {
    worker.join();
  }

  EXPECT_FALSE(unexpected_status.load(std::memory_order_relaxed));
  EXPECT_EQ(rejected.load(std::memory_order_relaxed), kThreadCount * kIterations);
  const auto snapshot = snapshot_for(cache, agent_id);
  ASSERT_TRUE(snapshot.state);
  EXPECT_EQ(snapshot.state->message.sequence, 100U);
}

TEST(AgentStateCacheConcurrency, RejectedTrafficCannotRefreshFreshness)
{
  constexpr std::size_t kThreadCount = 6;
  const std::string agent_id = "agent_1";
  const SteadyTimePoint start{};
  AgentStateCache cache;
  ASSERT_TRUE(cache.update(agent_id, make_state(agent_id, 10), start).accepted());

  StartGate start_gate{kThreadCount};
  std::vector<std::thread> workers;
  for (std::size_t index = 0; index < kThreadCount; ++index) {
    workers.emplace_back([&, index]() {
      start_gate.arrive_and_wait();
      for (std::size_t iteration = 0; iteration < 500; ++iteration) {
        auto duplicate = make_state(agent_id, 10);
        duplicate.header.stamp.sec = 1000 + static_cast<std::int32_t>(index + iteration);
        static_cast<void>(cache.update(agent_id, duplicate, start + 900ms));
      }
    });
  }
  for (auto & worker : workers) {
    worker.join();
  }

  const auto snapshot = cache.snapshot({agent_id}, start + 1001ms, 1000ms).front();
  EXPECT_EQ(snapshot.freshness, Freshness::kStale);
  ASSERT_TRUE(snapshot.age.has_value());
  EXPECT_EQ(*snapshot.age, 1001ms);
}

TEST(AgentStateCacheConcurrency, PreservesInvariantsUnderMixedStress)
{
  constexpr std::size_t kAgentCount = 4;
  constexpr std::size_t kWriterCount = 8;
  constexpr std::uint64_t kIterations = 2000;
  const std::vector<std::string> agent_ids{
    "agent_0", "agent_1", "agent_2", "agent_3"};

  AgentStateCache cache;
  StartGate start_gate{kWriterCount + 1U};
  std::atomic<std::size_t> writers_done{0};
  std::atomic<bool> failed{false};
  std::vector<std::thread> writers;

  for (std::size_t writer_index = 0; writer_index < kWriterCount; ++writer_index) {
    writers.emplace_back([&, writer_index]() {
      start_gate.arrive_and_wait();
      const auto & agent_id = agent_ids[writer_index % kAgentCount];
      for (std::uint64_t iteration = 0; iteration < kIterations; ++iteration) {
        // Two writers target each agent. Their monotonic subsequences interleave
        // in a scheduler-dependent order, intentionally exercising rejection.
        const std::uint64_t sequence = iteration * 2U + writer_index / kAgentCount;
        static_cast<void>(cache.update(
          agent_id, make_state(agent_id, sequence),
          SteadyTimePoint{} + std::chrono::nanoseconds{sequence}));
      }
      writers_done.fetch_add(1, std::memory_order_release);
    });
  }

  std::thread reader([&]() {
    start_gate.arrive_and_wait();
    std::vector<std::optional<std::uint64_t>> previous(kAgentCount);
    while (writers_done.load(std::memory_order_acquire) < kWriterCount) {
      const auto snapshots = cache.snapshot(agent_ids, SteadyTimePoint{} + 24h, 48h);
      if (snapshots.size() != kAgentCount) {
        failed.store(true, std::memory_order_relaxed);
        continue;
      }
      for (std::size_t index = 0; index < snapshots.size(); ++index) {
        if (!snapshots[index].state) {
          continue;
        }
        const auto sequence = snapshots[index].state->message.sequence;
        if (snapshots[index].state->message.agent_id != agent_ids[index] ||
          snapshots[index].state->message.pose.position.x != static_cast<double>(sequence) ||
          (previous[index].has_value() && sequence < *previous[index]))
        {
          failed.store(true, std::memory_order_relaxed);
        }
        previous[index] = sequence;
      }
      std::this_thread::yield();
    }
  });

  for (auto & writer : writers) {
    writer.join();
  }
  reader.join();

  constexpr std::uint64_t kFinalSequence = 1'000'000;
  for (const auto & agent_id : agent_ids) {
    ASSERT_TRUE(
      cache.update(agent_id, make_state(agent_id, kFinalSequence),
        SteadyTimePoint{} + 25h).accepted());
  }

  EXPECT_FALSE(failed.load(std::memory_order_relaxed));
  const auto final_snapshots = cache.snapshot(agent_ids, SteadyTimePoint{} + 26h, 48h);
  ASSERT_EQ(final_snapshots.size(), kAgentCount);
  for (const auto & snapshot : final_snapshots) {
    ASSERT_TRUE(snapshot.state);
    EXPECT_EQ(snapshot.state->message.sequence, kFinalSequence);
    EXPECT_EQ(snapshot.state->message.agent_id, snapshot.agent_id);
  }
}

}  // namespace
}  // namespace swarm_coordinator
