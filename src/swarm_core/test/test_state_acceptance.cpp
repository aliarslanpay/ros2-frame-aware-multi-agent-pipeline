#include <cstdint>
#include <limits>
#include <optional>

#include "gtest/gtest.h"
#include "swarm_core/state_acceptance.hpp"

namespace swarm_core
{
namespace
{

TEST(StateAcceptance, AcceptsFirstState)
{
  EXPECT_TRUE(should_accept(0, std::nullopt));
  EXPECT_TRUE(should_accept(std::numeric_limits<std::int64_t>::min(), std::nullopt));
}

TEST(StateAcceptance, AcceptsStrictlyNewerTimestamp)
{
  EXPECT_TRUE(should_accept(101, 100));
  EXPECT_TRUE(should_accept(
    std::numeric_limits<std::int64_t>::max(),
    std::numeric_limits<std::int64_t>::max() - 1));
}

TEST(StateAcceptance, RejectsDuplicateTimestamp)
{
  EXPECT_FALSE(should_accept(100, 100));
}

TEST(StateAcceptance, RejectsOutOfOrderTimestamp)
{
  EXPECT_FALSE(should_accept(99, 100));
  EXPECT_FALSE(should_accept(
    std::numeric_limits<std::int64_t>::min(),
    std::numeric_limits<std::int64_t>::max()));
}

}  // namespace
}  // namespace swarm_core

