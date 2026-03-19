#include "canopen_hw/realtime_loop.hpp"

#include <atomic>

#include <gtest/gtest.h>

namespace canopen_hw {
namespace {

TEST(RealtimeLoop, TickCalledExpectedTimes) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  int count = 0;
  loop.Run([&]() -> bool {
    return ++count < 10;
  });

  EXPECT_EQ(count, 10);
}

TEST(RealtimeLoop, TickReturnFalseStopsLoop) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  loop.Run([&]() -> bool { return false; });

  EXPECT_EQ(loop.stats().iterations, 1u);
}

TEST(RealtimeLoop, StatsReportReasonableValues) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  int count = 0;
  loop.Run([&]() -> bool { return ++count < 5; });

  auto s = loop.stats();
  EXPECT_EQ(s.iterations, 5u);
  EXPECT_GE(s.avg_jitter_us, 0);
  EXPECT_GE(s.max_jitter_us, 0);
}

TEST(RealtimeLoop, NullTickReturnsImmediately) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  loop.Run(nullptr);

  EXPECT_EQ(loop.stats().iterations, 0u);
}

}  // namespace
}  // namespace canopen_hw
