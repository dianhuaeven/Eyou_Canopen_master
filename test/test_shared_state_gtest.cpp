#include <gtest/gtest.h>

#include "canopen_hw/shared_state.hpp"

TEST(SharedStateGTest, UpdateAndSnapshot) {
  canopen_hw::SharedState shared(1);

  canopen_hw::AxisFeedback fb;
  fb.actual_position = 123;
  fb.actual_velocity = -7;
  fb.is_operational = true;

  canopen_hw::AxisCommand cmd;
  cmd.target_position = 456;

  shared.UpdateFeedback(0, fb);
  shared.UpdateCommand(0, cmd);
  shared.RecomputeAllOperational();

  const auto snap = shared.Snapshot();
  EXPECT_EQ(snap.feedback[0].actual_position, 123);
  EXPECT_EQ(snap.feedback[0].actual_velocity, -7);
  EXPECT_TRUE(snap.feedback[0].is_operational);
  EXPECT_EQ(snap.commands[0].target_position, 456);
  EXPECT_TRUE(snap.all_operational);
}

TEST(SharedStateGTest, OutOfRangeIgnored) {
  canopen_hw::SharedState shared(6);
  canopen_hw::AxisCommand cmd;
  cmd.target_position = 777;
  shared.UpdateCommand(0, cmd);

  cmd.target_position = 999;
  shared.UpdateCommand(99, cmd);

  const auto snap = shared.Snapshot();
  EXPECT_EQ(snap.commands[0].target_position, 777);
}

TEST(SharedStateGTest, RecomputeAllOperationalTrueWhenAllOperationalAndNoFault) {
  canopen_hw::SharedState shared(6);

  for (std::size_t i = 0; i < shared.axis_count(); ++i) {
    canopen_hw::AxisFeedback fb;
    fb.is_operational = true;
    fb.is_fault = false;
    shared.UpdateFeedback(i, fb);
  }

  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  EXPECT_TRUE(snap.all_operational);
}

TEST(SharedStateGTest, RecomputeAllOperationalFalseWhenAnyAxisFault) {
  canopen_hw::SharedState shared(6);

  for (std::size_t i = 0; i < shared.axis_count(); ++i) {
    canopen_hw::AxisFeedback fb;
    fb.is_operational = true;
    fb.is_fault = false;
    shared.UpdateFeedback(i, fb);
  }

  canopen_hw::AxisFeedback fault_fb;
  fault_fb.is_operational = true;
  fault_fb.is_fault = true;
  fault_fb.heartbeat_lost = true;
  shared.UpdateFeedback(3, fault_fb);

  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  EXPECT_FALSE(snap.all_operational);
  EXPECT_TRUE(snap.feedback[3].heartbeat_lost);
}

TEST(SharedStateGTest, RecomputeUsesConfiguredAxisCount) {
  canopen_hw::SharedState shared(1);

  canopen_hw::AxisFeedback axis0;
  axis0.is_operational = true;
  axis0.is_fault = false;
  shared.UpdateFeedback(0, axis0);

  // 其余轴不存在，不应影响只配置 1 轴时的汇总结果。
  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  EXPECT_TRUE(snap.all_operational);
}
