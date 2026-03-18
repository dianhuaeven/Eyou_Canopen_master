#include <gtest/gtest.h>

#include "canopen_hw/shared_state.hpp"

TEST(SharedStateGTest, UpdateAndSnapshot) {
  canopen_hw::SharedState shared;

  canopen_hw::AxisFeedback fb;
  fb.actual_position = 123;
  fb.actual_velocity = -7;
  fb.is_operational = true;

  canopen_hw::AxisCommand cmd;
  cmd.target_position = 456;

  shared.UpdateFeedback(0, fb);
  shared.UpdateCommand(0, cmd);
  shared.SetAllOperational(true);

  const auto snap = shared.Snapshot();
  EXPECT_EQ(snap.feedback[0].actual_position, 123);
  EXPECT_EQ(snap.feedback[0].actual_velocity, -7);
  EXPECT_TRUE(snap.feedback[0].is_operational);
  EXPECT_EQ(snap.commands[0].target_position, 456);
  EXPECT_TRUE(snap.all_operational);
}

TEST(SharedStateGTest, OutOfRangeIgnored) {
  canopen_hw::SharedState shared;
  canopen_hw::AxisCommand cmd;
  cmd.target_position = 777;
  shared.UpdateCommand(0, cmd);

  cmd.target_position = 999;
  shared.UpdateCommand(99, cmd);

  const auto snap = shared.Snapshot();
  EXPECT_EQ(snap.commands[0].target_position, 777);
}

TEST(SharedStateGTest, RecomputeAllOperationalTrueWhenAllOperationalAndNoFault) {
  canopen_hw::SharedState shared;

  for (std::size_t i = 0; i < canopen_hw::SharedState::kAxisCount; ++i) {
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
  canopen_hw::SharedState shared;

  for (std::size_t i = 0; i < canopen_hw::SharedState::kAxisCount; ++i) {
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
