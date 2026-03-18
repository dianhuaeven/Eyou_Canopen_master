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
