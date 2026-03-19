#include <gtest/gtest.h>

#include "canopen_hw/shared_state.hpp"

TEST(SharedState, BasicUpdateAndSnapshot) {
  canopen_hw::SharedState shared(1);

  canopen_hw::AxisFeedback fb;
  fb.actual_position = 123456;
  fb.actual_velocity = -345;
  fb.is_operational = true;

  canopen_hw::AxisCommand cmd;
  cmd.target_position = 223344;

  shared.UpdateFeedback(0, fb);
  shared.UpdateCommand(0, cmd);
  shared.RecomputeAllOperational();

  const canopen_hw::SharedSnapshot snap = shared.Snapshot();
  EXPECT_EQ(snap.feedback[0].actual_position, 123456);
  EXPECT_EQ(snap.feedback[0].actual_velocity, -345);
  EXPECT_TRUE(snap.feedback[0].is_operational);
  EXPECT_EQ(snap.commands[0].target_position, 223344);
  EXPECT_TRUE(snap.all_operational);
}

TEST(SharedState, OutOfRangeIgnored) {
  canopen_hw::SharedState shared(1);

  canopen_hw::AxisCommand cmd;
  cmd.target_position = 223344;
  shared.UpdateCommand(0, cmd);

  // 越界写入应被静默忽略, 不影响已有数据。
  cmd.target_position = 999;
  shared.UpdateCommand(99, cmd);
  const canopen_hw::SharedSnapshot snap2 = shared.Snapshot();
  EXPECT_EQ(snap2.commands[0].target_position, 223344);
}
