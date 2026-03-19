#include <gtest/gtest.h>

#include "canopen_hw/canopen_robot_hw.hpp"

TEST(RobotHw, ReadTicksToRad) {
  canopen_hw::SharedState shared;
  shared.SetActiveAxisCount(1);
  canopen_hw::CanopenRobotHw hw(&shared);

  canopen_hw::AxisFeedback fb;
  fb.actual_position = 5308416;  // 1 rev
  fb.actual_velocity = 5308416;  // 1 rev/s
  fb.actual_torque = 500;        // 50%
  fb.is_operational = true;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();

  hw.ReadFromSharedState();
  const double pos = hw.joint_position(0);
  EXPECT_GT(pos, 6.27);
  EXPECT_LT(pos, 6.29);
  EXPECT_TRUE(hw.all_operational());
}

TEST(RobotHw, WriteRadToTicks) {
  canopen_hw::SharedState shared;
  shared.SetActiveAxisCount(1);
  canopen_hw::CanopenRobotHw hw(&shared);

  canopen_hw::AxisFeedback fb;
  fb.is_operational = true;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();
  hw.ReadFromSharedState();

  hw.SetJointCommand(0, 3.14159265358979323846);
  hw.WriteToSharedState();
  const canopen_hw::SharedSnapshot snap = shared.Snapshot();
  EXPECT_GT(snap.commands[0].target_position, 2600000);
  EXPECT_LT(snap.commands[0].target_position, 2700000);
}
