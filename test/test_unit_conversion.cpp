#include <cassert>

#include "canopen_hw/canopen_robot_hw.hpp"

int main() {
  canopen_hw::SharedState shared;
  shared.SetActiveAxisCount(1);
  canopen_hw::CanopenRobotHw hw(&shared);

  // 轴0使用默认参数: 1 rev -> 2*pi rad
  canopen_hw::AxisFeedback fb0;
  fb0.actual_position = 5308416;
  fb0.is_operational = true;
  shared.UpdateFeedback(0, fb0);
  shared.RecomputeAllOperational();
  hw.ReadFromSharedState();
  assert(hw.joint_position(0) > 6.27 && hw.joint_position(0) < 6.29);

  // 轴1配置不同分辨率与比例，验证每轴参数生效。
  canopen_hw::CanopenRobotHw::AxisConversion conv1;
  conv1.counts_per_rev = 1000.0;
  conv1.rated_torque_nm = 10.0;
  conv1.velocity_scale = 2.0;
  conv1.torque_scale = 0.5;
  hw.ConfigureAxisConversion(1, conv1);

  canopen_hw::AxisFeedback fb1;
  fb1.actual_position = 500;   // 0.5 rev -> pi
  fb1.actual_velocity = 500;   // 0.5 rev/s -> pi rad/s, 再乘2 -> 2*pi
  fb1.actual_torque = 1000;    // 100% -> 10Nm, 再乘0.5 -> 5Nm
  shared.UpdateFeedback(1, fb1);
  hw.ReadFromSharedState();

  assert(hw.joint_position(1) > 3.13 && hw.joint_position(1) < 3.15);
  assert(hw.joint_velocity(1) > 6.27 && hw.joint_velocity(1) < 6.29);
  assert(hw.joint_effort(1) > 4.99 && hw.joint_effort(1) < 5.01);

  // all_operational=false 时 write() 不应覆盖命令。
  canopen_hw::AxisCommand cmd_before;
  cmd_before.target_position = 777;
  shared.UpdateCommand(2, cmd_before);
  fb0.is_operational = false;
  fb0.is_fault = true;
  shared.UpdateFeedback(0, fb0);
  shared.RecomputeAllOperational();
  hw.ReadFromSharedState();
  hw.SetJointCommand(2, 1.23);
  hw.WriteToSharedState();
  const auto snap = shared.Snapshot();
  assert(snap.commands[2].target_position == 777);

  return 0;
}
