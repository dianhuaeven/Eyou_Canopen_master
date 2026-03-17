#include <cassert>

#include "canopen_hw/canopen_robot_hw.hpp"

int main() {
  canopen_hw::SharedState shared;
  canopen_hw::CanopenRobotHw hw(&shared);

  // 准备一帧反馈数据并标记可运行。
  canopen_hw::AxisFeedback fb;
  fb.actual_position = 5308416;  // 1 rev
  fb.actual_velocity = 5308416;  // 1 rev/s
  fb.actual_torque = 500;        // 50%
  shared.UpdateFeedback(0, fb);
  shared.SetAllOperational(true);

  // read: ticks -> rad 换算应接近 2*pi。
  hw.ReadFromSharedState();
  const double pos = hw.joint_position(0);
  assert(pos > 6.27 && pos < 6.29);
  assert(hw.all_operational());

  // write: 命令写回 shared_state。
  hw.SetJointCommand(0, 3.14159265358979323846);
  hw.WriteToSharedState();
  const canopen_hw::SharedSnapshot snap = shared.Snapshot();
  assert(snap.commands[0].target_position > 2600000);
  assert(snap.commands[0].target_position < 2700000);

  return 0;
}
