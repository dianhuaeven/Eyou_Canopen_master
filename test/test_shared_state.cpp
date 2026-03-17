#include <cassert>

#include "canopen_hw/shared_state.hpp"

int main() {
  canopen_hw::SharedState shared;

  canopen_hw::AxisFeedback fb;
  fb.actual_position = 123456;
  fb.actual_velocity = -345;
  fb.is_operational = true;

  canopen_hw::AxisCommand cmd;
  cmd.target_position = 223344;

  shared.UpdateFeedback(0, fb);
  shared.UpdateCommand(0, cmd);
  shared.SetAllOperational(true);

  const canopen_hw::SharedSnapshot snap = shared.Snapshot();
  assert(snap.feedback[0].actual_position == 123456);
  assert(snap.feedback[0].actual_velocity == -345);
  assert(snap.feedback[0].is_operational);
  assert(snap.commands[0].target_position == 223344);
  assert(snap.all_operational);

  // 越界写入应被静默忽略, 不影响已有数据。
  cmd.target_position = 999;
  shared.UpdateCommand(99, cmd);
  const canopen_hw::SharedSnapshot snap2 = shared.Snapshot();
  assert(snap2.commands[0].target_position == 223344);

  return 0;
}
