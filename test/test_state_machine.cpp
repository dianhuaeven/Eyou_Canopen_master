#include <cassert>

#include "canopen_hw/cia402_state_machine.hpp"

using canopen_hw::CiA402State;
using canopen_hw::CiA402StateMachine;
using canopen_hw::kCtrl_EnableOperation;
using canopen_hw::kCtrl_FaultReset;
using canopen_hw::kCtrl_Shutdown;
using canopen_hw::kMode_CSP;

int main() {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  // 1) SwitchOnDisabled -> 应发 Shutdown
  sm.Update(0x0040, kMode_CSP, 100);
  assert(sm.state() == CiA402State::SwitchOnDisabled);
  assert(sm.controlword() == kCtrl_Shutdown);

  // 2) ReadyToSwitchOn + mode ok -> 直接 0x000F
  sm.Update(0x0021, kMode_CSP, 100);
  assert(sm.state() == CiA402State::ReadyToSwitchOn);
  assert(sm.controlword() == kCtrl_EnableOperation);

  // 3) 首次进入 OperationEnabled 时锁定位置
  sm.set_ros_target(500000);
  sm.Update(0x0027, kMode_CSP, 12345);
  assert(sm.state() == CiA402State::OperationEnabled);
  assert(sm.is_position_locked());
  assert(sm.safe_target() == 12345);

  // 4) ROS 目标接近后解锁
  sm.set_ros_target(12350);
  sm.Update(0x0027, kMode_CSP, 12348);
  assert(!sm.is_position_locked());
  assert(sm.is_operational());

  // 5) Fault 状态下进入复位流程, 最终应出现 fault reset 边沿
  sm.set_fault_reset_policy(2, 5, 2);
  sm.Update(0x0008, kMode_CSP, 12348);  // Hold 1
  sm.Update(0x0008, kMode_CSP, 12348);  // Hold 2
  sm.Update(0x0008, kMode_CSP, 12348);  // SendEdge
  assert(sm.controlword() == kCtrl_FaultReset);

  return 0;
}
