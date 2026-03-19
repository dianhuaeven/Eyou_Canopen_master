#include <gtest/gtest.h>

#include "canopen_hw/cia402_state_machine.hpp"

using canopen_hw::CiA402State;
using canopen_hw::CiA402StateMachine;
using canopen_hw::kCtrl_EnableOperation;
using canopen_hw::kCtrl_FaultReset;
using canopen_hw::kCtrl_Shutdown;
using canopen_hw::kMode_CSP;

TEST(CiA402SM, SwitchOnDisabledSendsShutdown) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  EXPECT_EQ(sm.state(), CiA402State::SwitchOnDisabled);
  EXPECT_EQ(sm.controlword(), kCtrl_Shutdown);
}

TEST(CiA402SM, ReadyToSwitchOnJumpEnable) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);
  EXPECT_EQ(sm.state(), CiA402State::ReadyToSwitchOn);
  EXPECT_EQ(sm.controlword(), kCtrl_EnableOperation);
}

TEST(CiA402SM, FirstOperationEnabledLocksPosition) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);

  sm.set_ros_target(500000);
  sm.Update(0x0027, kMode_CSP, 12345);
  EXPECT_EQ(sm.state(), CiA402State::OperationEnabled);
  EXPECT_TRUE(sm.is_position_locked());
  EXPECT_EQ(sm.safe_target(), 12345);
}

TEST(CiA402SM, RosTargetCloseUnlocks) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);

  sm.set_ros_target(500000);
  sm.Update(0x0027, kMode_CSP, 12345);
  EXPECT_TRUE(sm.is_position_locked());

  sm.set_ros_target(12350);
  sm.Update(0x0027, kMode_CSP, 12348);
  EXPECT_FALSE(sm.is_position_locked());
  EXPECT_TRUE(sm.is_operational());
}

TEST(CiA402SM, FaultResetThreePhaseFlow) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);
  sm.set_ros_target(500000);
  sm.Update(0x0027, kMode_CSP, 12345);
  sm.set_ros_target(12350);
  sm.Update(0x0027, kMode_CSP, 12348);

  sm.set_fault_reset_policy(2, 5, 2);

  // FaultReactionActive: 不应触发复位边沿
  sm.Update(0x000F, kMode_CSP, 12348);
  EXPECT_EQ(sm.state(), CiA402State::FaultReactionActive);
  EXPECT_NE(sm.controlword(), kCtrl_FaultReset);

  // Hold 阶段
  sm.Update(0x0008, kMode_CSP, 12348);  // Hold 1
  sm.Update(0x0008, kMode_CSP, 12348);  // Hold 2
  // SendEdge
  sm.Update(0x0008, kMode_CSP, 12348);
  EXPECT_EQ(sm.controlword(), kCtrl_FaultReset);
  EXPECT_EQ(sm.fault_reset_count(), 1);
}
