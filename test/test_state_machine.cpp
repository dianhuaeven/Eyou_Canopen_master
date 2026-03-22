#include <gtest/gtest.h>

#include "canopen_hw/cia402_state_machine.hpp"

using canopen_hw::CiA402State;
using canopen_hw::CiA402StateMachine;
using canopen_hw::kCtrl_DisableOperation;
using canopen_hw::kCtrl_EnableOperation;
using canopen_hw::kCtrl_Bit_Halt;
using canopen_hw::kCtrl_FaultReset;
using canopen_hw::kCtrl_Shutdown;
using canopen_hw::kMode_CSP;
using canopen_hw::kMode_CSV;

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

// 回归测试：Recover 后 tpdo 初始为零，驱动器可能短暂收到 mode=0 导致
// mode_display 归零。状态机不应因此卡死在 ReadyToSwitchOn。
TEST(CiA402SM, ReadyToSwitchOnEnablesEvenIfModeDisplayIsZero) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  // mode_display=0（驱动器因收到 mode=0 而清除了模式）
  sm.Update(0x0021, /*mode_display=*/0, 100);
  EXPECT_EQ(sm.state(), CiA402State::ReadyToSwitchOn);
  // 修复后：仍应发 EnableOperation，不因 mode_display 不匹配而卡住
  EXPECT_EQ(sm.controlword(), kCtrl_EnableOperation);
}

TEST(CiA402SM, FirstOperationEnabledAutoAlignsFarTarget) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);

  // 目标位置与实际位置差距很大时，状态机会自动重基准，
  // 避免长期停留在 not operational。
  sm.set_ros_target(500000);
  sm.Update(0x0027, kMode_CSP, 12345);
  EXPECT_EQ(sm.state(), CiA402State::OperationEnabled);
  EXPECT_FALSE(sm.is_position_locked());
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), 12345);
}

TEST(CiA402SM, RosTargetCloseUnlocks) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);

  sm.set_ros_target(12350);
  sm.Update(0x0027, kMode_CSP, 12348);
  EXPECT_FALSE(sm.is_position_locked());
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), 12350);
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

TEST(CiA402SM, DisableRequestDropsOperationalInOperationEnabled) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSV);

  sm.request_enable();
  sm.set_ros_target_velocity(500);

  // 进入 CSV 运行态：首帧归零，次帧解锁透传。
  sm.Update(0x0040, kMode_CSV, 1000);
  sm.Update(0x0021, kMode_CSV, 1000);
  sm.Update(0x0027, kMode_CSV, 1000);
  sm.Update(0x0027, kMode_CSV, 1000);
  ASSERT_TRUE(sm.is_operational());
  ASSERT_EQ(sm.safe_target_velocity(), 500);

  // 即使状态字暂时仍是 OperationEnabled，收到 disable 请求后也应立即回落。
  sm.request_disable();
  sm.Update(0x0027, kMode_CSV, 1000);

  EXPECT_EQ(sm.controlword(), kCtrl_DisableOperation);
  EXPECT_FALSE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);
}

TEST(CiA402SM, HaltBitFreezesTargetsAndResumeClearsBit) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSV);

  sm.request_enable();
  sm.set_ros_target_velocity(500);

  sm.Update(0x0040, kMode_CSV, 1000);
  sm.Update(0x0021, kMode_CSV, 1000);
  sm.Update(0x0027, kMode_CSV, 1000);  // CSV 首帧
  sm.Update(0x0027, kMode_CSV, 1000);  // CSV 解锁
  ASSERT_TRUE(sm.is_operational());
  ASSERT_EQ(sm.safe_target_velocity(), 500);

  sm.request_halt();
  sm.Update(0x0027, kMode_CSV, 1012);
  EXPECT_EQ(sm.controlword(), static_cast<uint16_t>(kCtrl_EnableOperation | kCtrl_Bit_Halt));
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), 1012);
  EXPECT_EQ(sm.safe_target_velocity(), 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);

  sm.set_ros_target_velocity(900);
  sm.Update(0x0027, kMode_CSV, 1020);
  EXPECT_EQ(sm.safe_target(), 1020);
  EXPECT_EQ(sm.safe_target_velocity(), 0);

  sm.request_resume();
  sm.Update(0x0027, kMode_CSV, 1020);
  EXPECT_EQ(sm.controlword(), kCtrl_EnableOperation);
  EXPECT_EQ(sm.safe_target_velocity(), 900);
}
