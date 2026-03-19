#include <gtest/gtest.h>

#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/cia402_state_machine.hpp"
#include "canopen_hw/shared_state.hpp"

using namespace canopen_hw;

// --- 状态机多模式测试 ---

class StateMachineMultiMode : public ::testing::Test {
 protected:
  CiA402StateMachine sm;

  void DriveToOperational(int8_t mode) {
    sm.set_target_mode(mode);
    sm.request_enable();
    sm.set_ros_target(1000);
    sm.set_position_lock_threshold(50000);

    // SwitchOnDisabled -> ReadyToSwitchOn
    sm.Update(kState_SwitchOnDisabled, mode, 1000);
    // ReadyToSwitchOn -> OperationEnabled (跳级)
    sm.Update(kState_ReadyToSwitchOn | (1u << 9), mode, 1000);
    // OperationEnabled
    sm.Update(kState_OperationEnabled, mode, 1000);
  }
};

TEST_F(StateMachineMultiMode, CSVModeEnablesCorrectly) {
  DriveToOperational(kMode_CSV);
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 0);  // 还在锁定阶段第一帧
}

TEST_F(StateMachineMultiMode, CSVModePassesVelocityAfterUnlock) {
  sm.set_target_mode(kMode_CSV);
  sm.request_enable();
  sm.set_ros_target(0);
  sm.set_ros_target_velocity(500);
  sm.set_position_lock_threshold(50000);

  sm.Update(kState_SwitchOnDisabled, kMode_CSV, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CSV, 0);
  // 首次进入 OperationEnabled: position locked, velocity=0
  sm.Update(kState_OperationEnabled, kMode_CSV, 0);
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 500);
}

TEST_F(StateMachineMultiMode, CSTModePassesTorqueAfterUnlock) {
  sm.set_target_mode(kMode_CST);
  sm.request_enable();
  sm.set_ros_target(0);
  sm.set_ros_target_torque(200);
  sm.set_position_lock_threshold(50000);

  sm.Update(kState_SwitchOnDisabled, kMode_CST, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CST, 0);
  sm.Update(kState_OperationEnabled, kMode_CST, 0);
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_torque(), 200);
}

TEST_F(StateMachineMultiMode, VelocityZeroedOnFault) {
  sm.set_target_mode(kMode_CSV);
  sm.request_enable();
  sm.set_ros_target(0);
  sm.set_ros_target_velocity(500);
  sm.set_position_lock_threshold(50000);

  sm.Update(kState_SwitchOnDisabled, kMode_CSV, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CSV, 0);
  sm.Update(kState_OperationEnabled, kMode_CSV, 0);
  EXPECT_EQ(sm.safe_target_velocity(), 500);

  // Fault
  sm.Update(kState_Fault, kMode_CSV, 0);
  EXPECT_EQ(sm.safe_target_velocity(), 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);
  EXPECT_FALSE(sm.is_operational());
}

TEST_F(StateMachineMultiMode, TorqueZeroedOnDisable) {
  sm.set_target_mode(kMode_CST);
  sm.request_enable();
  sm.set_ros_target(0);
  sm.set_ros_target_torque(300);
  sm.set_position_lock_threshold(50000);

  sm.Update(kState_SwitchOnDisabled, kMode_CST, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CST, 0);
  sm.Update(kState_OperationEnabled, kMode_CST, 0);
  EXPECT_EQ(sm.safe_target_torque(), 300);

  // Disable
  sm.request_disable();
  sm.Update(kState_SwitchedOn, kMode_CST, 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);
  EXPECT_EQ(sm.safe_target_velocity(), 0);
}

TEST_F(StateMachineMultiMode, SafeModeReflectsTargetMode) {
  sm.set_target_mode(kMode_CSV);
  EXPECT_EQ(sm.safe_mode_of_operation(), kMode_CSV);
  sm.set_target_mode(kMode_CST);
  EXPECT_EQ(sm.safe_mode_of_operation(), kMode_CST);
}

// --- RobotHw 多模式命令流测试 ---

class RobotHwMultiMode : public ::testing::Test {
 protected:
  SharedState state{2};
  CanopenRobotHw hw{&state};

  void MakeAllOperational() {
    for (std::size_t i = 0; i < 2; ++i) {
      AxisFeedback fb;
      fb.is_operational = true;
      fb.state = CiA402State::OperationEnabled;
      state.UpdateFeedback(i, fb);
    }
    state.RecomputeAllOperational();
    hw.ReadFromSharedState();
  }
};

TEST_F(RobotHwMultiMode, VelocityCommandWrittenToSharedState) {
  MakeAllOperational();
  hw.SetJointMode(0, kMode_CSV);
  hw.SetJointVelocityCommand(0, 1.0);
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(0, &cmd));
  EXPECT_EQ(cmd.mode_of_operation, kMode_CSV);
  EXPECT_NE(cmd.target_velocity, 0);
}

TEST_F(RobotHwMultiMode, TorqueCommandWrittenToSharedState) {
  MakeAllOperational();
  hw.SetJointMode(1, kMode_CST);
  hw.SetJointTorqueCommand(1, 3.0);
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(1, &cmd));
  EXPECT_EQ(cmd.mode_of_operation, kMode_CST);
  EXPECT_NE(cmd.target_torque, 0);
}

TEST_F(RobotHwMultiMode, DefaultModeIsCSP) {
  MakeAllOperational();
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(0, &cmd));
  EXPECT_EQ(cmd.mode_of_operation, kMode_CSP);
}

TEST_F(RobotHwMultiMode, NotOperationalBlocksWrite) {
  // Don't call MakeAllOperational.
  hw.SetJointMode(0, kMode_CSV);
  hw.SetJointVelocityCommand(0, 1.0);
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(0, &cmd));
  // Command should still be default (not written).
  EXPECT_EQ(cmd.mode_of_operation, kMode_CSP);
  EXPECT_EQ(cmd.target_velocity, 0);
}

TEST_F(RobotHwMultiMode, InvalidAxisIgnored) {
  hw.SetJointMode(99, kMode_CSV);
  hw.SetJointVelocityCommand(99, 1.0);
  hw.SetJointTorqueCommand(99, 1.0);
  // No crash expected.
}
