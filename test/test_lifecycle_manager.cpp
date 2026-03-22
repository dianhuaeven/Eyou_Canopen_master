#include <gtest/gtest.h>

#include "canopen_hw/lifecycle_manager.hpp"

using canopen_hw::CanopenMasterConfig;
using canopen_hw::LifecycleManager;
using canopen_hw::LifecycleState;

namespace {

CanopenMasterConfig MakeMinimalConfig() {
  CanopenMasterConfig config;
  config.axis_count = 1;
  config.can_interface = "can0";
  config.master_dcf_path = "/tmp/definitely_missing_master_for_lifecycle_test.dcf";
  config.joints.resize(1);
  config.joints[0].name = "joint_1";
  config.joints[0].node_id = 1;
  return config;
}

}  // namespace

// LifecycleManager 的状态跃迁测试。
// 不依赖真实 CAN 总线：InitMotors/Init 在当前环境应失败，
// 重点验证 guard 与失败语义。

TEST(LifecycleManager, StartsUnconfigured) {
  LifecycleManager lm;
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_EQ(lm.master(), nullptr);
  EXPECT_EQ(lm.robot_hw(), nullptr);
  EXPECT_EQ(lm.shared_state(), nullptr);
  EXPECT_FALSE(lm.ever_initialized());
}

TEST(LifecycleManager, HaltRejectsWhenUnconfigured) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Halt());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, RecoverRejectsWhenUnconfigured) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Recover());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, ShutdownFromUnconfiguredIsNoop) {
  LifecycleManager lm;
  EXPECT_TRUE(lm.Shutdown());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, ConfigureRejectsInvalidAxisCount) {
  LifecycleManager lm;
  CanopenMasterConfig config;
  config.axis_count = 0;
  EXPECT_FALSE(lm.Configure(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, ConfigureRejectsExcessiveAxisCount) {
  LifecycleManager lm;
  CanopenMasterConfig config;
  config.axis_count = canopen_hw::SharedState::kMaxAxisCount + 1;
  EXPECT_FALSE(lm.Configure(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, ConfigureValidConfigEntersConfigured) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  EXPECT_TRUE(lm.Configure(config));
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
  EXPECT_NE(lm.master(), nullptr);
  EXPECT_NE(lm.robot_hw(), nullptr);
  EXPECT_NE(lm.shared_state(), nullptr);
  EXPECT_FALSE(lm.ever_initialized());
}

TEST(LifecycleManager, RecoverRejectsBeforeFirstInitMotors) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  ASSERT_TRUE(lm.Configure(config));
  EXPECT_FALSE(lm.Recover());
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
}

TEST(LifecycleManager, InitMotorsFailureKeepsConfigured) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  ASSERT_TRUE(lm.Configure(config));
  EXPECT_FALSE(lm.InitMotors());
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
  EXPECT_FALSE(lm.ever_initialized());
  EXPECT_NE(lm.master(), nullptr);
}

TEST(LifecycleManager, InitCompatibilityFailureRollsBackToUnconfigured) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  EXPECT_FALSE(lm.Init(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_EQ(lm.master(), nullptr);
  EXPECT_EQ(lm.robot_hw(), nullptr);
  EXPECT_EQ(lm.shared_state(), nullptr);
  EXPECT_FALSE(lm.ever_initialized());
}

TEST(LifecycleManager, InitRejectsMissingJointsFile) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Init("/nonexistent/master.dcf", "/nonexistent/joints.yaml"));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}
