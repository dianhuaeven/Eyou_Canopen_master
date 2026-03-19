#include <gtest/gtest.h>

#include "canopen_hw/lifecycle_manager.hpp"

using canopen_hw::LifecycleManager;
using canopen_hw::LifecycleState;

// LifecycleManager 的纯状态跃迁测试。
// 不启动真实 CAN 总线（Init 会因无 CAN 设备失败），
// 仅验证状态机守卫逻辑和 Shutdown 路径。

TEST(LifecycleManager, StartsUnconfigured) {
  LifecycleManager lm;
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_EQ(lm.master(), nullptr);
  EXPECT_EQ(lm.robot_hw(), nullptr);
  EXPECT_EQ(lm.shared_state(), nullptr);
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

TEST(LifecycleManager, InitRejectsInvalidAxisCount) {
  LifecycleManager lm;
  canopen_hw::CanopenMasterConfig config;
  config.axis_count = 0;
  EXPECT_FALSE(lm.Init(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, InitRejectsExcessiveAxisCount) {
  LifecycleManager lm;
  canopen_hw::CanopenMasterConfig config;
  config.axis_count = canopen_hw::SharedState::kMaxAxisCount + 1;
  EXPECT_FALSE(lm.Init(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, InitRejectsMissingJointsFile) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Init("/nonexistent/master.dcf", "/nonexistent/joints.yaml"));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}
