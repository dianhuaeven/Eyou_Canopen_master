#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/shared_state.hpp"

TEST(StartupIntegration, ConfigPropagationFromYamlToRobotHw) {
  const std::string path = "/tmp/joints_startup_integration.yaml";
  {
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 10\n"
           "    velocity_scale: 2\n"
           "    torque_scale: 0.5\n"
           "  - name: joint_2\n"
           "    canopen:\n"
           "      node_id: 2\n"
           "    counts_per_rev: 2000\n"
           "    rated_torque_nm: 20\n"
           "    velocity_scale: 1.5\n"
           "    torque_scale: 2.0\n";
  }

  canopen_hw::CanopenMasterConfig cfg;
  std::string error;
  ASSERT_TRUE(canopen_hw::LoadJointsYaml(path, &error, &cfg)) << error;
  ASSERT_EQ(cfg.axis_count, 2u);

  canopen_hw::SharedState shared(cfg.axis_count);
  canopen_hw::CanopenRobotHw hw(&shared);
  hw.ApplyConfig(cfg);

  canopen_hw::AxisFeedback fb0;
  fb0.actual_position = 500;  // 0.5 rev -> pi
  fb0.actual_torque = 1000;   // 10Nm * 0.5 = 5Nm
  shared.UpdateFeedback(0, fb0);

  canopen_hw::AxisFeedback fb1;
  fb1.actual_position = 1000;  // 0.5 rev -> pi
  fb1.actual_torque = 1000;    // 20Nm * 2.0 = 40Nm
  shared.UpdateFeedback(1, fb1);

  hw.ReadFromSharedState();

  EXPECT_NEAR(hw.joint_position(0), 3.1415926, 0.01);
  EXPECT_NEAR(hw.joint_position(1), 3.1415926, 0.01);
  EXPECT_NEAR(hw.joint_effort(0), 5.0, 0.01);
  EXPECT_NEAR(hw.joint_effort(1), 40.0, 0.01);
}
