#include <cassert>
#include <fstream>
#include <string>

#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/shared_state.hpp"

int main() {
  canopen_hw::SharedState shared;
  canopen_hw::CanopenRobotHw hw(&shared);

  const std::string path = "/tmp/joints_test.yaml";
  {
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "      verify_pdo_mapping: true\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 10\n"
           "    velocity_scale: 2\n"
           "    torque_scale: 0.5\n"
           "  - name: joint_2\n"
           "    canopen:\n"
           "      node_id: 2\n"
           "      verify_pdo_mapping: false\n"
           "    counts_per_rev: 2000\n"
           "    rated_torque_nm: 20\n"
           "    velocity_scale: 1.5\n"
           "    torque_scale: 2.0\n";
  }

  std::string error;
  canopen_hw::CanopenRuntimeConfig runtime_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(path, &hw, &error, &runtime_cfg);
  assert(ok);
  assert(runtime_cfg.joints.size() == 2);
  assert(runtime_cfg.joints[0].node_id == 1);
  assert(runtime_cfg.joints[0].verify_pdo_mapping);
  assert(runtime_cfg.joints[1].node_id == 2);
  assert(!runtime_cfg.joints[1].verify_pdo_mapping);

  canopen_hw::AxisFeedback fb0;
  fb0.actual_position = 500;  // 0.5 rev -> pi
  fb0.actual_torque = 1000;   // 100% -> 10Nm * 0.5 -> 5Nm
  shared.UpdateFeedback(0, fb0);

  canopen_hw::AxisFeedback fb1;
  fb1.actual_position = 1000;  // 0.5 rev -> pi
  fb1.actual_torque = 1000;    // 100% -> 20Nm * 2 -> 40Nm
  shared.UpdateFeedback(1, fb1);

  hw.ReadFromSharedState();

  assert(hw.joint_position(0) > 3.13 && hw.joint_position(0) < 3.15);
  assert(hw.joint_position(1) > 3.13 && hw.joint_position(1) < 3.15);
  assert(hw.joint_effort(0) > 4.99 && hw.joint_effort(0) < 5.01);
  assert(hw.joint_effort(1) > 39.99 && hw.joint_effort(1) < 40.01);

  const std::string invalid_path = "/tmp/joints_test_invalid_node_id.yaml";
  {
    std::ofstream ofs(invalid_path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 300\n"
           "    counts_per_rev: 1000\n";
  }

  std::string invalid_error;
  const bool invalid_ok =
      canopen_hw::LoadJointsYaml(invalid_path, &hw, &invalid_error, nullptr);
  assert(!invalid_ok);
  assert(invalid_error.find("invalid node_id") != std::string::npos);

  return 0;
}
