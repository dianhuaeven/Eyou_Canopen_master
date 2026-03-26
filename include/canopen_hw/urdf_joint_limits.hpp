#pragma once

#include <string>
#include <vector>

#include "canopen_hw/canopen_master.hpp"

namespace canopen_hw {

struct JointLimitRad {
  double lower = 0.0;
  double upper = 0.0;
};

// 从 robot_description(URDF XML) 提取 joints 配置对应的关节上下限。
// 约束:
// - 仅接受 revolute/prismatic 且具备 limits 的关节。
// - joints 顺序与输入 config.joints 顺序保持一致。
bool ParseUrdfJointLimits(const std::string& urdf_xml,
                          const std::vector<CanopenMasterConfig::JointConfig>& joints,
                          std::vector<JointLimitRad>* out_limits,
                          std::string* error = nullptr);

}  // namespace canopen_hw
