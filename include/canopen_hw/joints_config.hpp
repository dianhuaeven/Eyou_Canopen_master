#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "canopen_hw/canopen_robot_hw.hpp"

namespace canopen_hw {

struct JointCanopenConfig {
  uint8_t node_id = 0;
  bool verify_pdo_mapping = false;
};

struct CanopenRuntimeConfig {
  std::vector<JointCanopenConfig> joints;
};

// 读取 joints.yaml 并将每轴换算参数注入 CanopenRobotHw。
// 返回 true 表示加载成功; 若失败可从 error 获取原因。
bool LoadJointsYaml(const std::string& path, CanopenRobotHw* robot_hw,
                    std::string* error,
                    CanopenRuntimeConfig* runtime_cfg = nullptr);

}  // namespace canopen_hw
