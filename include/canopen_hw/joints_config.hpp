#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "canopen_hw/canopen_robot_hw.hpp"

namespace canopen_hw {

struct JointCanopenConfig {
  uint8_t node_id = 0;
  bool verify_pdo_mapping = false;
  int32_t position_lock_threshold = 15000;
  int max_fault_resets = 3;
  int fault_reset_hold_cycles = 5;
};

struct MasterConfig {
  std::string can_interface = "can0";
  int bitrate = 1000000;
  uint8_t master_node_id = 127;
  int sync_period_us = 10000;
};

struct CanopenRuntimeConfig {
  MasterConfig master;
  std::vector<JointCanopenConfig> joints;
};

// 读取 joints.yaml 并将每轴换算参数注入 CanopenRobotHw。
// 返回 true 表示加载成功; 若失败可从 error 获取原因。
bool LoadJointsYaml(const std::string& path, CanopenRobotHw* robot_hw,
                    std::string* error,
                    CanopenRuntimeConfig* runtime_cfg = nullptr);

}  // namespace canopen_hw
