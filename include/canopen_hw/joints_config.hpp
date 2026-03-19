#pragma once

#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"

namespace canopen_hw {

// 读取 joints.yaml 并将配置填入 CanopenMasterConfig 和 CanopenRobotHw。
// config 可为 nullptr（此时仅加载换算参数到 robot_hw）。
// 返回 true 表示加载成功; 若失败可从 error 获取原因。
bool LoadJointsYaml(const std::string& path,
                    CanopenRobotHw* robot_hw,
                    std::string* error,
                    CanopenMasterConfig* config = nullptr);

}  // namespace canopen_hw
