#include "canopen_hw/joints_config.hpp"

#include <sstream>

#include <yaml-cpp/yaml.h>

namespace canopen_hw {

namespace {

void SetError(std::string* error, const std::string& msg) {
  if (error) {
    *error = msg;
  }
}

bool IsValidCanopenNodeId(int node_id) {
  return node_id >= 1 && node_id <= 127;
}

}  // namespace

bool LoadJointsYaml(const std::string& path, CanopenRobotHw* robot_hw,
                    std::string* error,
                    CanopenMasterConfig* config) {
  if (!robot_hw) {
    SetError(error, "robot_hw is null");
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    std::ostringstream oss;
    oss << "failed to load joints yaml: " << e.what();
    SetError(error, oss.str());
    return false;
  }

  const YAML::Node joints = root["joints"];
  if (!joints || !joints.IsSequence()) {
    SetError(error, "joints is missing or not a sequence");
    return false;
  }

  if (config) {
    const YAML::Node top_canopen = root["canopen"];
    if (top_canopen && top_canopen.IsMap()) {
      if (top_canopen["interface"]) {
        config->can_interface =
            top_canopen["interface"].as<std::string>();
      }
      if (top_canopen["master_node_id"]) {
        const int master_node_id = top_canopen["master_node_id"].as<int>();
        if (master_node_id > 0 && master_node_id <= 255) {
          config->master_node_id =
              static_cast<uint8_t>(master_node_id);
        }
      }
    }
    config->joints.clear();
    config->joints.reserve(joints.size());
  }

  std::size_t loaded = 0;
  std::size_t axis_index = 0;
  for (const auto& joint : joints) {
    if (!joint || !joint.IsMap()) {
      continue;
    }
    const YAML::Node canopen = joint["canopen"];
    const bool has_node_id =
        (canopen && canopen.IsMap() && canopen["node_id"]) || joint["node_id"];
    int node_id = 0;
    if (canopen && canopen.IsMap() && canopen["node_id"]) {
      node_id = canopen["node_id"].as<int>();
    } else if (joint["node_id"]) {
      node_id = joint["node_id"].as<int>();
    }
    if (has_node_id && !IsValidCanopenNodeId(node_id)) {
      std::ostringstream oss;
      oss << "invalid node_id at joints[" << axis_index
          << "]: expected 1..127, got " << node_id;
      SetError(error, oss.str());
      return false;
    }

    CanopenRobotHw::AxisConversion conv;
    if (joint["counts_per_rev"]) {
      conv.counts_per_rev = joint["counts_per_rev"].as<double>();
    }
    if (joint["rated_torque_nm"]) {
      conv.rated_torque_nm = joint["rated_torque_nm"].as<double>();
    }
    if (joint["velocity_scale"]) {
      conv.velocity_scale = joint["velocity_scale"].as<double>();
    }
    if (joint["torque_scale"]) {
      conv.torque_scale = joint["torque_scale"].as<double>();
    }

    // 参数合法性校验: counts_per_rev 和 rated_torque_nm 必须为正值。
    if (conv.counts_per_rev <= 0) {
      std::ostringstream oss;
      oss << "invalid counts_per_rev at joints[" << axis_index
          << "]: expected > 0, got " << conv.counts_per_rev;
      SetError(error, oss.str());
      return false;
    }
    if (conv.rated_torque_nm <= 0) {
      std::ostringstream oss;
      oss << "invalid rated_torque_nm at joints[" << axis_index
          << "]: expected > 0, got " << conv.rated_torque_nm;
      SetError(error, oss.str());
      return false;
    }

    // 统一映射规则: YAML 中第 N 个 joint 映射到 axis_index = N (从 0 开始),
    // node_id 仅用于总线通信, 不参与数组索引。
    if (axis_index < CanopenRobotHw::kAxisCount) {
      robot_hw->ConfigureAxisConversion(axis_index, conv);
    }

    if (config) {
      CanopenMasterConfig::JointConfig jcfg;
      jcfg.node_id = node_id > 0 ? static_cast<uint8_t>(node_id)
                                  : static_cast<uint8_t>(axis_index + 1);
      if (canopen && canopen.IsMap() && canopen["verify_pdo_mapping"]) {
        jcfg.verify_pdo_mapping = canopen["verify_pdo_mapping"].as<bool>();
      }
      if (joint["position_lock_threshold"]) {
        jcfg.position_lock_threshold = joint["position_lock_threshold"].as<int>();
      }
      if (joint["max_fault_resets"]) {
        jcfg.max_fault_resets = joint["max_fault_resets"].as<int>();
      }
      if (joint["fault_reset_hold_cycles"]) {
        jcfg.fault_reset_hold_cycles =
            joint["fault_reset_hold_cycles"].as<int>();
      }
      config->joints.push_back(jcfg);
    }

    ++loaded;
    ++axis_index;
  }

  if (loaded == 0) {
    SetError(error, "no joints loaded");
    return false;
  }

  // 从 joints 同步到扁平 vector，更新 axis_count。
  if (config) {
    config->SyncFromJoints();
  }

  return true;
}

}  // namespace canopen_hw
