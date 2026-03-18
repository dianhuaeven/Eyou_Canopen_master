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

}  // namespace

bool LoadJointsYaml(const std::string& path, CanopenRobotHw* robot_hw,
                    std::string* error,
                    CanopenRuntimeConfig* runtime_cfg) {
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

  if (runtime_cfg) {
    runtime_cfg->joints.clear();
    runtime_cfg->joints.reserve(joints.size());
  }

  std::size_t loaded = 0;
  std::size_t axis_index = 0;
  for (const auto& joint : joints) {
    if (!joint || !joint.IsMap()) {
      continue;
    }
    const YAML::Node canopen = joint["canopen"];
    int node_id = 0;
    if (canopen && canopen.IsMap() && canopen["node_id"]) {
      node_id = canopen["node_id"].as<int>();
    } else if (joint["node_id"]) {
      node_id = joint["node_id"].as<int>();
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

    if (node_id > 0 &&
        node_id <= static_cast<int>(CanopenRobotHw::kAxisCount)) {
      robot_hw->ConfigureAxisConversion(static_cast<std::size_t>(node_id - 1),
                                        conv);
    } else if (axis_index < CanopenRobotHw::kAxisCount) {
      robot_hw->ConfigureAxisConversion(axis_index, conv);
    }

    if (runtime_cfg) {
      JointCanopenConfig cfg;
      cfg.node_id = node_id > 0 ? static_cast<uint8_t>(node_id)
                                : static_cast<uint8_t>(axis_index + 1);
      if (canopen && canopen.IsMap() && canopen["verify_pdo_mapping"]) {
        cfg.verify_pdo_mapping = canopen["verify_pdo_mapping"].as<bool>();
      }
      if (joint["position_lock_threshold"]) {
        cfg.position_lock_threshold = joint["position_lock_threshold"].as<int>();
      }
      if (joint["max_fault_resets"]) {
        cfg.max_fault_resets = joint["max_fault_resets"].as<int>();
      }
      if (joint["fault_reset_hold_cycles"]) {
        cfg.fault_reset_hold_cycles =
            joint["fault_reset_hold_cycles"].as<int>();
      }
      runtime_cfg->joints.push_back(cfg);
    }

    ++loaded;
    ++axis_index;
  }

  if (loaded == 0) {
    SetError(error, "no joints loaded");
    return false;
  }

  return true;
}

}  // namespace canopen_hw
