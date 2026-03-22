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

bool LoadJointsYaml(const std::string& path, std::string* error,
                    CanopenMasterConfig* config) {
  if (!config) {
    SetError(error, "config is null");
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

  const YAML::Node top_canopen = root["canopen"];
  if (top_canopen && top_canopen.IsMap()) {
    try {
      if (top_canopen["interface"]) {
        config->can_interface = top_canopen["interface"].as<std::string>();
      }
      if (top_canopen["master_node_id"]) {
        const int master_node_id = top_canopen["master_node_id"].as<int>();
        if (master_node_id > 0 && master_node_id <= 255) {
          config->master_node_id = static_cast<uint8_t>(master_node_id);
        }
      }
      if (top_canopen["loop_hz"]) {
        const double hz = top_canopen["loop_hz"].as<double>();
        if (hz > 0) {
          config->loop_hz = hz;
        }
      }
    } catch (const YAML::Exception& e) {
      std::ostringstream oss;
      oss << "invalid field type at canopen: " << e.what();
      SetError(error, oss.str());
      return false;
    }
  }
  config->joints.clear();
  config->joints.reserve(joints.size());

  std::size_t loaded = 0;
  std::size_t axis_index = 0;
  for (const auto& joint : joints) {
    if (!joint || !joint.IsMap()) {
      continue;
    }
    const YAML::Node canopen = joint["canopen"];
    CanopenMasterConfig::JointConfig jcfg;
    if (joint["name"]) {
      jcfg.name = joint["name"].as<std::string>();
    } else {
      jcfg.name = "joint_" + std::to_string(axis_index + 1);
    }
    try {
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
      if (joint["max_velocity_for_clamp"]) {
        jcfg.max_velocity_for_clamp = joint["max_velocity_for_clamp"].as<double>();
      }
      if (joint["counts_per_rev"]) {
        jcfg.counts_per_rev = joint["counts_per_rev"].as<double>();
      }
      if (joint["rated_torque_nm"]) {
        jcfg.rated_torque_nm = joint["rated_torque_nm"].as<double>();
      }
      if (joint["velocity_scale"]) {
        jcfg.velocity_scale = joint["velocity_scale"].as<double>();
      }
      if (joint["torque_scale"]) {
        jcfg.torque_scale = joint["torque_scale"].as<double>();
      }
    } catch (const YAML::Exception& e) {
      std::ostringstream oss;
      oss << "invalid field type at joints[" << axis_index << "]: " << e.what();
      SetError(error, oss.str());
      return false;
    }

    // 参数合法性校验: counts_per_rev 和 rated_torque_nm 必须为正值。
    if (jcfg.counts_per_rev <= 0) {
      std::ostringstream oss;
      oss << "invalid counts_per_rev at joints[" << axis_index
          << "]: expected > 0, got " << jcfg.counts_per_rev;
      SetError(error, oss.str());
      return false;
    }
    if (jcfg.rated_torque_nm <= 0) {
      std::ostringstream oss;
      oss << "invalid rated_torque_nm at joints[" << axis_index
          << "]: expected > 0, got " << jcfg.rated_torque_nm;
      SetError(error, oss.str());
      return false;
    }
    if (jcfg.max_velocity_for_clamp <= 0) {
      std::ostringstream oss;
      oss << "invalid max_velocity_for_clamp at joints[" << axis_index
          << "]: expected > 0, got " << jcfg.max_velocity_for_clamp;
      SetError(error, oss.str());
      return false;
    }

    config->joints.push_back(jcfg);
    ++loaded;
    ++axis_index;
  }

  if (loaded == 0) {
    SetError(error, "no joints loaded");
    return false;
  }

  config->axis_count = config->joints.size();
  return true;
}

}  // namespace canopen_hw
