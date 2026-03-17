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

  if (runtime_cfg) {
    const YAML::Node canopen = root["canopen"];
    if (canopen && canopen.IsMap() && canopen["auto_fix_pdo"]) {
      runtime_cfg->auto_fix_pdo = canopen["auto_fix_pdo"].as<bool>();
    }
  }

  const YAML::Node joints = root["joints"];
  if (!joints || !joints.IsSequence()) {
    SetError(error, "joints is missing or not a sequence");
    return false;
  }

  std::size_t loaded = 0;
  for (const auto& joint : joints) {
    if (!joint || !joint.IsMap()) {
      continue;
    }
    if (!joint["node_id"]) {
      continue;
    }

    const int node_id = joint["node_id"].as<int>();
    if (node_id <= 0 || node_id > static_cast<int>(CanopenRobotHw::kAxisCount)) {
      continue;
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

    robot_hw->ConfigureAxisConversion(static_cast<std::size_t>(node_id - 1),
                                      conv);
    ++loaded;
  }

  if (loaded == 0) {
    SetError(error, "no joints loaded");
    return false;
  }

  return true;
}

}  // namespace canopen_hw
