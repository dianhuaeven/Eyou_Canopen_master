#include "canopen_hw/urdf_joint_limits.hpp"

#include <cmath>
#include <sstream>

#include <urdf/model.h>

namespace canopen_hw {

namespace {

void SetError(std::string* error, const std::string& message) {
  if (error) {
    *error = message;
  }
}

bool IsSupportedLimitedType(int joint_type) {
  return joint_type == urdf::Joint::REVOLUTE || joint_type == urdf::Joint::PRISMATIC;
}

}  // namespace

bool ParseUrdfJointLimits(
    const std::string& urdf_xml,
    const std::vector<CanopenMasterConfig::JointConfig>& joints,
    std::vector<JointLimitSpec>* out_limits, std::string* error) {
  if (!out_limits) {
    SetError(error, "out_limits is null");
    return false;
  }
  out_limits->clear();
  out_limits->reserve(joints.size());

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    SetError(error, "failed to parse robot_description");
    return false;
  }

  for (std::size_t axis_index = 0; axis_index < joints.size(); ++axis_index) {
    const auto& cfg = joints[axis_index];
    const auto joint = model.getJoint(cfg.name);
    if (!joint) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " joint '" << cfg.name
          << "' not found in URDF";
      SetError(error, oss.str());
      return false;
    }
    if (!IsSupportedLimitedType(joint->type)) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " joint '" << cfg.name
          << "' type is not limited revolute/prismatic";
      SetError(error, oss.str());
      return false;
    }
    if (!joint->limits) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " joint '" << cfg.name
          << "' has no position limits";
      SetError(error, oss.str());
      return false;
    }

    const double lower = joint->limits->lower;
    const double upper = joint->limits->upper;
    if (!std::isfinite(lower) || !std::isfinite(upper)) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " joint '" << cfg.name
          << "' limits are not finite";
      SetError(error, oss.str());
      return false;
    }
    if (lower > upper) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " joint '" << cfg.name
          << "' has invalid limits: lower > upper";
      SetError(error, oss.str());
      return false;
    }

    UrdfJointLimitUnit unit = UrdfJointLimitUnit::kRadians;
    if (joint->type == urdf::Joint::PRISMATIC) {
      unit = UrdfJointLimitUnit::kMeters;
    }

    out_limits->push_back(JointLimitSpec{lower, upper, unit});
  }

  return true;
}

}  // namespace canopen_hw
