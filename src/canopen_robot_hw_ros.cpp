#include "canopen_hw/canopen_robot_hw_ros.hpp"

#include <cassert>

#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

CanopenRobotHwRos::CanopenRobotHwRos(
    CanopenRobotHw* hw, const std::vector<std::string>& joint_names)
    : hw_(hw),
      pos_(joint_names.size(), 0.0),
      vel_(joint_names.size(), 0.0),
      eff_(joint_names.size(), 0.0),
      pos_cmd_(joint_names.size(), 0.0),
      vel_cmd_(joint_names.size(), 0.0),
      active_mode_(joint_names.size(), kMode_CSP) {
  assert(hw_ != nullptr);
  assert(joint_names.size() == hw_->axis_count());

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    hardware_interface::JointStateHandle state_handle(
        joint_names[i], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_iface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(
        jnt_state_iface_.getHandle(joint_names[i]), &pos_cmd_[i]);
    pos_cmd_iface_.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(
        jnt_state_iface_.getHandle(joint_names[i]), &vel_cmd_[i]);
    vel_cmd_iface_.registerHandle(vel_handle);
  }

  registerInterface(&jnt_state_iface_);
  registerInterface(&pos_cmd_iface_);
  registerInterface(&vel_cmd_iface_);
}

void CanopenRobotHwRos::SetMode(std::size_t axis_index, int8_t mode) {
  if (axis_index >= active_mode_.size()) {
    return;
  }
  active_mode_[axis_index] = mode;
  hw_->SetJointMode(axis_index, mode);
}

void CanopenRobotHwRos::read(const ros::Time& /*time*/,
                              const ros::Duration& /*period*/) {
  hw_->ReadFromSharedState();

  for (std::size_t i = 0; i < pos_.size(); ++i) {
    pos_[i] = hw_->joint_position(i);
    vel_[i] = hw_->joint_velocity(i);
    eff_[i] = hw_->joint_effort(i);
  }
}

void CanopenRobotHwRos::write(const ros::Time& /*time*/,
                               const ros::Duration& /*period*/) {
  for (std::size_t i = 0; i < pos_cmd_.size(); ++i) {
    switch (active_mode_[i]) {
      case kMode_CSV:
        hw_->SetJointVelocityCommand(i, vel_cmd_[i]);
        break;
      case kMode_CSP:
      default:
        hw_->SetJointCommand(i, pos_cmd_[i]);
        break;
    }
  }
  hw_->WriteToSharedState();
}

}  // namespace canopen_hw
