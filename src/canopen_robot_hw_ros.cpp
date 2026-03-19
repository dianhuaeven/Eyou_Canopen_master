#include "canopen_hw/canopen_robot_hw_ros.hpp"

#include <cassert>

namespace canopen_hw {

CanopenRobotHwRos::CanopenRobotHwRos(
    CanopenRobotHw* hw, const std::vector<std::string>& joint_names)
    : hw_(hw),
      pos_(joint_names.size(), 0.0),
      vel_(joint_names.size(), 0.0),
      eff_(joint_names.size(), 0.0),
      cmd_(joint_names.size(), 0.0) {
  assert(hw_ != nullptr);
  assert(joint_names.size() == hw_->axis_count());

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    // 注册关节状态 handle（位置/速度/力矩反馈）。
    hardware_interface::JointStateHandle state_handle(
        joint_names[i], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_iface_.registerHandle(state_handle);

    // 注册位置命令 handle（CSP 模式对应 PositionJointInterface）。
    hardware_interface::JointHandle cmd_handle(
        jnt_state_iface_.getHandle(joint_names[i]), &cmd_[i]);
    pos_cmd_iface_.registerHandle(cmd_handle);
  }

  registerInterface(&jnt_state_iface_);
  registerInterface(&pos_cmd_iface_);
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
  for (std::size_t i = 0; i < cmd_.size(); ++i) {
    hw_->SetJointCommand(i, cmd_[i]);
  }
  hw_->WriteToSharedState();
}

}  // namespace canopen_hw
