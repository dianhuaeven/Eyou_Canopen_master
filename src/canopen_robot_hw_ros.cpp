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
      active_mode_(joint_names.size(), kMode_CSP),
      cmd_ready_(joint_names.size(), false),
      cmd_ready_guard_(joint_names.size(), cmd_ready_guard_frames_),
      arm_epoch_cache_(joint_names.size(), 0u) {
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

  const bool now_all_operational = hw_->all_operational();
  const bool now_all_axes_halted_by_fault = hw_->all_axes_halted_by_fault();
  const bool all_operational_rising =
      (!prev_all_operational_ && now_all_operational);
  const bool fault_halt_rising =
      (!prev_all_axes_halted_by_fault_ && now_all_axes_halted_by_fault);
  const bool fault_halt_falling =
      (prev_all_axes_halted_by_fault_ && !now_all_axes_halted_by_fault);
  const bool need_resync =
      all_operational_rising || fault_halt_rising || fault_halt_falling;

  for (std::size_t i = 0; i < pos_.size(); ++i) {
    pos_[i] = hw_->joint_position(i);
    vel_[i] = hw_->joint_velocity(i);
    eff_[i] = hw_->joint_effort(i);
    arm_epoch_cache_[i] = hw_->arm_epoch(i);

    if (need_resync) {
      // 触发重同步时，先把命令缓冲对齐到当前反馈，再进入 guard。
      pos_cmd_[i] = pos_[i];
      cmd_ready_[i] = false;
      cmd_ready_guard_[i] = cmd_ready_guard_frames_;
    }

    if (now_all_axes_halted_by_fault) {
      cmd_ready_[i] = false;
      continue;
    }

    if (!cmd_ready_[i] && cmd_ready_guard_[i] > 0) {
      --cmd_ready_guard_[i];
    }
    if (!cmd_ready_[i] && cmd_ready_guard_[i] == 0) {
      cmd_ready_[i] = true;
    }
  }

  prev_all_operational_ = now_all_operational;
  prev_all_axes_halted_by_fault_ = now_all_axes_halted_by_fault;
}

void CanopenRobotHwRos::write(const ros::Time& /*time*/,
                               const ros::Duration& /*period*/) {
  const bool fault_halted = hw_->all_axes_halted_by_fault();
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

    const bool ready =
        cmd_ready_[i] && !fault_halted && (arm_epoch_cache_[i] != 0u);
    hw_->SetCommandReady(i, ready);
    hw_->SetCommandEpoch(i, arm_epoch_cache_[i]);
  }
  hw_->WriteToSharedState();
}

}  // namespace canopen_hw
