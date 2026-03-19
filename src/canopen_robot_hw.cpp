#include "canopen_hw/canopen_robot_hw.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace canopen_hw {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

CanopenRobotHw::CanopenRobotHw(SharedState* shared_state)
    : shared_state_(shared_state),
      axis_count_(shared_state ? shared_state->axis_count() : 6),
      joint_pos_(axis_count_, 0.0),
      joint_vel_(axis_count_, 0.0),
      joint_eff_(axis_count_, 0.0),
      joint_cmd_(axis_count_, 0.0),
      axis_conv_(axis_count_) {}

void CanopenRobotHw::ReadFromSharedState() {
  if (!shared_state_) {
    return;
  }

  const SharedSnapshot snap = shared_state_->Snapshot();
  all_operational_ = snap.all_operational;

  for (std::size_t i = 0; i < axis_count_; ++i) {
    joint_pos_[i] = TicksToRad(i, snap.feedback[i].actual_position);
    joint_vel_[i] = TicksPerSecToRadPerSec(i, snap.feedback[i].actual_velocity);
    joint_eff_[i] = TorquePermilleToNm(i, snap.feedback[i].actual_torque);
  }
}

void CanopenRobotHw::WriteToSharedState() {
  if (!shared_state_) {
    return;
  }

  // 安全策略: 只有全轴都 operational 才向底层写入新目标。
  if (!all_operational_) {
    return;
  }

  for (std::size_t i = 0; i < axis_count_; ++i) {
    AxisCommand cmd;
    cmd.target_position = RadToTicks(i, joint_cmd_[i]);
    shared_state_->UpdateCommand(i, cmd);
  }
}

void CanopenRobotHw::SetJointCommand(std::size_t axis_index, double pos_rad) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_cmd_[axis_index] = pos_rad;
}

double CanopenRobotHw::joint_position(std::size_t axis_index) const {
  return IsValidAxis(axis_index) ? joint_pos_[axis_index] : 0.0;
}

double CanopenRobotHw::joint_velocity(std::size_t axis_index) const {
  return IsValidAxis(axis_index) ? joint_vel_[axis_index] : 0.0;
}

double CanopenRobotHw::joint_effort(std::size_t axis_index) const {
  return IsValidAxis(axis_index) ? joint_eff_[axis_index] : 0.0;
}

bool CanopenRobotHw::IsValidAxis(std::size_t axis_index) const {
  return axis_index < axis_count_;
}

void CanopenRobotHw::ConfigureAxisConversion(
    std::size_t axis_index, const AxisConversion& conversion) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  axis_conv_[axis_index] = conversion;
}

double CanopenRobotHw::TicksToRad(std::size_t axis_index, int32_t ticks) const {
  const double counts_per_rev =
      std::max(1.0, axis_conv_[axis_index].counts_per_rev);
  return static_cast<double>(ticks) * (2.0 * kPi / counts_per_rev);
}

int32_t CanopenRobotHw::RadToTicks(std::size_t axis_index, double rad) const {
  const double counts_per_rev =
      std::max(1.0, axis_conv_[axis_index].counts_per_rev);
  return static_cast<int32_t>(
      std::llround(rad * counts_per_rev / (2.0 * kPi)));
}

double CanopenRobotHw::TicksPerSecToRadPerSec(std::size_t axis_index,
                                              int32_t ticks_per_sec) const {
  return TicksToRad(axis_index, ticks_per_sec) *
         axis_conv_[axis_index].velocity_scale;
}

double CanopenRobotHw::TorquePermilleToNm(std::size_t axis_index,
                                          int16_t permille) const {
  const double nm = static_cast<double>(permille) / 1000.0 *
                    axis_conv_[axis_index].rated_torque_nm;
  return nm * axis_conv_[axis_index].torque_scale;
}

}  // namespace canopen_hw
