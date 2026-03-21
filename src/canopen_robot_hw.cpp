#include "canopen_hw/canopen_robot_hw.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>

#include "canopen_hw/canopen_master.hpp"

namespace canopen_hw {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

CanopenRobotHw::CanopenRobotHw(SharedState* shared_state)
    : shared_state_(shared_state),
      axis_count_(shared_state ? shared_state->axis_count() : 0),
      joint_pos_(axis_count_, 0.0),
      joint_vel_(axis_count_, 0.0),
      joint_eff_(axis_count_, 0.0),
      joint_cmd_(axis_count_, 0.0),
      joint_vel_cmd_(axis_count_, 0.0),
      joint_torque_cmd_(axis_count_, 0.0),
      joint_mode_(axis_count_, kMode_CSP),
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

  for (std::size_t i = 0; i < axis_count_; ++i) {
    AxisCommand cmd;
    cmd.target_position = RadToTicks(i, joint_cmd_[i]);
    // 允许在未 fully-operational 阶段下发位置目标，
    // 让 CiA402 位置锁定逻辑有机会收敛到 operational；
    // 同时保持速度/力矩为 0，避免在未就绪阶段透传动态命令。
    if (all_operational_) {
      cmd.target_velocity = RadPerSecToTicksPerSec(i, joint_vel_cmd_[i]);
      cmd.target_torque = NmToTorquePermille(i, joint_torque_cmd_[i]);
    } else {
      cmd.target_velocity = 0;
      cmd.target_torque = 0;
    }
    cmd.mode_of_operation = joint_mode_[i];
    shared_state_->UpdateCommand(i, cmd);
  }
}

void CanopenRobotHw::SetJointCommand(std::size_t axis_index, double pos_rad) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_cmd_[axis_index] = pos_rad;
}

void CanopenRobotHw::SetJointVelocityCommand(std::size_t axis_index,
                                              double vel_rad_s) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_vel_cmd_[axis_index] = vel_rad_s;
}

void CanopenRobotHw::SetJointTorqueCommand(std::size_t axis_index,
                                            double torque_nm) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_torque_cmd_[axis_index] = torque_nm;
}

void CanopenRobotHw::SetJointMode(std::size_t axis_index, int8_t mode) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_mode_[axis_index] = mode;
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

void CanopenRobotHw::ApplyConfig(const CanopenMasterConfig& config) {
  const std::size_t n = std::min(axis_count_, config.joints.size());
  for (std::size_t i = 0; i < n; ++i) {
    AxisConversion conv;
    conv.counts_per_rev = config.joints[i].counts_per_rev;
    conv.rated_torque_nm = config.joints[i].rated_torque_nm;
    conv.velocity_scale = config.joints[i].velocity_scale;
    conv.torque_scale = config.joints[i].torque_scale;
    ConfigureAxisConversion(i, conv);
  }
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

int32_t CanopenRobotHw::RadPerSecToTicksPerSec(std::size_t axis_index,
                                                double rad_per_sec) const {
  const double counts_per_rev =
      std::max(1.0, axis_conv_[axis_index].counts_per_rev);
  const double scale = std::max(1e-9, axis_conv_[axis_index].velocity_scale);
  const long long raw =
      std::llround(rad_per_sec / scale * counts_per_rev / (2.0 * kPi));
  return static_cast<int32_t>(std::clamp(raw,
      static_cast<long long>(INT32_MIN), static_cast<long long>(INT32_MAX)));
}

double CanopenRobotHw::TorquePermilleToNm(std::size_t axis_index,
                                          int16_t permille) const {
  const double nm = static_cast<double>(permille) / 1000.0 *
                    axis_conv_[axis_index].rated_torque_nm;
  return nm * axis_conv_[axis_index].torque_scale;
}

int16_t CanopenRobotHw::NmToTorquePermille(std::size_t axis_index,
                                            double nm) const {
  const double rated = std::max(1e-9, axis_conv_[axis_index].rated_torque_nm);
  const double scale = std::max(1e-9, axis_conv_[axis_index].torque_scale);
  const long long raw = std::llround(nm / scale / rated * 1000.0);
  return static_cast<int16_t>(std::clamp(raw,
      static_cast<long long>(INT16_MIN), static_cast<long long>(INT16_MAX)));
}

}  // namespace canopen_hw
