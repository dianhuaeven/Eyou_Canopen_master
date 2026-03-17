#include "canopen_hw/canopen_robot_hw.hpp"

#include <cmath>
#include <cstdint>

namespace canopen_hw {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCountsPerRev = 5308416.0;  // PH11 默认值, 后续改为每轴参数
constexpr double kRatedTorqueNm = 6.0;       // 临时值, 后续从配置读取

}  // namespace

CanopenRobotHw::CanopenRobotHw(SharedState* shared_state)
    : shared_state_(shared_state) {}

void CanopenRobotHw::ReadFromSharedState() {
  if (!shared_state_) {
    return;
  }

  const SharedSnapshot snap = shared_state_->Snapshot();
  all_operational_ = snap.all_operational;

  for (std::size_t i = 0; i < kAxisCount; ++i) {
    joint_pos_[i] = TicksToRad(snap.feedback[i].actual_position);
    joint_vel_[i] = TicksPerSecToRadPerSec(snap.feedback[i].actual_velocity);
    joint_eff_[i] = TorquePermilleToNm(snap.feedback[i].actual_torque);
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

  for (std::size_t i = 0; i < kAxisCount; ++i) {
    AxisCommand cmd;
    cmd.target_position = RadToTicks(joint_cmd_[i]);
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

bool CanopenRobotHw::IsValidAxis(std::size_t axis_index) {
  return axis_index < kAxisCount;
}

double CanopenRobotHw::TicksToRad(int32_t ticks) {
  return static_cast<double>(ticks) * (2.0 * kPi / kCountsPerRev);
}

int32_t CanopenRobotHw::RadToTicks(double rad) {
  return static_cast<int32_t>(std::llround(rad * kCountsPerRev / (2.0 * kPi)));
}

double CanopenRobotHw::TicksPerSecToRadPerSec(int32_t ticks_per_sec) {
  return static_cast<double>(ticks_per_sec) * (2.0 * kPi / kCountsPerRev);
}

double CanopenRobotHw::TorquePermilleToNm(int16_t permille) {
  return static_cast<double>(permille) / 1000.0 * kRatedTorqueNm;
}

}  // namespace canopen_hw
