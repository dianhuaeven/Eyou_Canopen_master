#include "canopen_hw/axis_driver.hpp"

#include <system_error>

namespace canopen_hw {

AxisDriver::AxisDriver(lely::canopen::BasicMaster& master, uint8_t node_id,
                       std::size_t axis_index, SharedState* shared_state)
    : lely::canopen::BasicDriver(master, node_id),
      axis_index_(axis_index),
      shared_state_(shared_state) {
  state_machine_.set_target_mode(kMode_CSP);
}

void AxisDriver::SetRosTargetPosition(int32_t target_position) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_ros_target(target_position);
}

void AxisDriver::InjectFeedback(int32_t actual_position, int32_t actual_velocity,
                                int16_t actual_torque, uint16_t statusword,
                                int8_t mode_display) {
  std::lock_guard<std::mutex> lk(mtx_);

  feedback_cache_.actual_position = actual_position;
  feedback_cache_.actual_velocity = actual_velocity;
  feedback_cache_.actual_torque = actual_torque;
  feedback_cache_.statusword = statusword;
  feedback_cache_.mode_display = mode_display;

  // 这里是单轴状态机核心入口:
  // 每次收到一帧同步反馈，都用最新 statusword/mode 推进一步。
  state_machine_.Update(statusword, mode_display, actual_position);

  feedback_cache_.state = state_machine_.state();
  feedback_cache_.is_operational = state_machine_.is_operational();
  feedback_cache_.is_fault = state_machine_.is_fault();

  PublishSnapshot();
}

void AxisDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept {
  (void)idx;
  (void)subidx;
  // RPDO 触发后读取关键反馈字段并推进状态机。
  std::error_code ec;
  const auto statusword = rpdo_mapped[0x6041][0].Read<uint16_t>(ec);
  if (ec) {
    return;
  }
  const auto actual_position = rpdo_mapped[0x6064][0].Read<int32_t>(ec);
  if (ec) {
    return;
  }
  const auto mode_display = rpdo_mapped[0x6061][0].Read<int8_t>(ec);
  if (ec) {
    return;
  }
  const auto actual_velocity = rpdo_mapped[0x606C][0].Read<int32_t>(ec);
  if (ec) {
    return;
  }
  const auto actual_torque = rpdo_mapped[0x6077][0].Read<int16_t>(ec);
  if (ec) {
    return;
  }

  InjectFeedback(actual_position, actual_velocity, actual_torque, statusword,
                 mode_display);
}

void AxisDriver::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept {
  (void)eec;
  (void)er;
  (void)msef;
  // TODO: 增加错误码映射与日志记录。
}

void AxisDriver::OnHeartbeat(bool occurred) noexcept {
  (void)occurred;
  // TODO: 心跳超时/恢复状态上报到 SharedState。
}

void AxisDriver::OnBoot(lely::canopen::NmtState st, char es,
                        const std::string& what) noexcept {
  (void)st;
  (void)es;
  (void)what;
  // TODO: 上线后触发状态机使能流程。
}

void AxisDriver::PublishSnapshot() {
  if (!shared_state_) {
    return;
  }

  // 将状态机过滤后的目标位置同步回命令面，后续 write PDO 时直接读取该值。
  AxisCommand cmd;
  cmd.target_position = state_machine_.safe_target();

  shared_state_->UpdateFeedback(axis_index_, feedback_cache_);
  shared_state_->UpdateCommand(axis_index_, cmd);
}

}  // namespace canopen_hw
