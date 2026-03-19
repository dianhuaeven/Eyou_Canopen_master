#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

void SharedState::UpdateFeedback(std::size_t axis_index,
                                 const AxisFeedback& feedback) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_[axis_index] = feedback;
  }
  // 在锁外通知，避免被唤醒线程立即阻塞在 mtx_ 上。
  state_cv_.notify_all();
}

void SharedState::RecomputeAllOperational() {
  std::lock_guard<std::mutex> lk(mtx_);
  bool all_ok = true;
  for (std::size_t i = 0; i < active_axis_count_; ++i) {
    const auto& axis_feedback = feedback_[i];
    if (!axis_feedback.is_operational || axis_feedback.is_fault) {
      all_ok = false;
      break;
    }
  }
  all_operational_ = all_ok;
}

void SharedState::SetActiveAxisCount(std::size_t count) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (count == 0) {
    active_axis_count_ = 1;
  } else if (count > kAxisCount) {
    active_axis_count_ = kAxisCount;
  } else {
    active_axis_count_ = count;
  }
}

void SharedState::UpdateCommand(std::size_t axis_index,
                                const AxisCommand& command) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  commands_[axis_index] = command;
}

void SharedState::UpdateSafeCommand(std::size_t axis_index,
                                    const AxisSafeCommand& safe_command) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  safe_commands_[axis_index] = safe_command;
}

SharedSnapshot SharedState::Snapshot() const {
  std::lock_guard<std::mutex> lk(mtx_);
  SharedSnapshot s;
  s.feedback = feedback_;
  s.commands = commands_;
  s.safe_commands = safe_commands_;
  s.all_operational = all_operational_;
  return s;
}

bool SharedState::IsValidAxis(std::size_t axis_index) const {
  return axis_index < kAxisCount;
}

bool SharedState::WaitForStateChange(
    std::chrono::steady_clock::time_point deadline) {
  std::unique_lock<std::mutex> lk(mtx_);
  return state_cv_.wait_until(lk, deadline) == std::cv_status::no_timeout;
}

}  // namespace canopen_hw
