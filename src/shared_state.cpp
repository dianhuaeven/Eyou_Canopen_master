#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

void SharedState::UpdateFeedback(std::size_t axis_index,
                                 const AxisFeedback& feedback) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  feedback_[axis_index] = feedback;
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

SharedSnapshot SharedState::Snapshot() const {
  std::lock_guard<std::mutex> lk(mtx_);
  SharedSnapshot s;
  s.feedback = feedback_;
  s.commands = commands_;
  s.all_operational = all_operational_;
  return s;
}

bool SharedState::IsValidAxis(std::size_t axis_index) const {
  return axis_index < kAxisCount;
}

}  // namespace canopen_hw
