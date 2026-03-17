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
  bool all_ok = true;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto& axis_feedback : feedback_) {
      if (!axis_feedback.is_operational || axis_feedback.is_fault) {
        all_ok = false;
        break;
      }
    }
  }
  SetAllOperational(all_ok);
}

void SharedState::UpdateCommand(std::size_t axis_index,
                                const AxisCommand& command) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  commands_[axis_index] = command;
}

void SharedState::SetAllOperational(bool value) {
  std::lock_guard<std::mutex> lk(mtx_);
  all_operational_ = value;
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
