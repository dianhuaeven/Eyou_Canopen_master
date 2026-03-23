#include "canopen_hw/axis_logic.hpp"

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

AxisLogic::AxisLogic(std::size_t axis_index, BusIO* bus_io,
                     SharedState* shared_state)
    : axis_index_(axis_index),
      bus_io_(bus_io),
      shared_state_(shared_state) {
  state_machine_.set_target_mode(kMode_CSP);
}

void AxisLogic::ProcessRpdo(uint16_t statusword, int32_t actual_position,
                            int32_t actual_velocity, int16_t actual_torque,
                            int8_t mode_display) {
  int32_t safe_target_ticks = 0;
  int32_t safe_target_velocity = 0;
  int16_t safe_target_torque = 0;
  int8_t safe_mode = kMode_CSP;
  uint16_t controlword = 0;
  bool fault_detected = false;

  {
    std::lock_guard<std::mutex> lk(mtx_);

    if (shared_state_) {
      const bool global_fault = shared_state_->GetGlobalFault();
      state_machine_.set_global_fault(global_fault);
      if (global_fault) {
        state_machine_.set_forced_halt_by_fault(true);
      }
    }

    feedback_cache_.actual_position = actual_position;
    feedback_cache_.actual_velocity = actual_velocity;
    feedback_cache_.actual_torque = actual_torque;
    feedback_cache_.statusword = statusword;
    feedback_cache_.mode_display = mode_display;

    state_machine_.Update(statusword, mode_display, actual_position);
    controlword = state_machine_.controlword();

    feedback_cache_.state = state_machine_.state();
    feedback_cache_.is_operational = state_machine_.is_operational();
    feedback_cache_.is_fault = state_machine_.is_fault();
    feedback_cache_.arm_epoch = state_machine_.arm_epoch();
    fault_detected = feedback_cache_.is_fault;
    health_.fault_reset_attempts.store(
        static_cast<uint32_t>(state_machine_.fault_reset_count()),
        std::memory_order_relaxed);

    safe_target_ticks = state_machine_.safe_target();
    safe_target_velocity = state_machine_.safe_target_velocity();
    safe_target_torque = state_machine_.safe_target_torque();
    safe_mode = state_machine_.safe_mode_of_operation();
  }

  PublishSnapshot();

  if (shared_state_) {
    if (fault_detected) {
      shared_state_->SetGlobalFault(true);
      shared_state_->SetAllAxesHaltedByFault(true);
    }
    shared_state_->RecomputeAllOperational();
  }

  if (bus_io_) {
    bus_io_->WriteControlword(controlword);
    bus_io_->WriteModeOfOperation(safe_mode);
    bus_io_->WriteTargetPosition(safe_target_ticks);
    bus_io_->WriteTargetVelocity(safe_target_velocity);
    bus_io_->WriteTargetTorque(safe_target_torque);
  }
}

void AxisLogic::ProcessEmcy(uint16_t eec, uint8_t er) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_cache_.last_emcy_eec = eec;
  }
  health_.emcy_count.fetch_add(1, std::memory_order_relaxed);
  CANOPEN_LOG_ERROR("axis={} EMCY eec=0x{:04x} er=0x{:02x}",
                    axis_index_, static_cast<unsigned int>(eec),
                    static_cast<unsigned int>(er));
  PublishSnapshot();
}

void AxisLogic::ProcessHeartbeat(bool lost) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_cache_.heartbeat_lost = lost;
    if (lost) {
      // 心跳丢失后撤销使能请求，避免复电重连后沿用旧请求自动再使能。
      state_machine_.request_disable();
      feedback_cache_.is_fault = true;
      feedback_cache_.is_operational = false;
    } else {
      feedback_cache_.is_fault = state_machine_.is_fault();
      feedback_cache_.is_operational =
          state_machine_.is_operational() && (feedback_cache_.is_fault == false);
    }
  }
  if (lost) {
    health_.heartbeat_lost.fetch_add(1, std::memory_order_relaxed);
    CANOPEN_LOG_WARN("axis={}: heartbeat lost", axis_index_);
  } else {
    health_.heartbeat_recovered.fetch_add(1, std::memory_order_relaxed);
    CANOPEN_LOG_INFO("axis={}: heartbeat recovered", axis_index_);
  }
  PublishSnapshot();
  if (shared_state_) {
    if (lost) {
      shared_state_->SetGlobalFault(true);
      shared_state_->SetAllAxesHaltedByFault(true);
    }
    shared_state_->RecomputeAllOperational();
  }
}

void AxisLogic::Configure(int32_t position_lock_threshold,
                          int max_fault_resets,
                          int fault_reset_hold_cycles) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_position_lock_threshold(position_lock_threshold);
  state_machine_.set_fault_reset_policy(fault_reset_hold_cycles, 100,
                                        max_fault_resets);
}

void AxisLogic::SetRosTarget(int32_t target_position) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_ros_target(target_position);
}

void AxisLogic::SetRosTargetVelocity(int32_t target_velocity) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_ros_target_velocity(target_velocity);
}

void AxisLogic::SetRosTargetTorque(int16_t target_torque) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_ros_target_torque(target_torque);
}

void AxisLogic::SetTargetMode(int8_t mode) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_target_mode(mode);
}

void AxisLogic::SetExternalCommand(const AxisCommand& command) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_target_mode(command.mode_of_operation);
  state_machine_.SetExternalPositionCommand(
      command.target_position, command.valid, command.arm_epoch);
  state_machine_.SetExternalVelocityCommand(command.target_velocity);
  state_machine_.SetExternalTorqueCommand(command.target_torque);
}

void AxisLogic::SetGlobalFault(bool global_fault) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_global_fault(global_fault);
  if (global_fault) {
    state_machine_.set_forced_halt_by_fault(true);
  }
}

void AxisLogic::RequestEnable() {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.request_enable();
}

void AxisLogic::RequestDisable() {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    state_machine_.request_disable();
    // 关机路径期望“去使能请求”立即可见，避免等待下一帧 RPDO 才回落。
    feedback_cache_.is_operational = false;
  }
  PublishSnapshot();
  if (shared_state_) {
    shared_state_->RecomputeAllOperational();
  }
}

void AxisLogic::RequestHalt() {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.request_halt();
}

void AxisLogic::RequestResume() {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.request_resume();
}

void AxisLogic::ResetFault() {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.ResetFaultCounter();
  health_.fault_reset_attempts.store(0, std::memory_order_relaxed);
  state_machine_.request_enable();
}

CiA402State AxisLogic::feedback_state() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return feedback_cache_.state;
}

void AxisLogic::PublishSnapshot() {
  if (shared_state_ == nullptr) return;

  AxisFeedback feedback_snapshot;
  AxisSafeCommand safe_cmd;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_snapshot = feedback_cache_;
    safe_cmd.safe_target_position = state_machine_.safe_target();
    safe_cmd.safe_target_velocity = state_machine_.safe_target_velocity();
    safe_cmd.safe_target_torque = state_machine_.safe_target_torque();
    safe_cmd.safe_mode_of_operation = state_machine_.safe_mode_of_operation();
  }

  shared_state_->UpdateFeedback(axis_index_, feedback_snapshot);
  shared_state_->UpdateSafeCommand(axis_index_, safe_cmd);
}

}  // namespace canopen_hw
