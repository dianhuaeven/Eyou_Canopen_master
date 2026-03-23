#pragma once

#include <cstdint>

#include "canopen_hw/cia402_state_machine.hpp"

namespace canopen_hw {

// 阶段 3 过渡适配层：先复用现有 CiA402StateMachine 实现，
// 再逐步替换为纯协议翻译逻辑。
class CiA402Protocol {
 public:
  CiA402Protocol();

  void Update(uint16_t statusword, int8_t mode_display, int32_t actual_position) {
    sm_.Update(statusword, mode_display, actual_position);
  }

  uint16_t controlword() const { return sm_.controlword(); }
  CiA402State state() const { return sm_.state(); }
  bool is_operational() const { return sm_.is_operational(); }
  bool is_fault() const { return sm_.is_fault(); }
  int fault_reset_count() const { return sm_.fault_reset_count(); }

  void set_target_mode(int8_t mode) { sm_.set_target_mode(mode); }

  void request_enable() { sm_.request_enable(); }
  void request_disable() { sm_.request_disable(); }
  bool enable_requested() const { return sm_.enable_requested(); }

  void request_halt() { sm_.request_halt(); }
  void request_resume() { sm_.request_resume(); }
  bool halt_requested() const { return sm_.halt_requested(); }

  void SetExternalPositionCommand(int32_t target, bool valid, uint32_t arm_epoch) {
    sm_.SetExternalPositionCommand(target, valid, arm_epoch);
  }
  void SetExternalVelocityCommand(int32_t target) {
    sm_.SetExternalVelocityCommand(target);
  }
  void SetExternalTorqueCommand(int16_t target) {
    sm_.SetExternalTorqueCommand(target);
  }

  void set_ros_target(int32_t target) { sm_.set_ros_target(target); }
  void set_ros_target_velocity(int32_t target) {
    sm_.set_ros_target_velocity(target);
  }
  void set_ros_target_torque(int16_t target) { sm_.set_ros_target_torque(target); }

  void set_global_fault(bool fault) { sm_.set_global_fault(fault); }
  void set_forced_halt_by_fault(bool forced) { sm_.set_forced_halt_by_fault(forced); }
  bool forced_halt_by_fault() const { return sm_.forced_halt_by_fault(); }

  int32_t safe_target() const { return sm_.safe_target(); }
  int32_t safe_target_velocity() const { return sm_.safe_target_velocity(); }
  int16_t safe_target_torque() const { return sm_.safe_target_torque(); }
  int8_t safe_mode_of_operation() const { return sm_.safe_mode_of_operation(); }

  uint32_t arm_epoch() const { return sm_.arm_epoch(); }
  bool is_position_locked() const { return sm_.is_position_locked(); }

  void set_position_lock_threshold(int32_t threshold_counts) {
    sm_.set_position_lock_threshold(threshold_counts);
  }

  void set_max_delta_per_cycle(int32_t delta_counts) {
    sm_.set_max_delta_per_cycle(delta_counts);
  }

  void set_fault_reset_policy(int hold_cycles, int wait_cycles, int max_attempts) {
    sm_.set_fault_reset_policy(hold_cycles, wait_cycles, max_attempts);
  }

  void ResetFaultCounter() { sm_.ResetFaultCounter(); }

 private:
  CiA402StateMachine sm_;
};

}  // namespace canopen_hw
