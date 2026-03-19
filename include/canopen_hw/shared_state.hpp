#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

// 线程间共享的单轴反馈快照(由 Lely 线程刷新)。
struct AxisFeedback {
  int32_t actual_position = 0;
  int32_t actual_velocity = 0;
  int16_t actual_torque = 0;
  uint16_t statusword = 0;
  int8_t mode_display = 0;
  CiA402State state = CiA402State::NotReadyToSwitchOn;
  bool is_operational = false;
  bool is_fault = false;
  bool heartbeat_lost = false;
  uint16_t last_emcy_eec = 0;
};

// 线程间共享的单轴命令(由 ROS 线程写入)。
struct AxisCommand {
  int32_t target_position = 0;
};

// 状态机过滤后的安全目标位置(由 Lely 线程写入)。
// 与 AxisCommand 分离，避免 Lely 线程覆盖 ROS 线程写入的用户期望位置。
struct AxisSafeCommand {
  int32_t safe_target_position = 0;
};

// 给调用者返回的快照结构:
// - read() 一次锁内拷贝即可拿到 6 轴一致视图
// - 调用方后续使用不需要持锁
struct SharedSnapshot {
  std::array<AxisFeedback, 6> feedback{};
  std::array<AxisCommand, 6> commands{};
  std::array<AxisSafeCommand, 6> safe_commands{};
  bool all_operational = false;
};

class SharedState {
 public:
  static constexpr std::size_t kAxisCount = 6;

  SharedState() = default;

  // Lely 线程: 更新某轴反馈信息。
  void UpdateFeedback(std::size_t axis_index, const AxisFeedback& feedback);

  // ROS 线程: 更新某轴目标位置命令。
  void UpdateCommand(std::size_t axis_index, const AxisCommand& command);

  // Lely 线程: 更新某轴状态机过滤后的安全目标位置。
  void UpdateSafeCommand(std::size_t axis_index,
                         const AxisSafeCommand& safe_command);

  // 由 Lely 线程在每个 SYNC/RPDO 更新后调用，汇总全轴状态。
  void RecomputeAllOperational();

  // 配置有效轴数（用于 all_operational 汇总边界）。
  // count 会被限制在 [1, kAxisCount]。
  void SetActiveAxisCount(std::size_t count);

  // 任意线程: 获取完整快照。
  SharedSnapshot Snapshot() const;

  // 阻塞等待反馈状态发生变化（由 UpdateFeedback 通知唤醒）。
  // 返回 true 表示在 deadline 前被唤醒，false 表示超时。
  // 用于替代 WaitForAllState 中的 sleep_for busy-wait。
  bool WaitForStateChange(std::chrono::steady_clock::time_point deadline);

 private:
  bool IsValidAxis(std::size_t axis_index) const;

  mutable std::mutex mtx_;
  std::condition_variable state_cv_;  // 由 UpdateFeedback() 通知。
  std::array<AxisFeedback, kAxisCount> feedback_{};
  std::array<AxisCommand, kAxisCount> commands_{};
  std::array<AxisSafeCommand, kAxisCount> safe_commands_{};
  bool all_operational_ = false;
  std::size_t active_axis_count_ = kAxisCount;
};

}  // namespace canopen_hw
