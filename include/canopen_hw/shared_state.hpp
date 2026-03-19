#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <vector>

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
// - read() 一次锁内拷贝即可拿到全轴一致视图
// - 调用方后续使用不需要持锁
struct SharedSnapshot {
  std::vector<AxisFeedback> feedback;
  std::vector<AxisCommand> commands;
  std::vector<AxisSafeCommand> safe_commands;
  bool all_operational = false;
};

class SharedState {
 public:
  // 安全上限：构造时 axis_count 不得超过此值。
  static constexpr std::size_t kMaxAxisCount = 16;

  // 构造时确定轴数，内部 vector 一次性分配，运行期大小不变。
  // axis_count 会被限制在 [1, kMaxAxisCount]。
  explicit SharedState(std::size_t axis_count = 6);

  // 轴数（构造后不变）。
  std::size_t axis_count() const { return axis_count_; }

  // Lely 线程: 更新某轴反馈信息。
  void UpdateFeedback(std::size_t axis_index, const AxisFeedback& feedback);

  // ROS 线程: 更新某轴目标位置命令。
  void UpdateCommand(std::size_t axis_index, const AxisCommand& command);

  // 任意线程: 读取某轴目标位置命令。越界返回 false。
  bool GetCommand(std::size_t axis_index, AxisCommand* out) const;

  // Lely 线程: 更新某轴状态机过滤后的安全目标位置。
  void UpdateSafeCommand(std::size_t axis_index,
                         const AxisSafeCommand& safe_command);

  // 由 Lely 线程在每个 SYNC/RPDO 更新后调用，汇总全轴状态。
  void RecomputeAllOperational();

  // 任意线程: 获取完整快照。
  SharedSnapshot Snapshot() const;

  // 阻塞等待反馈状态发生变化（由 UpdateFeedback 通知唤醒）。
  // 返回 true 表示在 deadline 前被唤醒，false 表示超时。
  bool WaitForStateChange(std::chrono::steady_clock::time_point deadline);

 private:
  bool IsValidAxis(std::size_t axis_index) const;

  const std::size_t axis_count_;

  mutable std::mutex mtx_;
  std::condition_variable state_cv_;  // 由 UpdateFeedback() 通知。
  std::vector<AxisFeedback> feedback_;
  std::vector<AxisCommand> commands_;
  std::vector<AxisSafeCommand> safe_commands_;
  bool all_operational_ = false;
  uint64_t state_change_seq_ = 0;
};

}  // namespace canopen_hw
