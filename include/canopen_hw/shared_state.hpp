#pragma once

#include <array>
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
};

// 线程间共享的单轴命令(由 ROS 线程写入)。
struct AxisCommand {
  int32_t target_position = 0;
};

// 给调用者返回的快照结构:
// - read() 一次锁内拷贝即可拿到 6 轴一致视图
// - 调用方后续使用不需要持锁
struct SharedSnapshot {
  std::array<AxisFeedback, 6> feedback{};
  std::array<AxisCommand, 6> commands{};
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

  // 由 Lely 线程更新全局“可控”标志。
  void SetAllOperational(bool value);

  // 由 Lely 线程在每个 SYNC/RPDO 更新后调用，汇总全轴状态。
  void RecomputeAllOperational();

  // 任意线程: 获取完整快照。
  SharedSnapshot Snapshot() const;

 private:
  bool IsValidAxis(std::size_t axis_index) const;

  mutable std::mutex mtx_;
  std::array<AxisFeedback, kAxisCount> feedback_{};
  std::array<AxisCommand, kAxisCount> commands_{};
  bool all_operational_ = false;
};

}  // namespace canopen_hw
