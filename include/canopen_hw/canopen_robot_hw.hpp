#pragma once

#include <array>
#include <cstddef>

#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// ROS 硬件层骨架:
// - 当前阶段不直接依赖 ROS 头文件，先把 read/write 数据流封装稳定
// - 后续 commit 再替换为 hardware_interface::RobotHW 继承实现
class CanopenRobotHw {
 public:
  static constexpr std::size_t kAxisCount = SharedState::kAxisCount;

  explicit CanopenRobotHw(SharedState* shared_state);

  // 对应 RobotHW::read():
  // 从 SharedState 拉取反馈并更新本地关节状态缓存。
  void ReadFromSharedState();

  // 对应 RobotHW::write():
  // 当 all_operational=true 时将本地命令写入 SharedState。
  // 若系统不满足可运行条件，则不下发新命令。
  void WriteToSharedState();

  // 测试/上层适配接口: 设置某轴目标位置(单位: rad)。
  void SetJointCommand(std::size_t axis_index, double pos_rad);

  // 测试/上层适配接口: 读取关节状态缓存。
  double joint_position(std::size_t axis_index) const;
  double joint_velocity(std::size_t axis_index) const;
  double joint_effort(std::size_t axis_index) const;

  // 当前控制可运行标志(来自 SharedState::all_operational)。
  bool all_operational() const { return all_operational_; }

 private:
  static bool IsValidAxis(std::size_t axis_index);

  // 单位换算(临时实现):
  // - 先按每转计数固定值进行换算
  // - 后续改为每轴参数化(从 joints.yaml 读取)
  static double TicksToRad(int32_t ticks);
  static int32_t RadToTicks(double rad);
  static double TicksPerSecToRadPerSec(int32_t ticks_per_sec);
  static double TorquePermilleToNm(int16_t permille);

  SharedState* shared_state_ = nullptr;  // 非拥有指针。

  std::array<double, kAxisCount> joint_pos_{};
  std::array<double, kAxisCount> joint_vel_{};
  std::array<double, kAxisCount> joint_eff_{};
  std::array<double, kAxisCount> joint_cmd_{};

  bool all_operational_ = false;
};

}  // namespace canopen_hw
