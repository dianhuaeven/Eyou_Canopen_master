#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include <lely/coapp/driver.hpp>

#include "canopen_hw/cia402_state_machine.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// 单轴驱动骨架:
// - 继承 Lely BasicDriver，挂接节点事件回调
// - 维护本轴 CiA402 状态机
// - 通过 SharedState 与 ROS 线程交换数据
//
// 说明: 当前 commit 仅建立结构和最小行为，PDO 字段读取/写入将放在后续 commit 完成。
class AxisDriver final : public lely::canopen::BasicDriver {
 public:
  AxisDriver(lely::canopen::BasicMaster& master, uint8_t node_id,
             std::size_t axis_index, SharedState* shared_state);

  // ROS 线程每周期调用: 更新上层期望位置(仅写入本地缓存, 不直接触发总线发送)。
  void SetRosTargetPosition(int32_t target_position);

  // 由后续 PDO 解析逻辑调用, 将本周期反馈推进状态机。
  // 这里先提供显式入口, 便于在无硬件场景下做逻辑验证。
  void InjectFeedback(int32_t actual_position, int32_t actual_velocity,
                      int16_t actual_torque, uint16_t statusword,
                      int8_t mode_display);

  // 关机流程辅助接口。
  bool SendControlword(uint16_t controlword);
  bool SendNmtStopAll();
  CiA402State feedback_state() const;

 private:
  // Lely 回调: RPDO/SDO 写入了 RPDO-mapped 对象后触发。
  // 当前骨架阶段仅保留钩子，具体对象读取将在下一阶段接入。
  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

  // Lely 回调: EMCY 上报。
  void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

  // Lely 回调: 心跳超时事件发生/恢复。
  void OnHeartbeat(bool occurred) noexcept override;

  // Lely 回调: 节点 boot 过程完成。
  void OnBoot(lely::canopen::NmtState st, char es,
              const std::string& what) noexcept override;

  // 将状态机输出和反馈打包到 SharedState。
  void PublishSnapshot();

  std::size_t axis_index_ = 0;
  SharedState* shared_state_ = nullptr;  // 非拥有指针，生命周期由 Master 管理。

  // 保护本轴缓存, 避免 ROS 设置目标与 Lely 回调并发冲突。
  mutable std::mutex mtx_;
  CiA402StateMachine state_machine_;

  // 最近一帧反馈缓存(由 Lely 线程写入)。
  AxisFeedback feedback_cache_{};
};

}  // namespace canopen_hw
