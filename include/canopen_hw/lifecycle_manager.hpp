#pragma once

#include <memory>
#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

enum class LifecycleState {
  Unconfigured,
  Configured,
  Active,
  ShuttingDown,
};

// 生命周期管理器:
// 封装 SharedState / CanopenMaster / CanopenRobotHw 的构造与状态跃迁，
// 对外提供 Init / Halt / Recover / Shutdown 四个操作。
class LifecycleManager {
 public:
  LifecycleManager();

  // 加载配置并启动主站。
  // Unconfigured → Active。
  // 返回 false 表示启动失败，状态回到 Unconfigured。
  bool Init(const std::string& dcf_path, const std::string& joints_path);

  // 使用已解析的配置启动（供测试或外部已加载配置的场景）。
  bool Init(const CanopenMasterConfig& config);

  // 停止运动（全轴 disable），保持总线连接。
  // Active → Configured。
  bool Halt();

  // 从 Configured 或故障状态恢复到 Active。
  // 清除故障计数，重新使能所有轴。
  bool Recover();

  // 优雅关闭主站和总线。
  // 任意状态 → Unconfigured。
  bool Shutdown();

  LifecycleState state() const { return state_; }

  // 访问内部组件（供上层集成使用）。
  CanopenMaster* master() { return master_.get(); }
  CanopenRobotHw* robot_hw() { return robot_hw_.get(); }
  SharedState* shared_state() { return shared_state_.get(); }

 private:
  LifecycleState state_ = LifecycleState::Unconfigured;
  CanopenMasterConfig config_;
  std::unique_ptr<SharedState> shared_state_;
  std::unique_ptr<CanopenMaster> master_;
  std::unique_ptr<CanopenRobotHw> robot_hw_;
};

}  // namespace canopen_hw
