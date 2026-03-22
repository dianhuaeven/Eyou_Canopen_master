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
// 对外提供 Configure / InitMotors / Halt / Recover / Shutdown 等操作。
class LifecycleManager {
 public:
  LifecycleManager();

  // 仅加载配置并构造对象，不启动主站。
  // Unconfigured -> Configured。
  bool Configure(const CanopenMasterConfig& config);

  // 在已配置状态下执行首次电机初始化（启动主站）。
  // Configured -> Active。
  bool InitMotors();

  // 兼容接口：加载配置并立即启动主站。
  // 保持原有语义：失败后回到 Unconfigured。
  bool Init(const std::string& dcf_path, const std::string& joints_path);
  bool Init(const CanopenMasterConfig& config);

  // 停止运动（全轴 disable），保持配置与对象。
  // Active -> Configured。
  bool Halt();

  // 仅允许在“曾经初始化成功过”的 Configured 状态恢复。
  // Configured -> Active。
  bool Recover();

  // 优雅关闭主站并释放对象。
  // 任意状态 -> Unconfigured。
  bool Shutdown();

  LifecycleState state() const { return state_; }
  bool ever_initialized() const { return ever_initialized_; }

  // 访问内部组件（供上层集成使用）。
  CanopenMaster* master() { return master_.get(); }
  CanopenRobotHw* robot_hw() { return robot_hw_.get(); }
  SharedState* shared_state() { return shared_state_.get(); }

 private:
  LifecycleState state_ = LifecycleState::Unconfigured;
  bool ever_initialized_ = false;
  CanopenMasterConfig config_;
  std::unique_ptr<SharedState> shared_state_;
  std::unique_ptr<CanopenMaster> master_;
  std::unique_ptr<CanopenRobotHw> robot_hw_;
};

}  // namespace canopen_hw
