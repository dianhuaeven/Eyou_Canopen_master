#include "canopen_hw/lifecycle_manager.hpp"

#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/logging.hpp"

namespace canopen_hw {

LifecycleManager::LifecycleManager() = default;

bool LifecycleManager::Init(const std::string& dcf_path,
                            const std::string& joints_path) {
  if (state_ != LifecycleState::Unconfigured) {
    CANOPEN_LOG_WARN("Init called in state {}, expected Unconfigured",
                     static_cast<int>(state_));
    return false;
  }

  CanopenMasterConfig config;
  config.master_dcf_path = dcf_path;

  std::string error;
  if (!LoadJointsYaml(joints_path, &error, &config)) {
    CANOPEN_LOG_ERROR("Init: load joints.yaml failed: {}", error);
    return false;
  }

  return Init(config);
}

bool LifecycleManager::Init(const CanopenMasterConfig& config) {
  if (state_ != LifecycleState::Unconfigured) {
    CANOPEN_LOG_WARN("Init called in state {}, expected Unconfigured",
                     static_cast<int>(state_));
    return false;
  }

  config_ = config;

  if (config_.axis_count == 0 || config_.axis_count > SharedState::kMaxAxisCount) {
    CANOPEN_LOG_ERROR("Init: invalid axis_count {}", config_.axis_count);
    return false;
  }

  shared_state_ = std::make_unique<SharedState>(config_.axis_count);
  robot_hw_ = std::make_unique<CanopenRobotHw>(shared_state_.get());
  robot_hw_->ApplyConfig(config_);

  master_ = std::make_unique<CanopenMaster>(config_, shared_state_.get());

  if (!master_->Start()) {
    CANOPEN_LOG_ERROR("Init: master start failed");
    master_.reset();
    robot_hw_.reset();
    shared_state_.reset();
    return false;
  }

  state_ = LifecycleState::Active;
  CANOPEN_LOG_INFO("LifecycleManager: Active");
  return true;
}

bool LifecycleManager::Halt() {
  if (state_ != LifecycleState::Active) {
    CANOPEN_LOG_WARN("Halt called in state {}, expected Active",
                     static_cast<int>(state_));
    return false;
  }

  if (master_ && master_->running()) {
    master_->GracefulShutdown();
  }

  state_ = LifecycleState::Configured;
  CANOPEN_LOG_INFO("LifecycleManager: Configured (halted)");
  return true;
}

bool LifecycleManager::Recover() {
  if (state_ != LifecycleState::Configured) {
    CANOPEN_LOG_WARN("Recover called in state {}, expected Configured",
                     static_cast<int>(state_));
    return false;
  }

  // 重新启动主站。Stop + Start 会重建 Lely 事件循��和轴驱动。
  if (master_) {
    master_->Stop();
    if (!master_->Start()) {
      CANOPEN_LOG_ERROR("Recover: master restart failed");
      return false;
    }
  }

  state_ = LifecycleState::Active;
  CANOPEN_LOG_INFO("LifecycleManager: Active (recovered)");
  return true;
}

bool LifecycleManager::Shutdown() {
  if (state_ == LifecycleState::Unconfigured) {
    return true;
  }

  state_ = LifecycleState::ShuttingDown;

  if (master_) {
    master_->Stop();
  }

  master_.reset();
  robot_hw_.reset();
  shared_state_.reset();

  state_ = LifecycleState::Unconfigured;
  CANOPEN_LOG_INFO("LifecycleManager: Unconfigured (shutdown)");
  return true;
}

}  // namespace canopen_hw
