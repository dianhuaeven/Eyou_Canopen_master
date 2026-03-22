#include "canopen_hw/lifecycle_manager.hpp"

#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/logging.hpp"

namespace canopen_hw {

LifecycleManager::LifecycleManager() = default;

bool LifecycleManager::Configure(const CanopenMasterConfig& config) {
  if (state_ != LifecycleState::Unconfigured) {
    CANOPEN_LOG_WARN("Configure called in state {}, expected Unconfigured",
                     static_cast<int>(state_));
    return false;
  }

  config_ = config;

  if (config_.axis_count == 0 || config_.axis_count > SharedState::kMaxAxisCount) {
    CANOPEN_LOG_ERROR("Configure: invalid axis_count {}", config_.axis_count);
    return false;
  }

  shared_state_ = std::make_unique<SharedState>(config_.axis_count);
  robot_hw_ = std::make_unique<CanopenRobotHw>(shared_state_.get());
  robot_hw_->ApplyConfig(config_);

  master_ = std::make_unique<CanopenMaster>(config_, shared_state_.get());

  state_ = LifecycleState::Configured;
  ever_initialized_ = false;
  require_init_ = false;
  CANOPEN_LOG_INFO("LifecycleManager: Configured");
  return true;
}

bool LifecycleManager::InitMotors() {
  if (state_ != LifecycleState::Configured) {
    CANOPEN_LOG_WARN("InitMotors called in state {}, expected Configured",
                     static_cast<int>(state_));
    return false;
  }

  if (!master_) {
    CANOPEN_LOG_ERROR("InitMotors: master is null");
    return false;
  }

  if (!master_->Start()) {
    CANOPEN_LOG_ERROR("InitMotors: master start failed");
    return false;
  }

  state_ = LifecycleState::Active;
  ever_initialized_ = true;
  require_init_ = false;
  CANOPEN_LOG_INFO("LifecycleManager: Active (initialized)");
  return true;
}

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

  if (!Configure(config)) {
    return false;
  }

  if (!InitMotors()) {
    // 兼容旧语义：Init 失败时回到 Unconfigured。
    Shutdown();
    return false;
  }

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
  require_init_ = false;
  CANOPEN_LOG_INFO("LifecycleManager: Configured (halted)");
  return true;
}

bool LifecycleManager::StopCommunication(std::string* detail) {
  if (detail) {
    detail->clear();
  }

  if (state_ != LifecycleState::Active &&
      state_ != LifecycleState::Configured) {
    CANOPEN_LOG_WARN(
        "StopCommunication called in state {}, expected Active/Configured",
        static_cast<int>(state_));
    if (detail) {
      *detail = "invalid lifecycle state";
    }
    return false;
  }

  if (!master_) {
    CANOPEN_LOG_ERROR("StopCommunication: master is null");
    if (detail) {
      *detail = "master is null";
    }
    return false;
  }

  bool graceful_ok = true;
  if (master_->running()) {
    graceful_ok = master_->GracefulShutdown(detail);
  }

  // 即使 402 失能超时，也继续执行通信停机，避免 service 卡死。
  master_->Stop();
  state_ = LifecycleState::Configured;
  require_init_ = true;

  if (!graceful_ok) {
    CANOPEN_LOG_WARN("LifecycleManager: communication stopped with detail: {}",
                     detail ? *detail : std::string("graceful shutdown timeout"));
  } else {
    CANOPEN_LOG_INFO("LifecycleManager: Configured (communication stopped)");
  }
  return graceful_ok;
}

bool LifecycleManager::Recover() {
  if (state_ != LifecycleState::Configured) {
    CANOPEN_LOG_WARN("Recover called in state {}, expected Configured",
                     static_cast<int>(state_));
    return false;
  }

  if (require_init_) {
    CANOPEN_LOG_WARN(
        "Recover called after communication shutdown; call InitMotors first");
    return false;
  }

  if (!ever_initialized_) {
    CANOPEN_LOG_WARN("Recover called before first InitMotors; call InitMotors first");
    return false;
  }

  if (!master_) {
    CANOPEN_LOG_ERROR("Recover: master is null");
    return false;
  }

  master_->Stop();
  if (!master_->Start()) {
    CANOPEN_LOG_ERROR("Recover: master restart failed");
    return false;
  }

  state_ = LifecycleState::Active;
  CANOPEN_LOG_INFO("LifecycleManager: Active (recovered)");
  return true;
}

bool LifecycleManager::Shutdown() {
  if (state_ == LifecycleState::Unconfigured) {
    require_init_ = false;
    return true;
  }

  state_ = LifecycleState::ShuttingDown;

  if (master_) {
    master_->Stop();
  }

  master_.reset();
  robot_hw_.reset();
  shared_state_.reset();
  config_ = CanopenMasterConfig();
  ever_initialized_ = false;
  require_init_ = false;

  state_ = LifecycleState::Unconfigured;
  CANOPEN_LOG_INFO("LifecycleManager: Unconfigured (shutdown)");
  return true;
}

}  // namespace canopen_hw
