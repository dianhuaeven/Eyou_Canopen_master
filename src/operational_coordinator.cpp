#include "canopen_hw/operational_coordinator.hpp"

#include <algorithm>
#include <sstream>

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

const char* SystemOpModeName(SystemOpMode mode) {
  switch (mode) {
    case SystemOpMode::Inactive:
      return "Inactive";
    case SystemOpMode::Configured:
      return "Configured";
    case SystemOpMode::Standby:
      return "Standby";
    case SystemOpMode::Armed:
      return "Armed";
    case SystemOpMode::Running:
      return "Running";
    case SystemOpMode::Faulted:
      return "Faulted";
    case SystemOpMode::Recovering:
      return "Recovering";
    case SystemOpMode::ShuttingDown:
      return "ShuttingDown";
  }
  return "Unknown";
}

OperationalCoordinator::OperationalCoordinator(CanopenMaster* master,
                                               SharedState* shared_state,
                                               std::size_t axis_count)
    : master_(master), shared_state_(shared_state), axis_count_(axis_count) {}

void OperationalCoordinator::SetConfigured() {
  mode_.store(SystemOpMode::Configured, std::memory_order_release);
}

OperationalCoordinator::Result OperationalCoordinator::DoTransition(
    std::initializer_list<SystemOpMode> allowed_from, SystemOpMode to,
    std::function<bool(std::string*)> action) {
  std::lock_guard<std::mutex> lk(transition_mtx_);

  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  bool allowed = false;
  for (const auto from : allowed_from) {
    if (from == current) {
      allowed = true;
      break;
    }
  }

  if (!allowed) {
    if (current == to) {
      return {true, std::string("already ") + SystemOpModeName(to)};
    }
    std::ostringstream oss;
    oss << "cannot transition from " << SystemOpModeName(current) << " to "
        << SystemOpModeName(to);
    return {false, oss.str()};
  }

  std::string detail;
  if (action && !action(&detail)) {
    if (detail.empty()) {
      detail = "action failed";
    }
    return {false, detail};
  }

  mode_.store(to, std::memory_order_release);
  CANOPEN_LOG_INFO("OperationalCoordinator: {} -> {}", SystemOpModeName(current),
                   SystemOpModeName(to));
  if (detail.empty()) {
    detail = std::string("-> ") + SystemOpModeName(to);
  }
  return {true, detail};
}

OperationalCoordinator::Result OperationalCoordinator::RequestInit() {
  return DoTransition(
      {SystemOpMode::Configured}, SystemOpMode::Running,
      [this](std::string* detail) {
        if (!master_) {
          if (detail) {
            *detail = "master is null";
          }
          return false;
        }
        if (!master_->running() && !master_->Start()) {
          if (detail) {
            *detail = "master start failed";
          }
          return false;
        }
        if (!master_->EnableAll()) {
          if (detail) {
            *detail = "EnableAll failed";
          }
          return false;
        }
        return true;
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestEnable() {
  return DoTransition(
      {SystemOpMode::Standby}, SystemOpMode::Armed,
      [this](std::string* detail) {
        if (!master_) {
          if (detail) {
            *detail = "master is null";
          }
          return false;
        }
        if (!master_->EnableAll()) {
          if (detail) {
            *detail = "EnableAll failed";
          }
          return false;
        }
        if (!master_->HaltAll()) {
          if (detail) {
            *detail = "HaltAll failed";
          }
          return false;
        }
        return true;
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestRelease() {
  return DoTransition(
      {SystemOpMode::Armed}, SystemOpMode::Running,
      [this](std::string* detail) {
        if (!master_) {
          if (detail) {
            *detail = "master is null";
          }
          return false;
        }
        if (!master_->ResumeAll()) {
          if (detail) {
            *detail = "ResumeAll failed";
          }
          return false;
        }
        return true;
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestHalt() {
  return DoTransition(
      {SystemOpMode::Running}, SystemOpMode::Armed,
      [this](std::string* detail) {
        if (!master_) {
          if (detail) {
            *detail = "master is null";
          }
          return false;
        }
        if (!master_->HaltAll()) {
          if (detail) {
            *detail = "HaltAll failed";
          }
          return false;
        }
        return true;
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestRecover() {
  return DoTransition(
      {SystemOpMode::Faulted}, SystemOpMode::Armed,
      [this](std::string* detail) {
        if (!master_) {
          if (detail) {
            *detail = "master is null";
          }
          return false;
        }
        mode_.store(SystemOpMode::Recovering, std::memory_order_release);

        std::string recover_detail;
        if (!master_->RecoverFaultedAxes(&recover_detail)) {
          mode_.store(SystemOpMode::Faulted, std::memory_order_release);
          if (detail) {
            *detail = recover_detail.empty() ? "recover failed" : recover_detail;
          }
          return false;
        }

        if (shared_state_) {
          shared_state_->SetGlobalFault(false);
          shared_state_->SetAllAxesHaltedByFault(false);
        }

        if (!master_->HaltAll()) {
          mode_.store(SystemOpMode::Faulted, std::memory_order_release);
          if (detail) {
            *detail = "recover succeeded but HaltAll failed";
          }
          return false;
        }

        if (detail && !recover_detail.empty()) {
          *detail = recover_detail;
        }
        return true;
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestShutdown() {
  return DoTransition(
      {SystemOpMode::Configured, SystemOpMode::Standby, SystemOpMode::Armed,
       SystemOpMode::Running, SystemOpMode::Faulted},
      SystemOpMode::Configured,
      [this](std::string* detail) {
        if (!master_) {
          if (detail) {
            *detail = "master is null";
          }
          return false;
        }

        mode_.store(SystemOpMode::ShuttingDown, std::memory_order_release);

        std::string graceful_detail;
        const bool graceful_ok = master_->GracefulShutdown(&graceful_detail);
        master_->Stop();

        if (!graceful_ok) {
          if (detail) {
            *detail = graceful_detail.empty() ? "graceful shutdown timeout"
                                              : graceful_detail;
          }
          // 保持与现有 stop-communication 语义一致：即使超时，通信也已停止。
          mode_.store(SystemOpMode::Configured, std::memory_order_release);
          return false;
        }

        if (shared_state_) {
          shared_state_->SetGlobalFault(false);
          shared_state_->SetAllAxesHaltedByFault(false);
        }

        if (detail && !graceful_detail.empty()) {
          *detail = graceful_detail;
        }
        return true;
      });
}

void OperationalCoordinator::ComputeIntents() {
  if (!shared_state_) {
    return;
  }

  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  AxisIntent intent = AxisIntent::Disable;

  switch (current) {
    case SystemOpMode::Running:
      intent = AxisIntent::Run;
      break;
    case SystemOpMode::Armed:
    case SystemOpMode::Recovering:
    case SystemOpMode::Faulted:
      intent = AxisIntent::Halt;
      break;
    case SystemOpMode::Inactive:
    case SystemOpMode::Configured:
    case SystemOpMode::Standby:
    case SystemOpMode::ShuttingDown:
      intent = AxisIntent::Disable;
      break;
  }

  for (std::size_t i = 0; i < axis_count_; ++i) {
    shared_state_->SetAxisIntent(i, intent);
  }
  shared_state_->AdvanceIntentSequence();
}

void OperationalCoordinator::UpdateFromFeedback() {
  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  if (current != SystemOpMode::Armed && current != SystemOpMode::Running) {
    return;
  }
  if (!shared_state_) {
    return;
  }

  const SharedSnapshot snap = shared_state_->Snapshot();
  const std::size_t n = std::min(axis_count_, snap.feedback.size());
  bool any_fault = false;
  for (std::size_t i = 0; i < n; ++i) {
    if (snap.feedback[i].is_fault || snap.feedback[i].heartbeat_lost) {
      any_fault = true;
      break;
    }
  }

  if (!any_fault) {
    return;
  }

  SystemOpMode expected = current;
  if (mode_.compare_exchange_strong(expected, SystemOpMode::Faulted,
                                    std::memory_order_acq_rel)) {
    CANOPEN_LOG_WARN("OperationalCoordinator: {} -> Faulted (auto)",
                     SystemOpModeName(current));
  }
}

}  // namespace canopen_hw
