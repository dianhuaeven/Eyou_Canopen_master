#include "canopen_hw/canopen_master.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <ctime>

#include "canopen_hw/cia402_defs.hpp"
#include "canopen_hw/logging.hpp"

namespace canopen_hw {

CanopenMaster::CanopenMaster(const CanopenMasterConfig& config,
                             SharedState* shared_state)
    : config_(config), shared_state_(shared_state) {
  if (config_.axis_count == 0) {
    config_.axis_count = 1;
  }
  if (config_.axis_count > SharedState::kAxisCount) {
    CANOPEN_LOG_WARN("axis_count {} exceeds max {}, clamping",
                     config_.axis_count, SharedState::kAxisCount);
    config_.axis_count = SharedState::kAxisCount;
  }
  if (config_.node_ids.size() < config_.axis_count) {
    config_.node_ids.resize(config_.axis_count);
  }
  for (std::size_t i = 0; i < config_.axis_count; ++i) {
    const uint8_t id = config_.node_ids[i];
    if (id == 0 || id > 127) {
      if (id > 127) {
        CANOPEN_LOG_WARN("node_ids[{}]={} out of range 1..127, replacing with default {}",
                         i, static_cast<int>(id), (i + 1));
      }
      config_.node_ids[i] = static_cast<uint8_t>(i + 1);
    }
  }
  if (config_.verify_pdo_mapping.size() < config_.axis_count) {
    config_.verify_pdo_mapping.resize(config_.axis_count, false);
  }
  if (config_.position_lock_thresholds.size() < config_.axis_count) {
    config_.position_lock_thresholds.resize(config_.axis_count, 15000);
  }
  if (config_.max_fault_resets.size() < config_.axis_count) {
    config_.max_fault_resets.resize(config_.axis_count, 3);
  }
  if (config_.fault_reset_hold_cycles.size() < config_.axis_count) {
    config_.fault_reset_hold_cycles.resize(config_.axis_count, 5);
  }
  // 预分配驱动容器容量，保证运行阶段不会因为扩容触发堆分配。
  axis_drivers_.reserve(config_.axis_count);
}

bool CanopenMaster::Start() {
  if (running_.load()) {
    return true;
  }

  try {
    io_guard_ = std::make_unique<lely::io::IoGuard>();
    io_ctx_ = std::make_unique<lely::io::Context>();
    io_poll_ = std::make_unique<lely::io::Poll>(*io_ctx_);
    ev_loop_ = std::make_unique<lely::ev::Loop>(io_poll_->get_poll());
    io_timer_ = std::make_unique<lely::io::Timer>(
        *io_poll_, ev_loop_->get_executor(), CLOCK_MONOTONIC);
    can_ctrl_ =
        std::make_unique<lely::io::CanController>(config_.can_interface.c_str());
    can_chan_ =
        std::make_unique<lely::io::CanChannel>(*io_poll_,
                                               ev_loop_->get_executor());
    can_chan_->open(*can_ctrl_);

    master_ = std::make_unique<lely::canopen::AsyncMaster>(
        ev_loop_->get_executor(), *io_timer_, *can_chan_,
        config_.master_dcf_path, std::string(), config_.master_node_id);
    CreateAxisDrivers(*master_);

    ev_loop_->restart();
    ev_thread_ = std::thread([this]() { ev_loop_->run(); });
    master_->Reset();

    running_.store(true);
    return true;
  } catch (const std::exception& e) {
    CANOPEN_LOG_ERROR("CanopenMaster start failed: {}", e.what());
    if (ev_loop_) {
      ev_loop_->stop();
    }
    if (ev_thread_.joinable()) {
      ev_thread_.join();
    }
    axis_drivers_.clear();
    master_.reset();
    can_chan_.reset();
    can_ctrl_.reset();
    io_timer_.reset();
    ev_loop_.reset();
    io_poll_.reset();
    io_ctx_.reset();
    io_guard_.reset();
    running_.store(false);
    return false;
  }
}

void CanopenMaster::Stop() {
  if (!running_.load() && !master_ && !ev_loop_ && axis_drivers_.empty()) {
    return;
  }

  GracefulShutdown();

  if (ev_loop_) {
    ev_loop_->stop();
  }
  if (ev_thread_.joinable()) {
    ev_thread_.join();
  }

  axis_drivers_.clear();
  master_.reset();
  can_chan_.reset();
  can_ctrl_.reset();
  io_timer_.reset();
  ev_loop_.reset();
  io_poll_.reset();
  io_ctx_.reset();
  io_guard_.reset();

  running_.store(false);
}

bool CanopenMaster::GracefulShutdown() {
  if (axis_drivers_.empty()) {
    return true;
  }

  // 注意: Stop() 必须在 Lely 事件循环仍在运行时调用，否则 RPDO 不再更新，
  // 下面的等待将只会超时返回。
  for (const auto& axis : axis_drivers_) {
    if (axis) {
      axis->SendControlword(kCtrl_DisableOperation);
    }
  }
  WaitForAllState(CiA402State::SwitchedOn,
                  std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(2000));

  for (const auto& axis : axis_drivers_) {
    if (axis) {
      axis->SendControlword(kCtrl_Shutdown);
    }
  }
  WaitForAllState(CiA402State::ReadyToSwitchOn,
                  std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(1000));

  if (axis_drivers_.front()) {
    axis_drivers_.front()->SendNmtStopAll();
  }
  return true;
}

bool CanopenMaster::WaitForAllState(
    CiA402State target_state,
    std::chrono::steady_clock::time_point deadline) {
  while (std::chrono::steady_clock::now() < deadline) {
    bool all_match = true;
    for (const auto& axis : axis_drivers_) {
      if (!axis) {
        continue;
      }
      if (axis->feedback_state() != target_state) {
        all_match = false;
        break;
      }
    }
    if (all_match) {
      return true;
    }
    // 阻塞等待反馈更新通知，而非固定 sleep 轮询。
    // 每次 UpdateFeedback() 完成后会唤醒，避免空转消耗 CPU。
    shared_state_->WaitForStateChange(
        std::min(deadline,
                 std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(10)));
  }
  return false;
}

void CanopenMaster::CreateAxisDrivers(lely::canopen::BasicMaster& can_master) {
  // 初始化阶段函数: clear 后重新 emplace，不会超过预留容量。
  axis_drivers_.clear();

  for (std::size_t i = 0; i < config_.axis_count; ++i) {
    const uint8_t node_id = config_.node_ids[i];
    auto axis = std::make_unique<AxisDriver>(can_master, node_id, i, shared_state_,
                                             config_.verify_pdo_mapping[i],
                                             config_.master_dcf_path);
    axis->ConfigureStateMachine(config_.position_lock_thresholds[i],
                                config_.max_fault_resets[i],
                                config_.fault_reset_hold_cycles[i]);
    axis_drivers_.emplace_back(std::move(axis));
  }
}

}  // namespace canopen_hw
