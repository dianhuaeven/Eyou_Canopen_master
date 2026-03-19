#include "canopen_hw/axis_driver.hpp"

#include <chrono>
#include <system_error>

#include <lely/coapp/node.hpp>

#include "canopen_hw/logging.hpp"
#include "canopen_hw/pdo_mapping.hpp"

namespace canopen_hw {

AxisDriver::AxisDriver(lely::canopen::BasicMaster& can_master, uint8_t node_id,
                       std::size_t axis_index, SharedState* shared_state,
                       bool verify_pdo_mapping, const std::string& dcf_path)
    : lely::canopen::BasicDriver(can_master, node_id),
      axis_index_(axis_index),
      shared_state_(shared_state),
      logic_(axis_index, this, shared_state),
      verify_pdo_mapping_(verify_pdo_mapping),
      dcf_path_(dcf_path) {
  pdo_verified_.store(!verify_pdo_mapping_);
  pdo_verification_done_.store(!verify_pdo_mapping_);
  if (verify_pdo_mapping_) {
    expected_pdo_ = std::make_shared<PdoMapping>();
    if (dcf_path_.empty()) {
      CANOPEN_LOG_WARN("axis={} node={}: DCF path empty, skip PDO verify",
                       axis_index_, static_cast<int>(id()));
      pdo_verified_.store(false);
      pdo_verification_done_.store(true);
    } else {
      std::string err;
      if (LoadExpectedPdoMappingFromDcf(dcf_path_, expected_pdo_.get(), &err)) {
        expected_pdo_loaded_ = true;
        pdo_verified_.store(false);
        pdo_verification_done_.store(false);
      } else {
        CANOPEN_LOG_ERROR("axis={} node={}: DCF load failed: {}",
                          axis_index_, static_cast<int>(id()), err);
        pdo_verified_.store(false);
        pdo_verification_done_.store(true);
      }
    }
  }
}

void AxisDriver::InjectFeedback(int32_t actual_position, int32_t actual_velocity,
                                int16_t actual_torque, uint16_t statusword,
                                int8_t mode_display) {
  logic_.ProcessRpdo(statusword, actual_position, actual_velocity,
                     actual_torque, mode_display);
}

// --- BusIO implementation ---

bool AxisDriver::WriteControlword(uint16_t cw) {
  std::error_code ec;
  tpdo_mapped[0x6040][0].Write(cw, ec);
  if (ec) return false;
  tpdo_mapped[0x6040][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::WriteTargetPosition(int32_t pos) {
  std::error_code ec;
  tpdo_mapped[0x607A][0].Write(pos, ec);
  if (ec) return false;
  tpdo_mapped[0x607A][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::WriteTargetVelocity(int32_t vel) {
  std::error_code ec;
  tpdo_mapped[0x60FF][0].Write(vel, ec);
  if (ec) return false;
  tpdo_mapped[0x60FF][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::WriteTargetTorque(int16_t torque) {
  std::error_code ec;
  tpdo_mapped[0x6071][0].Write(torque, ec);
  if (ec) return false;
  tpdo_mapped[0x6071][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::WriteModeOfOperation(int8_t mode) {
  std::error_code ec;
  tpdo_mapped[0x6060][0].Write(mode, ec);
  if (ec) return false;
  tpdo_mapped[0x6060][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::SendNmtStopAll() {
  master.Command(lely::canopen::NmtCommand::STOP);
  return true;
}

// --- Delegated to AxisLogic ---

CiA402State AxisDriver::feedback_state() const {
  return logic_.feedback_state();
}

void AxisDriver::ConfigureStateMachine(int32_t position_lock_threshold,
                                       int max_fault_resets,
                                       int fault_reset_hold_cycles) {
  logic_.Configure(position_lock_threshold, max_fault_resets,
                   fault_reset_hold_cycles);
}

void AxisDriver::RequestEnable() { logic_.RequestEnable(); }
void AxisDriver::RequestDisable() { logic_.RequestDisable(); }
void AxisDriver::ResetFault() { logic_.ResetFault(); }

// --- SDO (stays in AxisDriver, Lely-specific) ---

void AxisDriver::AsyncSdoRead(uint16_t index, uint8_t subindex,
                               SdoReadCallback cb) {
  SubmitRead<uint32_t>(
      index, subindex,
      [cb](uint8_t, uint16_t, uint8_t, std::error_code ec, uint32_t value) {
        if (ec) {
          if (cb) cb(false, {}, ec.message());
          return;
        }
        std::vector<uint8_t> data(4);
        data[0] = static_cast<uint8_t>(value & 0xFF);
        data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
        data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
        if (cb) cb(true, data, std::string());
      });
}

void AxisDriver::AsyncSdoWrite(uint16_t index, uint8_t subindex,
                                const std::vector<uint8_t>& data,
                                SdoWriteCallback cb) {
  uint32_t value = 0;
  if (data.size() >= 1) value |= static_cast<uint32_t>(data[0]);
  if (data.size() >= 2) value |= static_cast<uint32_t>(data[1]) << 8;
  if (data.size() >= 3) value |= static_cast<uint32_t>(data[2]) << 16;
  if (data.size() >= 4) value |= static_cast<uint32_t>(data[3]) << 24;

  SubmitWrite(
      index, subindex, value,
      [cb](uint8_t, uint16_t, uint8_t, std::error_code ec) {
        if (cb) cb(!ec, ec ? ec.message() : std::string());
      });
}

// --- Lely callbacks ---

void AxisDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept {
  (void)idx;
  (void)subidx;

  // 从 SharedState 读取 ROS 侧命令并注入 AxisLogic。
  if (shared_state_) {
    AxisCommand cmd;
    if (shared_state_->GetCommand(axis_index_, &cmd)) {
      logic_.SetRosTarget(cmd.target_position);
      logic_.SetRosTargetVelocity(cmd.target_velocity);
      logic_.SetRosTargetTorque(cmd.target_torque);
      logic_.SetTargetMode(cmd.mode_of_operation);
    }
  }

  // 读取 RPDO 反馈字段。
  std::error_code ec;
  const auto statusword = rpdo_mapped[0x6041][0].Read<uint16_t>(ec);
  if (ec) return;
  const auto actual_position = rpdo_mapped[0x6064][0].Read<int32_t>(ec);
  if (ec) return;
  const auto mode_display = rpdo_mapped[0x6061][0].Read<int8_t>(ec);
  if (ec) return;
  const auto actual_velocity = rpdo_mapped[0x606C][0].Read<int32_t>(ec);
  if (ec) return;
  const auto actual_torque = rpdo_mapped[0x6077][0].Read<int16_t>(ec);
  if (ec) return;

  logic_.ProcessRpdo(statusword, actual_position, actual_velocity,
                     actual_torque, mode_display);
}

void AxisDriver::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept {
  (void)msef;
  logic_.ProcessEmcy(eec, er);
}

void AxisDriver::OnHeartbeat(bool occurred) noexcept {
  logic_.ProcessHeartbeat(occurred);
}

void AxisDriver::OnBoot(lely::canopen::NmtState st, char es,
                        const std::string& what) noexcept {
  (void)st;
  (void)what;
  if (!verify_pdo_mapping_) {
    pdo_verified_.store(true);
    pdo_verification_done_.store(true);
    boot_retry_count_.store(0);
    return;
  }
  if (pdo_verification_done_.load()) {
    return;
  }
  if (es != 0) {
    int retry_count = boot_retry_count_.load();
    while (retry_count < max_boot_retries_ &&
           !boot_retry_count_.compare_exchange_weak(retry_count,
                                                    retry_count + 1)) {
    }
    if (retry_count < max_boot_retries_) {
      const int attempt = retry_count + 1;
      logic_.mutable_health().boot_retries.fetch_add(1, std::memory_order_relaxed);
      CANOPEN_LOG_WARN("axis={} node={}: OnConfig failed (es={}), retry {}/{} with RESET_NODE",
                       axis_index_, static_cast<int>(id()), static_cast<int>(es),
                       attempt, max_boot_retries_);
      master.Command(lely::canopen::NmtCommand::RESET_NODE, id());
      return;
    }
    CANOPEN_LOG_ERROR("axis={} node={}: OnConfig failed (es={}), retries exhausted; mark PDO verify failed",
                      axis_index_, static_cast<int>(id()), static_cast<int>(es));
    pdo_verified_.store(false);
    pdo_verification_done_.store(true);
    logic_.mutable_health().pdo_verify_fail.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  boot_retry_count_.store(0);
  if (!expected_pdo_loaded_ || !expected_pdo_) {
    CANOPEN_LOG_WARN("axis={} node={}: DCF not loaded, skip PDO verify",
                     axis_index_, static_cast<int>(id()));
    pdo_verified_.store(false);
    pdo_verification_done_.store(true);
    return;
  }

  pdo_reader_ = std::make_shared<PdoMappingReader>();
  pdo_reader_->Start(*this, [this](bool ok, const std::string& error,
                                   const PdoMapping& actual) {
    if (!ok) {
      CANOPEN_LOG_ERROR("axis={} node={}: PDO read failed: {}",
                        axis_index_, static_cast<int>(id()), error);
      pdo_verified_.store(false);
      pdo_verification_done_.store(true);
      if (error.find("timeout") != std::string::npos) {
        logic_.mutable_health().pdo_verify_timeout.fetch_add(1, std::memory_order_relaxed);
      } else {
        logic_.mutable_health().pdo_verify_fail.fetch_add(1, std::memory_order_relaxed);
      }
      return;
    }

    std::vector<std::string> diffs;
    if (!DiffPdoMapping(*expected_pdo_, actual, &diffs)) {
      CANOPEN_LOG_WARN("axis={} node={}: PDO mapping mismatch",
                       axis_index_, static_cast<int>(id()));
      for (const auto& diff : diffs) {
        CANOPEN_LOG_WARN("  {}", diff);
      }
      pdo_verified_.store(false);
      logic_.mutable_health().pdo_verify_fail.fetch_add(1, std::memory_order_relaxed);
    } else {
      CANOPEN_LOG_INFO("axis={} node={}: PDO mapping verified",
                       axis_index_, static_cast<int>(id()));
      pdo_verified_.store(true);
      logic_.mutable_health().pdo_verify_ok.fetch_add(1, std::memory_order_relaxed);
    }

    pdo_verification_done_.store(true);
  }, std::chrono::milliseconds(2000));
}

}  // namespace canopen_hw
