#include "canopen_hw/axis_driver.hpp"

#include <iostream>
#include <system_error>

#include <lely/coapp/node.hpp>

#include "canopen_hw/pdo_mapping.hpp"

namespace canopen_hw {

AxisDriver::AxisDriver(lely::canopen::BasicMaster& master, uint8_t node_id,
                       std::size_t axis_index, SharedState* shared_state,
                       bool verify_pdo_mapping, const std::string& dcf_path)
    : lely::canopen::BasicDriver(master, node_id),
      axis_index_(axis_index),
      shared_state_(shared_state),
      verify_pdo_mapping_(verify_pdo_mapping),
      dcf_path_(dcf_path) {
  state_machine_.set_target_mode(kMode_CSP);
  pdo_verified_ = !verify_pdo_mapping_;
  pdo_verification_done_ = !verify_pdo_mapping_;
}

void AxisDriver::SetRosTargetPosition(int32_t target_position) {
  std::lock_guard<std::mutex> lk(mtx_);
  state_machine_.set_ros_target(target_position);
}

void AxisDriver::InjectFeedback(int32_t actual_position, int32_t actual_velocity,
                                int16_t actual_torque, uint16_t statusword,
                                int8_t mode_display) {
  std::lock_guard<std::mutex> lk(mtx_);

  feedback_cache_.actual_position = actual_position;
  feedback_cache_.actual_velocity = actual_velocity;
  feedback_cache_.actual_torque = actual_torque;
  feedback_cache_.statusword = statusword;
  feedback_cache_.mode_display = mode_display;

  // 这里是单轴状态机核心入口:
  // 每次收到一帧同步反馈，都用最新 statusword/mode 推进一步。
  state_machine_.Update(statusword, mode_display, actual_position);

  feedback_cache_.state = state_machine_.state();
  feedback_cache_.is_operational = state_machine_.is_operational();
  feedback_cache_.is_fault = state_machine_.is_fault();
  if (verify_pdo_mapping_ &&
      (!pdo_verification_done_ || !pdo_verified_)) {
    feedback_cache_.is_operational = false;
  }

  PublishSnapshot();
}

bool AxisDriver::SendControlword(uint16_t controlword) {
  // 从主站视角发送 TPDO（从站视角接收 RPDO），用于下发 0x6040。
  std::error_code ec;
  tpdo_mapped[0x6040][0].Write(controlword, ec);
  if (ec) {
    return false;
  }
  tpdo_mapped[0x6040][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::SendNmtStopAll() {
  master.Command(lely::canopen::NmtCommand::STOP);
  return true;
}

CiA402State AxisDriver::feedback_state() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return feedback_cache_.state;
}

void AxisDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept {
  (void)idx;
  (void)subidx;
  // RPDO 触发后读取关键反馈字段并推进状态机。
  std::error_code ec;
  const auto statusword = rpdo_mapped[0x6041][0].Read<uint16_t>(ec);
  if (ec) {
    return;
  }
  const auto actual_position = rpdo_mapped[0x6064][0].Read<int32_t>(ec);
  if (ec) {
    return;
  }
  const auto mode_display = rpdo_mapped[0x6061][0].Read<int8_t>(ec);
  if (ec) {
    return;
  }
  const auto actual_velocity = rpdo_mapped[0x606C][0].Read<int32_t>(ec);
  if (ec) {
    return;
  }
  const auto actual_torque = rpdo_mapped[0x6077][0].Read<int16_t>(ec);
  if (ec) {
    return;
  }

  InjectFeedback(actual_position, actual_velocity, actual_torque, statusword,
                 mode_display);

  if (shared_state_) {
    shared_state_->RecomputeAllOperational();
  }
}

void AxisDriver::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept {
  (void)eec;
  (void)er;
  (void)msef;
  // TODO: 增加错误码映射与日志记录。
}

void AxisDriver::OnHeartbeat(bool occurred) noexcept {
  (void)occurred;
  // TODO: 心跳超时/恢复状态上报到 SharedState。
}

void AxisDriver::OnBoot(lely::canopen::NmtState st, char es,
                        const std::string& what) noexcept {
  (void)what;
  if (!verify_pdo_mapping_) {
    pdo_verified_ = true;
    pdo_verification_done_ = true;
    return;
  }
  if (pdo_verification_done_) {
    return;
  }
  if (es != 0 || st == lely::canopen::NmtState::BOOTUP) {
    std::cerr << "Axis " << axis_index_ << " (node " << static_cast<int>(id())
              << "): OnConfig failed, skip PDO verify" << std::endl;
    pdo_verified_ = false;
    pdo_verification_done_ = true;
    return;
  }
  if (dcf_path_.empty()) {
    std::cerr << "Axis " << axis_index_ << " (node " << static_cast<int>(id())
              << "): DCF path empty, skip PDO verify" << std::endl;
    pdo_verified_ = false;
    pdo_verification_done_ = true;
    return;
  }

  pdo_reader_ = std::make_shared<PdoMappingReader>();
  pdo_reader_->Start(*this, [this](bool ok, const std::string& error,
                                   const PdoMapping& actual) {
    if (!ok) {
      std::cerr << "Axis " << axis_index_ << " (node " << static_cast<int>(id())
                << "): PDO read failed: " << error << std::endl;
      pdo_verified_ = false;
      pdo_verification_done_ = true;
      pdo_reader_.reset();
      return;
    }

    PdoMapping expected;
    std::string err;
    if (!LoadExpectedPdoMappingFromDcf(dcf_path_, &expected, &err)) {
      std::cerr << "Axis " << axis_index_ << " (node " << static_cast<int>(id())
                << "): DCF load failed: " << err << std::endl;
      pdo_verified_ = false;
      pdo_verification_done_ = true;
      pdo_reader_.reset();
      return;
    }

    std::vector<std::string> diffs;
    if (!DiffPdoMapping(expected, actual, &diffs)) {
      std::cerr << "Axis " << axis_index_ << " (node " << static_cast<int>(id())
                << "): PDO mapping mismatch" << std::endl;
      for (const auto& diff : diffs) {
        std::cerr << "  " << diff << std::endl;
      }
      pdo_verified_ = false;
    } else {
      std::cout << "Axis " << axis_index_ << " (node " << static_cast<int>(id())
                << "): PDO mapping verified" << std::endl;
      pdo_verified_ = true;
    }

    pdo_verification_done_ = true;
    pdo_reader_.reset();
  });
}

void AxisDriver::PublishSnapshot() {
  if (!shared_state_) {
    return;
  }

  // 将状态机过滤后的目标位置同步回命令面，后续 write PDO 时直接读取该值。
  AxisCommand cmd;
  cmd.target_position = state_machine_.safe_target();

  shared_state_->UpdateFeedback(axis_index_, feedback_cache_);
  shared_state_->UpdateCommand(axis_index_, cmd);
}

}  // namespace canopen_hw
