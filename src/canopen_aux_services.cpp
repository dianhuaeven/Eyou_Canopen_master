#include "canopen_hw/canopen_aux_services.hpp"

#include <sstream>

#include "Eyou_Canopen_Master/SetMode.h"
#include "Eyou_Canopen_Master/SetZero.h"
#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

namespace {

constexpr auto kSoftLimitSdoIdleTimeout = std::chrono::milliseconds(2500);
constexpr auto kSetZeroSdoIdleTimeout = std::chrono::milliseconds(2500);

bool IsAllowedMode(int8_t mode) {
  return mode == kMode_IP || mode == kMode_CSP ||
         mode == kMode_CSV || mode == kMode_CST;
}

}  // namespace

bool CanopenAuxServices::SetModeAxis(std::size_t axis_index,
                                     int8_t mode,
                                     std::string* detail) {
  if (detail != nullptr) {
    detail->clear();
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto system_mode = coordinator_->mode();
  if (system_mode != SystemOpMode::Standby) {
    if (detail != nullptr) {
      *detail = "set_mode only allowed in Standby; call ~/disable first";
    }
    return false;
  }
  if (axis_index >= robot_hw_ros_->axis_count()) {
    if (detail != nullptr) {
      *detail = "axis_index out of range";
    }
    return false;
  }
  if (!IsAllowedMode(mode)) {
    if (detail != nullptr) {
      *detail =
          "unsupported mode: " + std::to_string(mode) +
          ", allowed: 7(IP),8(CSP),9(CSV),10(CST)";
    }
    return false;
  }

  robot_hw_ros_->SetMode(axis_index, mode);
  if (detail != nullptr) {
    *detail = "mode set";
  }
  return true;
}

CanopenAuxServices::CanopenAuxServices(
    ros::NodeHandle* pnh,
    CanopenRobotHwRos* robot_hw_ros,
    OperationalCoordinator* coordinator,
    const CanopenMasterConfig* master_cfg,
    CanopenMaster* master,
    std::mutex* loop_mtx)
    : robot_hw_ros_(robot_hw_ros),
      coordinator_(coordinator),
      master_cfg_(master_cfg),
      master_(master),
      loop_mtx_(loop_mtx),
      zero_executor_(master, master_cfg) {
  // ── ~/set_mode ──
  set_mode_srv_ = pnh->advertiseService<Eyou_Canopen_Master::SetMode::Request,
                                        Eyou_Canopen_Master::SetMode::Response>(
      "set_mode", [this](Eyou_Canopen_Master::SetMode::Request& req,
                         Eyou_Canopen_Master::SetMode::Response& res) {
        std::string detail;
        res.success = SetModeAxis(req.axis_index, req.mode, &detail);
        res.message = detail;
        return true;
      });

  // ── ~/set_zero ──
  set_zero_srv_ = pnh->advertiseService<Eyou_Canopen_Master::SetZero::Request,
                                        Eyou_Canopen_Master::SetZero::Response>(
      "set_zero", [this](Eyou_Canopen_Master::SetZero::Request& req,
                         Eyou_Canopen_Master::SetZero::Response& res) {
        std::lock_guard<std::mutex> lk(*loop_mtx_);
        const auto mode = coordinator_->mode();
        if (mode != SystemOpMode::Standby) {
          res.success = false;
          res.message = "set_zero only allowed in Standby; call ~/disable first";
          return true;
        }
        if (req.axis_index >= master_cfg_->joints.size()) {
          res.success = false;
          res.message = "axis_index out of range";
          return true;
        }

        std::string detail;
        PreparedSoftLimit prepared;
        int32_t previous_home_offset = 0;
        if (master_cfg_->auto_write_soft_limits_from_urdf) {
          if (!PrepareSoftLimitAxis(req.axis_index, &prepared, &detail)) {
            res.success = false;
            res.message = detail.empty() ? "soft limit precheck failed" : detail;
            return true;
          }
          if (!WaitForAxisSdoIdle(req.axis_index, kSetZeroSdoIdleTimeout, &detail)) {
            res.success = false;
            res.message = detail.empty() ? "axis SDO idle wait failed" : detail;
            return true;
          }
          if (!zero_executor_.ReadHomeOffset(req.axis_index,
                                             &previous_home_offset, &detail)) {
            res.success = false;
            res.message = detail.empty() ? "read home offset failed" : detail;
            return true;
          }
        }

        if (!zero_executor_.SetCurrentPositionAsZero(req.axis_index, &detail)) {
          res.success = false;
          res.message = detail.empty() ? "set_zero failed" : detail;
          return true;
        }

        if (master_cfg_->auto_write_soft_limits_from_urdf) {
          if (!zero_executor_.ApplySoftLimitCounts(
                  req.axis_index, prepared.min_counts,
                  prepared.max_counts, &detail)) {
            const std::string soft_limit_error =
                detail.empty() ? "soft limit rewrite failed" : detail;
            std::string rollback_detail;
            if (!zero_executor_.RestoreHomeOffset(
                    req.axis_index, previous_home_offset, &rollback_detail)) {
              res.success = false;
              res.message =
                  "zero set and soft limit rewrite failed; rollback failed: " +
                  soft_limit_error;
              if (!rollback_detail.empty()) {
                res.message += "; " + rollback_detail;
              }
              return true;
            }
            res.success = false;
            res.message =
                "soft limit rewrite failed after zeroing; restored previous home offset: " +
                soft_limit_error;
            return true;
          }
        }

        res.success = true;
        res.message = detail.empty() ? "zero set" : detail;
        return true;
      });
}

// ── URDF 软限位 ──────────────────────────────────────

bool CanopenAuxServices::EnsureUrdfLimits(std::string* detail) {
  if (urdf_limits_ready_) {
    return true;
  }
  std::string urdf_xml;
  ros::NodeHandle nh;
  if (!nh.getParam("robot_description", urdf_xml) || urdf_xml.empty()) {
    if (detail) {
      *detail = "robot_description is missing; cannot derive URDF soft limits";
    }
    return false;
  }
  std::string parse_error;
  if (!ParseUrdfJointLimits(urdf_xml, master_cfg_->joints, &urdf_limits_,
                            &parse_error)) {
    if (detail) {
      *detail = parse_error.empty() ? "failed to parse URDF joint limits"
                                    : parse_error;
    }
    return false;
  }
  urdf_limits_ready_ = true;
  return true;
}

bool CanopenAuxServices::ApplySoftLimitAxis(std::size_t axis_index,
                                            std::string* detail) {
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (axis_index >= urdf_limits_.size()) {
    if (detail) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " out of URDF limit range";
      *detail = oss.str();
    }
    return false;
  }
  const auto& limit = urdf_limits_[axis_index];
  if (limit.unit == UrdfJointLimitUnit::kRadians) {
    return zero_executor_.ApplySoftLimitRadians(
        axis_index, limit.lower, limit.upper, detail);
  }
  if (limit.unit == UrdfJointLimitUnit::kMeters) {
    return zero_executor_.ApplySoftLimitMeters(
        axis_index, limit.lower, limit.upper, detail);
  }
  if (detail) {
    *detail = "unsupported URDF limit unit";
  }
  return false;
}

bool CanopenAuxServices::PrepareSoftLimitAxis(std::size_t axis_index,
                                               PreparedSoftLimit* out,
                                               std::string* detail) {
  if (!out) {
    if (detail) {
      *detail = "prepared soft limit output is null";
    }
    return false;
  }
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (axis_index >= urdf_limits_.size() ||
      axis_index >= master_cfg_->joints.size()) {
    if (detail) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " out of URDF limit range";
      *detail = oss.str();
    }
    return false;
  }

  const auto& limit = urdf_limits_[axis_index];
  if (limit.unit == UrdfJointLimitUnit::kRadians) {
    return zero_executor_.PrepareSoftLimitRadians(
        axis_index, limit.lower, limit.upper,
        &out->min_counts, &out->max_counts, detail);
  }
  if (limit.unit == UrdfJointLimitUnit::kMeters) {
    return zero_executor_.PrepareSoftLimitMeters(
        axis_index, limit.lower, limit.upper,
        &out->min_counts, &out->max_counts, detail);
  }
  if (detail) {
    *detail = "unsupported URDF limit unit";
  }
  return false;
}

bool CanopenAuxServices::WaitForAllSdoIdle(std::chrono::milliseconds timeout,
                                           std::string* detail) const {
  if (!master_) {
    if (detail) {
      *detail = "master is null; cannot wait for SDO idle";
    }
    return false;
  }

  std::vector<std::size_t> pending_axes;
  if (master_->WaitForAllSdoIdle(timeout, &pending_axes)) {
    return true;
  }

  if (detail) {
    std::ostringstream oss;
    oss << "timed out waiting for all-axis SDO idle";
    if (!pending_axes.empty()) {
      oss << "; pending axes: ";
      for (std::size_t i = 0; i < pending_axes.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << pending_axes[i];
        if (master_cfg_ && pending_axes[i] < master_cfg_->joints.size()) {
          oss << "(node="
              << static_cast<int>(master_cfg_->joints[pending_axes[i]].node_id)
              << ")";
        }
      }
    }
    *detail = oss.str();
  }
  return false;
}

bool CanopenAuxServices::WaitForAxisSdoIdle(std::size_t axis_index,
                                            std::chrono::milliseconds timeout,
                                            std::string* detail) const {
  if (!master_) {
    if (detail) {
      *detail = "master is null; cannot wait for SDO idle";
    }
    return false;
  }
  if (axis_index >= master_cfg_->joints.size()) {
    if (detail) {
      *detail = "axis_index out of range for SDO idle wait";
    }
    return false;
  }
  if (master_->WaitForSdoIdle(axis_index, timeout)) {
    return true;
  }

  if (detail) {
    std::ostringstream oss;
    oss << "timed out waiting for axis " << axis_index << " SDO idle"
        << " (node=" << static_cast<int>(master_cfg_->joints[axis_index].node_id)
        << ")";
    *detail = oss.str();
  }
  return false;
}

bool CanopenAuxServices::ApplySoftLimitAll(std::string* detail) {
  if (!master_cfg_->auto_write_soft_limits_from_urdf) {
    return true;
  }
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (!WaitForAllSdoIdle(kSoftLimitSdoIdleTimeout, detail)) {
    return false;
  }
  for (std::size_t i = 0; i < master_cfg_->joints.size(); ++i) {
    std::string axis_detail;
    if (!ApplySoftLimitAxis(i, &axis_detail)) {
      if (detail) {
        std::ostringstream oss;
        oss << "apply soft limit failed at axis " << i;
        if (!axis_detail.empty()) {
          oss << ": " << axis_detail;
        }
        *detail = oss.str();
      }
      return false;
    }
  }
  if (detail) {
    *detail = "soft limits applied from URDF";
  }
  return true;
}

}  // namespace canopen_hw
