#include "canopen_hw/zero_soft_limit_executor.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace canopen_hw {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

ZeroSoftLimitExecutor::ZeroSoftLimitExecutor(CanopenMaster* master,
                                             const CanopenMasterConfig* config)
    : config_(config), master_(master), sdo_accessor_(master) {
  ops_.read = [this](uint8_t node_id, uint16_t index, uint8_t subindex,
                     std::chrono::milliseconds timeout, std::size_t expected_size) {
    return sdo_accessor_.Read(node_id, index, subindex, timeout, expected_size);
  };
  ops_.write_u32 = [this](uint8_t node_id, uint16_t index, uint8_t subindex,
                          uint32_t value, std::chrono::milliseconds timeout) {
    return sdo_accessor_.WriteU32(node_id, index, subindex, value, timeout);
  };
}

ZeroSoftLimitExecutor::ZeroSoftLimitExecutor(const CanopenMasterConfig* config, Ops ops)
    : config_(config), ops_(std::move(ops)) {}

bool ZeroSoftLimitExecutor::SetCurrentPositionAsZero(std::size_t axis_index,
                                                     std::string* detail) {
  if (!ValidateAxis(axis_index, detail)) {
    return false;
  }

  if (!WriteU32(axis_index, kObj_HomeOffset, 0, 0u, "write 0x607C:00=0", detail)) {
    return false;
  }

  int32_t actual_position = 0;
  if (!ReadI32(axis_index, kObj_PositionActual, 0, &actual_position, "read 0x6064:00",
               detail)) {
    return false;
  }

  if (actual_position == std::numeric_limits<int32_t>::min()) {
    SetError(detail, "actual position overflow for home offset negation");
    return false;
  }

  const int32_t home_offset = -actual_position;
  if (!WriteU32(axis_index, kObj_HomeOffset, 0, static_cast<uint32_t>(home_offset),
                "write 0x607C:00=-actual_position", detail)) {
    return false;
  }

  if (!WriteU32(axis_index, kObj_StoreParameters, 1, kStoreSaveSignature,
                "write 0x1010:01='save'", detail)) {
    return false;
  }

  if (detail) {
    std::ostringstream oss;
    oss << "axis " << axis_index << " zero set, home_offset=" << home_offset;
    *detail = oss.str();
  }
  return true;
}

bool ZeroSoftLimitExecutor::ApplySoftLimitCounts(std::size_t axis_index, int32_t min_counts,
                                                 int32_t max_counts,
                                                 std::string* detail) {
  if (!ValidateAxis(axis_index, detail)) {
    return false;
  }
  if (min_counts > max_counts) {
    std::ostringstream oss;
    oss << "invalid limits for axis " << axis_index << ": min(" << min_counts
        << ") > max(" << max_counts << ")";
    SetError(detail, oss.str());
    return false;
  }

  if (!WriteU32(axis_index, kObj_SoftLimitState, 0, kSoftLimitEnableMagic,
                "write 0x2003:00 enable soft limit", detail)) {
    return false;
  }
  if (!WriteU32(axis_index, kObj_SoftwarePositionLimit, 2,
                static_cast<uint32_t>(max_counts), "write 0x607D:02 max", detail)) {
    return false;
  }
  if (!WriteU32(axis_index, kObj_SoftwarePositionLimit, 1,
                static_cast<uint32_t>(min_counts), "write 0x607D:01 min", detail)) {
    return false;
  }

  if (detail) {
    std::ostringstream oss;
    oss << "axis " << axis_index << " soft limits set: [" << min_counts << ", "
        << max_counts << "]";
    *detail = oss.str();
  }
  return true;
}

bool ZeroSoftLimitExecutor::ApplySoftLimitRadians(std::size_t axis_index, double min_rad,
                                                  double max_rad, std::string* detail) {
  if (!ValidateAxis(axis_index, detail)) {
    return false;
  }

  const auto& joint = config_->joints[axis_index];
  int32_t min_counts = 0;
  if (!RadToCounts(min_rad, joint.counts_per_rev, &min_counts, detail)) {
    if (detail) {
      *detail = "axis " + std::to_string(axis_index) + " min limit conversion failed: " +
                *detail;
    }
    return false;
  }
  int32_t max_counts = 0;
  if (!RadToCounts(max_rad, joint.counts_per_rev, &max_counts, detail)) {
    if (detail) {
      *detail = "axis " + std::to_string(axis_index) + " max limit conversion failed: " +
                *detail;
    }
    return false;
  }
  return ApplySoftLimitCounts(axis_index, min_counts, max_counts, detail);
}

bool ZeroSoftLimitExecutor::RadToCounts(double rad, double counts_per_rev, int32_t* out,
                                        std::string* error) {
  if (!out) {
    SetError(error, "out is null");
    return false;
  }
  if (!std::isfinite(rad)) {
    SetError(error, "rad is not finite");
    return false;
  }
  if (!std::isfinite(counts_per_rev) || counts_per_rev <= 0.0) {
    SetError(error, "counts_per_rev must be > 0");
    return false;
  }

  const double raw = rad * counts_per_rev / (2.0 * kPi);
  if (raw < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      raw > static_cast<double>(std::numeric_limits<int32_t>::max())) {
    SetError(error, "converted counts out of int32 range");
    return false;
  }
  *out = static_cast<int32_t>(std::llround(raw));
  return true;
}

bool ZeroSoftLimitExecutor::ValidateAxis(std::size_t axis_index,
                                         std::string* detail) const {
  if (!config_) {
    SetError(detail, "config is null");
    return false;
  }
  if (axis_index >= config_->joints.size()) {
    std::ostringstream oss;
    oss << "axis_index out of range: " << axis_index;
    SetError(detail, oss.str());
    return false;
  }
  if (!ops_.read || !ops_.write_u32) {
    SetError(detail, "sdo operations not initialized");
    return false;
  }
  if (master_ && !master_->running()) {
    SetError(detail, "master not running");
    return false;
  }
  return true;
}

bool ZeroSoftLimitExecutor::WriteU32(std::size_t axis_index, uint16_t index,
                                     uint8_t subindex, uint32_t value,
                                     const char* step, std::string* detail) {
  const uint8_t node_id = config_->joints[axis_index].node_id;
  const SdoResult r =
      ops_.write_u32(node_id, index, subindex, value, std::chrono::milliseconds(1000));
  if (r.ok) {
    return true;
  }
  std::ostringstream oss;
  oss << "axis " << axis_index << " node " << static_cast<int>(node_id) << ' ' << step
      << " failed";
  if (!r.error.empty()) {
    oss << ": " << r.error;
  }
  SetError(detail, oss.str());
  return false;
}

bool ZeroSoftLimitExecutor::ReadI32(std::size_t axis_index, uint16_t index, uint8_t subindex,
                                    int32_t* out, const char* step,
                                    std::string* detail) {
  if (!out) {
    SetError(detail, "out is null");
    return false;
  }
  const uint8_t node_id = config_->joints[axis_index].node_id;
  const SdoResult r =
      ops_.read(node_id, index, subindex, std::chrono::milliseconds(1000), 4);
  if (r.ok) {
    *out = r.as_i32();
    return true;
  }
  std::ostringstream oss;
  oss << "axis " << axis_index << " node " << static_cast<int>(node_id) << ' ' << step
      << " failed";
  if (!r.error.empty()) {
    oss << ": " << r.error;
  }
  SetError(detail, oss.str());
  return false;
}

void ZeroSoftLimitExecutor::SetError(std::string* detail, const std::string& message) {
  if (detail) {
    *detail = message;
  }
}

}  // namespace canopen_hw
