#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

enum class SystemOpMode : uint8_t {
  Inactive,
  Configured,
  Standby,
  Armed,
  Running,
  Faulted,
  Recovering,
  ShuttingDown,
};

const char* SystemOpModeName(SystemOpMode mode);

class OperationalCoordinator {
 public:
  struct Result {
    bool ok = false;
    std::string message;
  };

  OperationalCoordinator(CanopenMaster* master, SharedState* shared_state,
                         std::size_t axis_count);

  SystemOpMode mode() const { return mode_.load(std::memory_order_acquire); }

  // Configure 成功后调用，将协同器置为 Configured 起点。
  void SetConfigured();

  // 阶段 1 骨架：内部仍复用现有 CanopenMaster 的高层 API。
  Result RequestInit();
  Result RequestEnable();
  Result RequestRelease();
  Result RequestHalt();
  Result RequestRecover();
  Result RequestShutdown();

  // 根据当前模式计算并下发每轴 AxisIntent。
  void ComputeIntents();

  // 由主循环周期调用，自动将 Armed/Running 下的故障降级到 Faulted。
  void UpdateFromFeedback();

 private:
  Result DoTransition(std::initializer_list<SystemOpMode> allowed_from,
                      SystemOpMode to,
                      std::function<bool(std::string*)> action = nullptr);

  std::atomic<SystemOpMode> mode_{SystemOpMode::Inactive};
  mutable std::mutex transition_mtx_;

  CanopenMaster* master_ = nullptr;
  SharedState* shared_state_ = nullptr;
  std::size_t axis_count_ = 0;
};

}  // namespace canopen_hw
