#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>

#include "canopen_hw/axis_driver.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// 主站基础配置。当前 commit 仅保留最小字段，后续再扩展到完整 YAML 参数。
struct CanopenMasterConfig {
  std::string can_interface = "can0";
  std::string master_dcf_path;
  uint8_t master_node_id = 127;
  std::size_t axis_count = 6;
  std::vector<uint8_t> node_ids;
  std::vector<bool> verify_pdo_mapping;
  std::vector<int32_t> position_lock_thresholds;
  std::vector<int> max_fault_resets;
  std::vector<int> fault_reset_hold_cycles;
};

class CanopenMaster {
 public:
  explicit CanopenMaster(const CanopenMasterConfig& config,
                         SharedState* shared_state);

  // 启动主站骨架:
  // - 标记运行状态
  // - 为后续 Lely 初始化预留入口
  // 返回 true 表示流程进入“已启动”状态。
  bool Start();

  // 停止主站骨架，释放已创建的轴驱动。
  void Stop();

  bool running() const { return running_.load(); }
  std::size_t axis_count() const { return axis_drivers_.size(); }
  const CanopenMasterConfig& config() const { return config_; }

  // 供后续真实 master 初始化后调用:
  // 基于 node-id 1..N 创建 AxisDriver。
  // 约束: 该函数只能在初始化阶段调用，禁止在运行循环中调用。
  void CreateAxisDrivers(lely::canopen::BasicMaster& master);

 private:
  bool GracefulShutdown();
  bool WaitForAllState(CiA402State target_state,
                       std::chrono::steady_clock::time_point deadline);

  CanopenMasterConfig config_;
  SharedState* shared_state_ = nullptr;  // 非拥有指针。
  std::atomic<bool> running_{false};

  std::unique_ptr<lely::io::IoGuard> io_guard_;
  std::unique_ptr<lely::io::Context> io_ctx_;
  std::unique_ptr<lely::io::Poll> io_poll_;
  std::unique_ptr<lely::ev::Loop> ev_loop_;
  std::unique_ptr<lely::io::Timer> io_timer_;
  std::unique_ptr<lely::io::CanController> can_ctrl_;
  std::unique_ptr<lely::io::CanChannel> can_chan_;
  std::unique_ptr<lely::canopen::AsyncMaster> master_;
  std::thread ev_thread_;

  // 每轴一个 driver，索引与轴号一一对应(0 -> node_id 1)。
  // 约束: 容量在初始化阶段 reserve，运行阶段只读/不扩容。
  std::vector<std::unique_ptr<AxisDriver>> axis_drivers_;
};

}  // namespace canopen_hw
