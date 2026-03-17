#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <lely/coapp/master.hpp>

#include "canopen_hw/axis_driver.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// 主站基础配置。当前 commit 仅保留最小字段，后续再扩展到完整 YAML 参数。
struct CanopenMasterConfig {
  std::string can_interface = "can0";
  std::string master_dcf_path;
  uint8_t master_node_id = 127;
  std::size_t axis_count = 6;
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
  CanopenMasterConfig config_;
  SharedState* shared_state_ = nullptr;  // 非拥有指针。
  std::atomic<bool> running_{false};

  // 每轴一个 driver，索引与轴号一一对应(0 -> node_id 1)。
  // 约束: 容量在初始化阶段 reserve，运行阶段只读/不扩容。
  std::vector<std::unique_ptr<AxisDriver>> axis_drivers_;
};

}  // namespace canopen_hw
