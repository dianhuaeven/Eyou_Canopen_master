#include "canopen_hw/canopen_master.hpp"

#include <algorithm>

namespace canopen_hw {

CanopenMaster::CanopenMaster(const CanopenMasterConfig& config,
                             SharedState* shared_state)
    : config_(config), shared_state_(shared_state) {
  if (config_.axis_count == 0) {
    config_.axis_count = 1;
  }
  // 预分配驱动容器容量，保证运行阶段不会因为扩容触发堆分配。
  axis_drivers_.reserve(config_.axis_count);
}

bool CanopenMaster::Start() {
  if (running_.load()) {
    return true;
  }

  // TODO(commit 8+): 在这里初始化 Lely IO 组件并创建 AsyncMaster:
  // 1) io::IoGuard / io::Context / io::Poll / ev::Loop
  // 2) io::Timer + io::CanChannel(can_interface)
  // 3) AsyncMaster(dcf_txt, concise_dcf, node_id)
  // 4) 调用 CreateAxisDrivers(master) 注册 6 轴驱动
  //
  // 当前阶段先交付可编译的生命周期骨架，避免一次引入过多变量。
  running_.store(true);
  return true;
}

void CanopenMaster::Stop() {
  if (!running_.load()) {
    return;
  }

  // 先释放驱动对象，确保不会再触发回调访问 shared_state。
  axis_drivers_.clear();
  running_.store(false);
}

void CanopenMaster::CreateAxisDrivers(lely::canopen::BasicMaster& master) {
  // 初始化阶段函数: clear 后重新 emplace，不会超过预留容量。
  axis_drivers_.clear();

  for (std::size_t i = 0; i < config_.axis_count; ++i) {
    // 轴索引 i -> node_id i+1。后续可改为从 joints.yaml 读取映射。
    const uint8_t node_id = static_cast<uint8_t>(i + 1);
    axis_drivers_.emplace_back(
        std::make_unique<AxisDriver>(master, node_id, i, shared_state_));
  }
}

}  // namespace canopen_hw
