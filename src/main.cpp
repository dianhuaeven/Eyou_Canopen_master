#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/shared_state.hpp"

namespace {

std::atomic<bool> g_run{true};

void HandleSignal(int) {
  g_run.store(false);
}

}  // namespace

int main() {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  canopen_hw::SharedState shared_state;

  canopen_hw::CanopenMasterConfig master_cfg;
  master_cfg.can_interface = "can0";
  master_cfg.master_node_id = 127;
  master_cfg.axis_count = 6;
  master_cfg.master_dcf_path = "config/master.dcf";

  canopen_hw::CanopenMaster master(master_cfg, &shared_state);
  canopen_hw::CanopenRobotHw robot_hw(&shared_state);

  if (!master.Start()) {
    return 1;
  }

  // 应用主循环骨架: 100Hz 执行 read/write。
  // 运行期约束:
  // - 循环内禁止任何动态内存分配(不 new、不扩容容器、不构造大临时对象)。
  // - 所有资源在进入循环前初始化完成。
  // 后续接入 ROS controller_manager 后，这里将替换为 ros::Rate + cm.update()。
  const auto period = std::chrono::milliseconds(10);
  while (g_run.load()) {
    robot_hw.ReadFromSharedState();
    robot_hw.WriteToSharedState();
    std::this_thread::sleep_for(period);
  }

  master.Stop();
  return 0;
}
