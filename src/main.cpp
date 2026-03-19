#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <string>
#include <thread>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/shared_state.hpp"

namespace {

std::atomic<bool> g_run{true};

void HandleSignal(int) {
  g_run.store(false);
}

struct StartupOptions {
  std::string dcf_path = "config/master.dcf";
  std::string joints_path = "config/joints.yaml";
};

StartupOptions ParseArgs(int argc, char** argv) {
  StartupOptions opts;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--dcf" && i + 1 < argc) {
      opts.dcf_path = argv[++i];
    } else if (arg == "--joints" && i + 1 < argc) {
      opts.joints_path = argv[++i];
    }
  }
  return opts;
}

std::string MakeAbsolutePath(const std::string& path) {
  if (path.empty()) {
    return path;
  }
  std::filesystem::path p(path);
  if (p.is_absolute()) {
    return p.string();
  }
  return std::filesystem::absolute(p).string();
}

bool FileExists(const std::string& path) {
  return !path.empty() && std::filesystem::exists(path);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  const StartupOptions opts = ParseArgs(argc, argv);
  const std::string joints_path = MakeAbsolutePath(opts.joints_path);

  canopen_hw::CanopenMasterConfig master_cfg;
  master_cfg.can_interface = "can0";
  master_cfg.master_node_id = 127;
  master_cfg.axis_count = 6;
  master_cfg.master_dcf_path = MakeAbsolutePath(opts.dcf_path);
  if (!FileExists(master_cfg.master_dcf_path)) {
    CANOPEN_LOG_ERROR("master_dcf_path not found: {}",
                      master_cfg.master_dcf_path);
    return 1;
  }

  // 先用临时 SharedState 解析 joints.yaml 以确定轴数和参数。
  // LoadJointsYaml 需要 CanopenRobotHw 指针来配置单位换算，
  // 但此时还不知道最终轴数，所以先用默认值构造。
  canopen_hw::SharedState temp_state;
  canopen_hw::CanopenRobotHw temp_hw(&temp_state);

  if (!FileExists(joints_path)) {
    CANOPEN_LOG_WARN("joints.yaml not found: {}", joints_path);
  } else {
    std::string error;
    if (!canopen_hw::LoadJointsYaml(joints_path, &temp_hw, &error,
                                    &master_cfg)) {
      CANOPEN_LOG_ERROR("Load joints.yaml failed: {}", error);
    } else {
      CANOPEN_LOG_INFO("Loaded config: interface={} master_node_id={}",
                       master_cfg.can_interface,
                       static_cast<int>(master_cfg.master_node_id));
    }
  }

  if (master_cfg.axis_count > canopen_hw::SharedState::kMaxAxisCount) {
    CANOPEN_LOG_ERROR("configured axis_count exceeds supported maximum: {} > {}",
                      master_cfg.axis_count,
                      canopen_hw::SharedState::kMaxAxisCount);
    return 1;
  }

  // 用确定的轴数构造正式的 SharedState 和 RobotHw。
  canopen_hw::SharedState shared_state(master_cfg.axis_count);
  canopen_hw::CanopenRobotHw robot_hw(&shared_state);
  if (FileExists(joints_path)) {
    canopen_hw::LoadJointsYaml(joints_path, &robot_hw, nullptr, nullptr);
  }

  canopen_hw::CanopenMaster master(master_cfg, &shared_state);

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
