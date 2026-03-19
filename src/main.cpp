#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <string>
#include <thread>

#include "canopen_hw/lifecycle_manager.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/realtime_loop.hpp"

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
  const std::string dcf_path = MakeAbsolutePath(opts.dcf_path);
  const std::string joints_path = MakeAbsolutePath(opts.joints_path);

  if (!FileExists(dcf_path)) {
    CANOPEN_LOG_ERROR("master_dcf_path not found: {}", dcf_path);
    return 1;
  }
  if (!FileExists(joints_path)) {
    CANOPEN_LOG_ERROR("joints.yaml not found: {}", joints_path);
    return 1;
  }

  canopen_hw::LifecycleManager lifecycle;

  if (!lifecycle.Init(dcf_path, joints_path)) {
    return 1;
  }

  // 应用主循环: 100Hz 执行 read/write。
  // 使用 RealtimeLoop 的 clock_nanosleep 绝对时间等待，消除累积漂移。
  canopen_hw::RealtimeLoop::Config loop_config;
  loop_config.period = std::chrono::milliseconds(10);
  canopen_hw::RealtimeLoop loop(loop_config);

  loop.Run([&]() -> bool {
    if (!g_run.load()) return false;
    lifecycle.robot_hw()->ReadFromSharedState();
    lifecycle.robot_hw()->WriteToSharedState();
    return true;
  });

  lifecycle.Shutdown();
  return 0;
}
