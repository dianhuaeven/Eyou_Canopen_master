#include <atomic>
#include <csignal>
#include <filesystem>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Trigger.h>

#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/lifecycle_manager.hpp"
#include "canopen_hw/logging.hpp"

namespace {

std::atomic<bool> g_run{true};

void HandleSignal(int) { g_run.store(false); }

std::string MakeAbsolutePath(const std::string& path) {
  if (path.empty()) return path;
  std::filesystem::path p(path);
  return p.is_absolute() ? p.string() : std::filesystem::absolute(p).string();
}

bool FileExists(const std::string& path) {
  return !path.empty() && std::filesystem::exists(path);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "canopen_hw_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  // 从 ROS 参数读取配置文件路径。
  std::string dcf_path, joints_path;
  pnh.param<std::string>("dcf_path", dcf_path, "config/master.dcf");
  pnh.param<std::string>("joints_path", joints_path, "config/joints.yaml");
  dcf_path = MakeAbsolutePath(dcf_path);
  joints_path = MakeAbsolutePath(joints_path);

  if (!FileExists(dcf_path)) {
    CANOPEN_LOG_ERROR("master_dcf_path not found: {}", dcf_path);
    return 1;
  }
  if (!FileExists(joints_path)) {
    CANOPEN_LOG_ERROR("joints.yaml not found: {}", joints_path);
    return 1;
  }

  // 解析配置（需要 joint names 来构造 ROS adapter）。
  canopen_hw::CanopenMasterConfig master_cfg;
  master_cfg.master_dcf_path = dcf_path;

  std::string error;
  if (!canopen_hw::LoadJointsYaml(joints_path, &error, &master_cfg)) {
    CANOPEN_LOG_ERROR("Load joints.yaml failed: {}", error);
    return 1;
  }

  std::vector<std::string> joint_names;
  joint_names.reserve(master_cfg.joints.size());
  for (const auto& jcfg : master_cfg.joints) {
    joint_names.push_back(jcfg.name);
  }

  // 通过 LifecycleManager 启动主站。
  canopen_hw::LifecycleManager lifecycle;
  if (!lifecycle.Init(master_cfg)) {
    return 1;
  }

  // ROS 适配层。
  canopen_hw::CanopenRobotHwRos robot_hw_ros(lifecycle.robot_hw(), joint_names);

  // 生命周期 services。
  auto halt_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                       std_srvs::Trigger::Response>(
      "halt", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = lifecycle.Halt();
        res.message = res.success ? "halted" : "halt failed";
        return true;
      });

  auto recover_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                          std_srvs::Trigger::Response>(
      "recover", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = lifecycle.Recover();
        res.message = res.success ? "recovered" : "recover failed";
        return true;
      });

  auto shutdown_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                           std_srvs::Trigger::Response>(
      "shutdown", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = lifecycle.Shutdown();
        res.message = res.success ? "shutdown" : "shutdown failed";
        g_run.store(false);
        return true;
      });

  controller_manager::ControllerManager cm(&robot_hw_ros, nh);

  double loop_hz = 100.0;
  pnh.param("loop_hz", loop_hz, 100.0);
  ros::Rate rate(loop_hz);
  ros::Time last_time = ros::Time::now();

  while (ros::ok() && g_run.load()) {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - last_time;
    last_time = now;

    robot_hw_ros.read(now, period);
    cm.update(now, period);
    robot_hw_ros.write(now, period);

    ros::spinOnce();
    rate.sleep();
  }

  lifecycle.Shutdown();
  return 0;
}
