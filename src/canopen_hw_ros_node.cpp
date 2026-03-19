#include <atomic>
#include <csignal>
#include <filesystem>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/shared_state.hpp"

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

  // 从 ROS 参数或默认路径读取配置文件。
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

  canopen_hw::CanopenMasterConfig master_cfg;
  master_cfg.master_dcf_path = dcf_path;

  std::string error;
  if (!canopen_hw::LoadJointsYaml(joints_path, &error, &master_cfg)) {
    CANOPEN_LOG_ERROR("Load joints.yaml failed: {}", error);
    return 1;
  }

  if (master_cfg.axis_count > canopen_hw::SharedState::kMaxAxisCount) {
    CANOPEN_LOG_ERROR("axis_count exceeds maximum: {} > {}",
                      master_cfg.axis_count,
                      canopen_hw::SharedState::kMaxAxisCount);
    return 1;
  }

  // 从配置中提取 joint name 列表。
  std::vector<std::string> joint_names;
  joint_names.reserve(master_cfg.joints.size());
  for (const auto& jcfg : master_cfg.joints) {
    joint_names.push_back(jcfg.name);
  }

  canopen_hw::SharedState shared_state(master_cfg.axis_count);
  canopen_hw::CanopenRobotHw robot_hw(&shared_state);
  robot_hw.ApplyConfig(master_cfg);

  // ROS 适配层: 注册 JointStateInterface + PositionJointInterface。
  canopen_hw::CanopenRobotHwRos robot_hw_ros(&robot_hw, joint_names);

  canopen_hw::CanopenMaster master(master_cfg, &shared_state);
  if (!master.Start()) {
    return 1;
  }

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

    rate.sleep();
  }

  master.Stop();
  return 0;
}
