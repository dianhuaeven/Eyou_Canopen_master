#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "Eyou_Canopen_Master/SetMode.h"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/lifecycle_manager.hpp"
#include "canopen_hw/logging.hpp"

namespace {

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

  // 先进入 Configured，不自动初始化电机。
  canopen_hw::LifecycleManager lifecycle;
  if (!lifecycle.Configure(master_cfg)) {
    return 1;
  }

  // ROS 适配层。
  canopen_hw::CanopenRobotHwRos robot_hw_ros(lifecycle.robot_hw(), joint_names);
  std::mutex loop_mtx;

  bool auto_init = false;
  pnh.param("auto_init", auto_init, false);

  // 生命周期 services。
  auto init_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                       std_srvs::Trigger::Response>(
      "init", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lk(loop_mtx);
        if (lifecycle.state() == canopen_hw::LifecycleState::Active) {
          res.success = true;
          res.message = "already initialized";
          return true;
        }
        res.success = lifecycle.InitMotors();
        res.message = res.success ? "initialized" : "init failed";
        return true;
      });

  auto halt_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                       std_srvs::Trigger::Response>(
      "halt", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lk(loop_mtx);
        res.success = lifecycle.Halt();
        res.message = res.success ? "halted" : "halt failed";
        return true;
      });

  auto recover_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                          std_srvs::Trigger::Response>(
      "recover", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lk(loop_mtx);
        if (lifecycle.state() == canopen_hw::LifecycleState::Configured &&
            !lifecycle.ever_initialized()) {
          res.success = false;
          res.message = "recover unavailable before init; call ~/init first";
          return true;
        }
        res.success = lifecycle.Recover();
        res.message = res.success ? "recovered" : "recover failed";
        return true;
      });

  auto shutdown_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                           std_srvs::Trigger::Response>(
      "shutdown", [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        {
          std::lock_guard<std::mutex> lk(loop_mtx);
          res.success = lifecycle.Shutdown();
        }
        res.message = res.success ? "shutdown" : "shutdown failed";
        ros::shutdown();
        return true;
      });

  auto set_mode_srv = pnh.advertiseService<Eyou_Canopen_Master::SetMode::Request,
                                           Eyou_Canopen_Master::SetMode::Response>(
      "set_mode", [&](Eyou_Canopen_Master::SetMode::Request& req,
                      Eyou_Canopen_Master::SetMode::Response& res) {
        std::lock_guard<std::mutex> lk(loop_mtx);
        if (lifecycle.state() == canopen_hw::LifecycleState::Active) {
          res.success = false;
          res.message = "set_mode not allowed in Active state; call ~/halt first";
          return true;
        }
        if (req.axis_index >= robot_hw_ros.axis_count()) {
          res.success = false;
          res.message = "axis_index out of range";
          return true;
        }
        robot_hw_ros.SetMode(req.axis_index, req.mode);
        res.success = true;
        res.message = "mode set";
        return true;
      });

  controller_manager::ControllerManager cm(&robot_hw_ros, nh);

  // Diagnostics。
  diagnostic_updater::Updater diag_updater;
  diag_updater.setHardwareID("canopen_hw");

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    diag_updater.add(joint_names[i], [&lifecycle, i](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      const auto* counters = lifecycle.master() ? lifecycle.master()->GetHealthCounters(i) : nullptr;
      canopen_hw::AxisFeedback fb;
      bool got_fb = lifecycle.master() && lifecycle.master()->GetAxisFeedback(i, &fb);

      if (!counters || !got_fb) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "no data");
        return;
      }

      stat.add("heartbeat_lost", counters->heartbeat_lost.load());
      stat.add("emcy_count", counters->emcy_count.load());
      stat.add("fault_reset_attempts", counters->fault_reset_attempts.load());
      stat.add("is_operational", fb.is_operational);
      stat.add("is_fault", fb.is_fault);
      stat.add("heartbeat_lost_flag", fb.heartbeat_lost);

      if (fb.is_fault) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "fault");
      } else if (fb.heartbeat_lost) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "heartbeat lost");
      } else if (!fb.is_operational) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "not operational");
      } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "operational");
      }
    });
  }

  if (auto_init) {
    std::lock_guard<std::mutex> lk(loop_mtx);
    if (!lifecycle.InitMotors()) {
      CANOPEN_LOG_ERROR("auto_init enabled but InitMotors failed");
      return 1;
    }
  }

  double loop_hz = 100.0;
  pnh.param("loop_hz", loop_hz, 100.0);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(loop_hz);
  ros::Time last_time = ros::Time::now();

  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - last_time;
    last_time = now;

    {
      std::lock_guard<std::mutex> lk(loop_mtx);
      robot_hw_ros.read(now, period);
      cm.update(now, period);
      robot_hw_ros.write(now, period);
      diag_updater.update();
    }
    rate.sleep();
  }

  spinner.stop();
  {
    std::lock_guard<std::mutex> lk(loop_mtx);
    lifecycle.Shutdown();
  }
  return 0;
}
