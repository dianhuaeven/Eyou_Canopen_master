#include <chrono>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "Eyou_Canopen_Master/SetMode.h"
#include "Eyou_Canopen_Master/SetZero.h"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/cia402_defs.hpp"
#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/lifecycle_manager.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/service_gateway.hpp"
#include "canopen_hw/urdf_joint_limits.hpp"
#include "canopen_hw/zero_soft_limit_executor.hpp"

namespace {

std::string MakeAbsolutePath(const std::string& path) {
  if (path.empty()) return path;
  std::filesystem::path p(path);
  return p.is_absolute() ? p.string() : std::filesystem::absolute(p).string();
}

bool FileExists(const std::string& path) {
  return !path.empty() && std::filesystem::exists(path);
}

bool IsAllowedMode(int8_t mode) {
  return mode == canopen_hw::kMode_IP || mode == canopen_hw::kMode_CSP ||
         mode == canopen_hw::kMode_CSV || mode == canopen_hw::kMode_CST;
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
  for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
    robot_hw_ros.SetMode(i, master_cfg.joints[i].default_mode);
  }
  std::mutex loop_mtx;
  canopen_hw::OperationalCoordinator coordinator(
      lifecycle.master(), lifecycle.shared_state(), master_cfg.joints.size());
  coordinator.SetConfigured();
  canopen_hw::ServiceGateway service_gateway(&pnh, &coordinator, &loop_mtx);
  canopen_hw::ZeroSoftLimitExecutor zero_soft_limit_executor(lifecycle.master(),
                                                             &master_cfg);
  std::vector<canopen_hw::JointLimitSpec> urdf_limits;
  bool urdf_limits_ready = false;

  auto ensure_urdf_limits = [&](std::string* detail) -> bool {
    if (urdf_limits_ready) {
      return true;
    }
    std::string urdf_xml;
    if (!nh.getParam("robot_description", urdf_xml) || urdf_xml.empty()) {
      if (detail) {
        *detail = "robot_description is missing; cannot derive URDF soft limits";
      }
      return false;
    }
    std::string parse_error;
    if (!canopen_hw::ParseUrdfJointLimits(urdf_xml, master_cfg.joints, &urdf_limits,
                                          &parse_error)) {
      if (detail) {
        *detail = parse_error.empty() ? "failed to parse URDF joint limits"
                                      : parse_error;
      }
      return false;
    }
    urdf_limits_ready = true;
    return true;
  };

  auto apply_soft_limit_axis = [&](std::size_t axis_index,
                                   std::string* detail) -> bool {
    if (!ensure_urdf_limits(detail)) {
      return false;
    }
    if (axis_index >= urdf_limits.size()) {
      if (detail) {
        std::ostringstream oss;
        oss << "axis " << axis_index << " out of URDF limit range";
        *detail = oss.str();
      }
      return false;
    }
    const auto& limit = urdf_limits[axis_index];
    if (limit.unit == canopen_hw::UrdfJointLimitUnit::kRadians) {
      return zero_soft_limit_executor.ApplySoftLimitRadians(
          axis_index, limit.lower, limit.upper, detail);
    }
    if (limit.unit == canopen_hw::UrdfJointLimitUnit::kMeters) {
      return zero_soft_limit_executor.ApplySoftLimitMeters(
          axis_index, limit.lower, limit.upper, detail);
    }
    if (detail) {
      *detail = "unsupported URDF limit unit";
    }
    return false;
  };

  struct PreparedSoftLimit {
    int32_t min_counts = 0;
    int32_t max_counts = 0;
  };

  auto prepare_soft_limit_axis = [&](std::size_t axis_index, PreparedSoftLimit* out,
                                     std::string* detail) -> bool {
    if (!out) {
      if (detail) {
        *detail = "prepared soft limit output is null";
      }
      return false;
    }
    if (!ensure_urdf_limits(detail)) {
      return false;
    }
    if (axis_index >= urdf_limits.size() || axis_index >= master_cfg.joints.size()) {
      if (detail) {
        std::ostringstream oss;
        oss << "axis " << axis_index << " out of URDF limit range";
        *detail = oss.str();
      }
      return false;
    }

    const auto& limit = urdf_limits[axis_index];
    if (limit.unit == canopen_hw::UrdfJointLimitUnit::kRadians) {
      return zero_soft_limit_executor.PrepareSoftLimitRadians(
          axis_index, limit.lower, limit.upper, &out->min_counts, &out->max_counts,
          detail);
    }
    if (limit.unit == canopen_hw::UrdfJointLimitUnit::kMeters) {
      return zero_soft_limit_executor.PrepareSoftLimitMeters(
          axis_index, limit.lower, limit.upper, &out->min_counts, &out->max_counts,
          detail);
    }
    if (detail) {
      *detail = "unsupported URDF limit unit";
    }
    return false;
  };

  auto apply_soft_limit_all = [&](std::string* detail) -> bool {
    if (!master_cfg.auto_write_soft_limits_from_urdf) {
      return true;
    }
    if (!ensure_urdf_limits(detail)) {
      return false;
    }
    for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
      std::string axis_detail;
      if (!apply_soft_limit_axis(i, &axis_detail)) {
        if (detail) {
          std::ostringstream oss;
          oss << "apply soft limit failed at axis " << i;
          if (!axis_detail.empty()) {
            oss << ": " << axis_detail;
          }
          *detail = oss.str();
        }
        return false;
      }
    }
    if (detail) {
      *detail = "soft limits applied from URDF";
    }
    return true;
  };

  service_gateway.SetPostInitHook(
      [&](std::string* detail) { return apply_soft_limit_all(detail); });

  bool auto_init = false;
  bool auto_enable = false;
  bool auto_release = false;
  bool use_ip_executor = false;
  double ip_executor_rate_hz = master_cfg.loop_hz;
  std::string ip_executor_action_ns =
      "arm_position_controller/follow_joint_trajectory";
  pnh.param("auto_init", auto_init, false);
  pnh.param("auto_enable", auto_enable, false);
  pnh.param("auto_release", auto_release, false);
  pnh.param("use_ip_executor", use_ip_executor, false);
  pnh.param("ip_executor_rate_hz", ip_executor_rate_hz, ip_executor_rate_hz);
  pnh.param("ip_executor_action_ns", ip_executor_action_ns,
            ip_executor_action_ns);

  std::unique_ptr<canopen_hw::IpFollowJointTrajectoryExecutor> ip_executor;
  if (use_ip_executor) {
    canopen_hw::IpFollowJointTrajectoryExecutor::Config exec_cfg;
    exec_cfg.joint_names.clear();
    exec_cfg.joint_indices.clear();
    exec_cfg.max_velocities.clear();
    exec_cfg.max_accelerations.clear();
    exec_cfg.max_jerks.clear();
    exec_cfg.goal_tolerances.clear();
    exec_cfg.action_ns = ip_executor_action_ns;
    exec_cfg.command_rate_hz = ip_executor_rate_hz;
    exec_cfg.joint_names.reserve(master_cfg.joints.size());
    exec_cfg.joint_indices.reserve(master_cfg.joints.size());
    exec_cfg.max_velocities.reserve(master_cfg.joints.size());
    exec_cfg.max_accelerations.reserve(master_cfg.joints.size());
    exec_cfg.max_jerks.reserve(master_cfg.joints.size());
    exec_cfg.goal_tolerances.reserve(master_cfg.joints.size());

    for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
      const auto& jcfg = master_cfg.joints[i];
      exec_cfg.joint_names.push_back(jcfg.name);
      exec_cfg.joint_indices.push_back(i);
      exec_cfg.max_velocities.push_back(jcfg.ip_max_velocity);
      exec_cfg.max_accelerations.push_back(jcfg.ip_max_acceleration);
      exec_cfg.max_jerks.push_back(jcfg.ip_max_jerk);
      exec_cfg.goal_tolerances.push_back(jcfg.ip_goal_tolerance);
    }

    ip_executor = std::make_unique<canopen_hw::IpFollowJointTrajectoryExecutor>(
        &pnh, &robot_hw_ros, &loop_mtx, std::move(exec_cfg));
  }

  if ((auto_enable || auto_release) && !auto_init) {
    CANOPEN_LOG_ERROR(
        "auto_enable/auto_release require auto_init=true for strict transition path");
    return 1;
  }
  if (auto_release && !auto_enable) {
    CANOPEN_LOG_ERROR("auto_release requires auto_enable=true");
    return 1;
  }

  auto set_mode_srv = pnh.advertiseService<Eyou_Canopen_Master::SetMode::Request,
                                           Eyou_Canopen_Master::SetMode::Response>(
      "set_mode", [&](Eyou_Canopen_Master::SetMode::Request& req,
                      Eyou_Canopen_Master::SetMode::Response& res) {
        {
          std::lock_guard<std::mutex> lk(loop_mtx);
          const auto mode = coordinator.mode();
          if (mode != canopen_hw::SystemOpMode::Standby) {
            res.success = false;
            res.message = "set_mode only allowed in Standby; call ~/disable first";
            return true;
          }
          if (req.axis_index >= robot_hw_ros.axis_count()) {
            res.success = false;
            res.message = "axis_index out of range";
            return true;
          }
          if (!IsAllowedMode(req.mode)) {
            res.success = false;
            res.message =
                "unsupported mode: " + std::to_string(req.mode) +
                ", allowed: 7(IP),8(CSP),9(CSV),10(CST)";
            return true;
          }

          robot_hw_ros.SetMode(req.axis_index, req.mode);
        }

        res.success = true;
        res.message = "mode set";
        return true;
      });

  auto set_zero_srv = pnh.advertiseService<Eyou_Canopen_Master::SetZero::Request,
                                           Eyou_Canopen_Master::SetZero::Response>(
      "set_zero", [&](Eyou_Canopen_Master::SetZero::Request& req,
                      Eyou_Canopen_Master::SetZero::Response& res) {
        std::lock_guard<std::mutex> lk(loop_mtx);
        const auto mode = coordinator.mode();
        if (mode != canopen_hw::SystemOpMode::Standby) {
          res.success = false;
          res.message = "set_zero only allowed in Standby; call ~/disable first";
          return true;
        }
        if (req.axis_index >= master_cfg.joints.size()) {
          res.success = false;
          res.message = "axis_index out of range";
          return true;
        }

        std::string detail;
        PreparedSoftLimit prepared_soft_limit;
        int32_t previous_home_offset = 0;
        if (master_cfg.auto_write_soft_limits_from_urdf) {
          if (!prepare_soft_limit_axis(req.axis_index, &prepared_soft_limit, &detail)) {
            res.success = false;
            res.message = detail.empty() ? "soft limit precheck failed" : detail;
            return true;
          }
          if (!zero_soft_limit_executor.ReadHomeOffset(req.axis_index,
                                                       &previous_home_offset,
                                                       &detail)) {
            res.success = false;
            res.message = detail.empty() ? "read home offset failed" : detail;
            return true;
          }
        }

        if (!zero_soft_limit_executor.SetCurrentPositionAsZero(req.axis_index,
                                                               &detail)) {
          res.success = false;
          res.message = detail.empty() ? "set_zero failed" : detail;
          return true;
        }

        if (master_cfg.auto_write_soft_limits_from_urdf) {
          if (!zero_soft_limit_executor.ApplySoftLimitCounts(
                  req.axis_index, prepared_soft_limit.min_counts,
                  prepared_soft_limit.max_counts, &detail)) {
            const std::string soft_limit_error =
                detail.empty() ? "soft limit rewrite failed" : detail;
            std::string rollback_detail;
            if (!zero_soft_limit_executor.RestoreHomeOffset(
                    req.axis_index, previous_home_offset, &rollback_detail)) {
              res.success = false;
              res.message =
                  "zero set and soft limit rewrite failed; rollback failed: " +
                  soft_limit_error;
              if (!rollback_detail.empty()) {
                res.message += "; " + rollback_detail;
              }
              return true;
            }
            res.success = false;
            res.message =
                "soft limit rewrite failed after zeroing; restored previous home offset: " +
                soft_limit_error;
            return true;
          }
        }

        res.success = true;
        res.message = detail.empty() ? "zero set" : detail;
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
    const auto init_result = coordinator.RequestInit();
    if (!init_result.ok) {
      CANOPEN_LOG_ERROR("auto_init enabled but init failed: {}",
                        init_result.message);
      return 1;
    }
    std::string soft_limit_detail;
    if (!apply_soft_limit_all(&soft_limit_detail)) {
      const auto shutdown_result = coordinator.RequestShutdown();
      CANOPEN_LOG_ERROR("auto_init post-init soft limit apply failed: {}",
                        soft_limit_detail);
      if (!shutdown_result.ok) {
        CANOPEN_LOG_ERROR("rollback shutdown failed after post-init failure: {}",
                          shutdown_result.message);
      }
      return 1;
    }
    if (auto_enable) {
      const auto enable_result = coordinator.RequestEnable();
      if (!enable_result.ok) {
        CANOPEN_LOG_ERROR("auto_enable enabled but enable failed: {}",
                          enable_result.message);
        return 1;
      }
    }
    if (auto_release) {
      const auto release_result = coordinator.RequestRelease();
      if (!release_result.ok) {
        CANOPEN_LOG_ERROR("auto_release enabled but release failed: {}",
                          release_result.message);
        return 1;
      }
    }
  }

  double loop_hz = master_cfg.loop_hz;
  pnh.param("loop_hz", loop_hz, loop_hz);  // launch 参数可覆盖 yaml 值。
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
      coordinator.UpdateFromFeedback();
      coordinator.ComputeIntents();
      robot_hw_ros.read(now, period);
      cm.update(now, period);
      if (ip_executor) {
        ip_executor->update(now, period);
      }
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
