#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include "canopen_hw/canopen_robot_hw_ros.hpp"

namespace canopen_hw {

class IpFollowJointTrajectoryExecutor {
 public:
  struct Config {
    Config() = default;

    std::string action_ns{"ip_follow_joint_trajectory"};
    std::string joint_name{"joint_1"};
    double command_rate_hz{100.0};
  };

  IpFollowJointTrajectoryExecutor(ros::NodeHandle* pnh, CanopenRobotHwRos* hw,
                                  std::mutex* loop_mtx);
  IpFollowJointTrajectoryExecutor(ros::NodeHandle* pnh, CanopenRobotHwRos* hw,
                                  std::mutex* loop_mtx, Config config);

  void update(const ros::Time& now, const ros::Duration& period);
  bool enabled() const { return server_ != nullptr; }

  static bool ValidateGoal(
      const control_msgs::FollowJointTrajectoryGoal& goal,
      const std::string& joint_name, std::string* error);

 private:
  using Action = control_msgs::FollowJointTrajectoryAction;
  using GoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;
  using Server = actionlib::SimpleActionServer<Action>;

  void ExecuteGoal(const GoalConstPtr& goal);

  CanopenRobotHwRos* hw_ = nullptr;
  std::mutex* loop_mtx_ = nullptr;
  Config config_;
  std::unique_ptr<Server> server_;
};

}  // namespace canopen_hw
