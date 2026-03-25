#include "canopen_hw/ip_follow_joint_trajectory_executor.hpp"

#include <utility>

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, CanopenRobotHwRos* hw, std::mutex* loop_mtx)
    : IpFollowJointTrajectoryExecutor(pnh, hw, loop_mtx, Config{}) {}

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, CanopenRobotHwRos* hw, std::mutex* loop_mtx,
    Config config)
    : hw_(hw), loop_mtx_(loop_mtx), config_(std::move(config)) {
  if (pnh == nullptr) {
    return;
  }

  server_ = std::make_unique<Server>(
      *pnh, config_.action_ns,
      [this](const GoalConstPtr& goal) { ExecuteGoal(goal); }, false);
  server_->start();
}

void IpFollowJointTrajectoryExecutor::update(const ros::Time& /*now*/,
                                             const ros::Duration& /*period*/) {}

bool IpFollowJointTrajectoryExecutor::ValidateGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::string& joint_name, std::string* error) {
  if (error) {
    error->clear();
  }

  if (goal.trajectory.joint_names.size() != 1u ||
      goal.trajectory.joint_names.front() != joint_name) {
    if (error) {
      *error = "goal must target exactly one configured joint";
    }
    return false;
  }

  if (goal.trajectory.points.empty()) {
    if (error) {
      *error = "goal contains no trajectory points";
    }
    return false;
  }

  return true;
}

void IpFollowJointTrajectoryExecutor::ExecuteGoal(const GoalConstPtr& goal) {
  if (!server_) {
    return;
  }

  std::string error;
  if (!goal || !ValidateGoal(*goal, config_.joint_name, &error)) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code =
        control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = error.empty() ? "invalid goal" : error;
    server_->setAborted(result, result.error_string);
    return;
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code =
      control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
  result.error_string = "IP executor skeleton is not wired into runtime yet";
  CANOPEN_LOG_WARN("IpFollowJointTrajectoryExecutor: received goal for {}, "
                   "but runtime execution is not enabled yet",
                   config_.joint_name);
  server_->setAborted(result, result.error_string);
}

}  // namespace canopen_hw
