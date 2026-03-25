#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/ruckig.hpp>

#include "canopen_hw/canopen_robot_hw_ros.hpp"

namespace canopen_hw {

class IpFollowJointTrajectoryExecutor {
 public:
  struct State {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
  };

  enum class StepStatus {
    kIdle,
    kWorking,
    kFinished,
    kError,
  };

  struct Config {
    Config() = default;

    std::string action_ns{"ip_follow_joint_trajectory"};
    std::string joint_name{"joint_1"};
    double command_rate_hz{100.0};
    double max_velocity{1.0};
    double max_acceleration{2.0};
    double max_jerk{10.0};
    double goal_tolerance{1e-3};
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

  bool startGoal(const control_msgs::FollowJointTrajectoryGoal& goal,
                 const State& actual, std::string* error);
  void cancelGoal();
  StepStatus step(const State& actual, State* command, std::string* error);
  bool hasActiveGoal() const;

 private:
  using Action = control_msgs::FollowJointTrajectoryAction;
  using GoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;
  using Server = actionlib::SimpleActionServer<Action>;

  void ExecuteGoal(const GoalConstPtr& goal);

  CanopenRobotHwRos* hw_ = nullptr;
  std::mutex* loop_mtx_ = nullptr;
  Config config_;
  std::unique_ptr<Server> server_;

  mutable std::mutex exec_mtx_;
  std::optional<control_msgs::FollowJointTrajectoryGoal> active_goal_;
  ruckig::Ruckig<1> otg_;
  ruckig::InputParameter<1> input_;
  ruckig::OutputParameter<1> output_;
};

}  // namespace canopen_hw
