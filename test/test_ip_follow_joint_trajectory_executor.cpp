#include <gtest/gtest.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "canopen_hw/ip_follow_joint_trajectory_executor.hpp"

namespace canopen_hw {
namespace {

control_msgs::FollowJointTrajectoryGoal MakeSingleJointGoal(
    const std::string& joint_name) {
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {joint_name};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = {1.0};
  point.time_from_start = ros::Duration(1.0);
  goal.trajectory.points.push_back(point);
  return goal;
}

TEST(IpFollowJointTrajectoryExecutor, RejectsWrongJointName) {
  auto goal = MakeSingleJointGoal("joint_x");
  std::string error;
  EXPECT_FALSE(IpFollowJointTrajectoryExecutor::ValidateGoal(
      goal, "joint_1", &error));
  EXPECT_EQ(error, "goal must target exactly one configured joint");
}

TEST(IpFollowJointTrajectoryExecutor, RejectsEmptyTrajectory) {
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"joint_1"};

  std::string error;
  EXPECT_FALSE(IpFollowJointTrajectoryExecutor::ValidateGoal(
      goal, "joint_1", &error));
  EXPECT_EQ(error, "goal contains no trajectory points");
}

TEST(IpFollowJointTrajectoryExecutor, AcceptsSingleJointGoal) {
  auto goal = MakeSingleJointGoal("joint_1");
  std::string error;
  EXPECT_TRUE(IpFollowJointTrajectoryExecutor::ValidateGoal(
      goal, "joint_1", &error));
  EXPECT_TRUE(error.empty());
}

}  // namespace
}  // namespace canopen_hw
