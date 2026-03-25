#include <gtest/gtest.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/time.h>
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

TEST(IpFollowJointTrajectoryExecutor, StartsFromActualState) {
  IpFollowJointTrajectoryExecutor executor(nullptr, nullptr, nullptr);
  auto goal = MakeSingleJointGoal("joint_1");
  IpFollowJointTrajectoryExecutor::State actual;
  actual.position = 0.25;
  actual.velocity = -0.1;

  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;
  EXPECT_TRUE(executor.hasActiveGoal());
}

TEST(IpFollowJointTrajectoryExecutor, StepProducesProgressTowardsGoal) {
  IpFollowJointTrajectoryExecutor::Config config;
  config.max_velocity = 2.0;
  config.max_acceleration = 4.0;
  config.max_jerk = 20.0;
  config.goal_tolerance = 1e-4;
  IpFollowJointTrajectoryExecutor executor(nullptr, nullptr, nullptr, config);

  auto goal = MakeSingleJointGoal("joint_1");
  IpFollowJointTrajectoryExecutor::State actual;
  actual.position = 0.0;

  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;

  IpFollowJointTrajectoryExecutor::State cmd;
  const auto status = executor.step(actual, &cmd, &error);
  EXPECT_EQ(status, IpFollowJointTrajectoryExecutor::StepStatus::kWorking);
  EXPECT_GT(cmd.position, 0.0);
  EXPECT_LT(cmd.position, 1.0);
}

TEST(IpFollowJointTrajectoryExecutor, ConsecutiveStepsAdvanceBeyondFirstSample) {
  IpFollowJointTrajectoryExecutor::Config config;
  config.max_velocity = 2.0;
  config.max_acceleration = 4.0;
  config.max_jerk = 20.0;
  config.goal_tolerance = 1e-4;
  IpFollowJointTrajectoryExecutor executor(nullptr, nullptr, nullptr, config);

  auto goal = MakeSingleJointGoal("joint_1");
  IpFollowJointTrajectoryExecutor::State actual;
  actual.position = 0.0;

  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;

  IpFollowJointTrajectoryExecutor::State first_cmd;
  ASSERT_EQ(executor.step(actual, &first_cmd, &error),
            IpFollowJointTrajectoryExecutor::StepStatus::kWorking);

  IpFollowJointTrajectoryExecutor::State second_cmd;
  ASSERT_EQ(executor.step(actual, &second_cmd, &error),
            IpFollowJointTrajectoryExecutor::StepStatus::kWorking);

  EXPECT_GT(second_cmd.position, first_cmd.position);
}

TEST(IpFollowJointTrajectoryExecutor, CancelClearsActiveGoal) {
  IpFollowJointTrajectoryExecutor executor(nullptr, nullptr, nullptr);
  auto goal = MakeSingleJointGoal("joint_1");
  IpFollowJointTrajectoryExecutor::State actual;
  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;
  ASSERT_TRUE(executor.hasActiveGoal());

  executor.cancelGoal();
  EXPECT_FALSE(executor.hasActiveGoal());
}

}  // namespace
}  // namespace canopen_hw
