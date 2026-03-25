#include "canopen_hw/ip_follow_joint_trajectory_executor.hpp"

#include <cmath>
#include <chrono>
#include <thread>
#include <utility>

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, CanopenRobotHwRos* hw, std::mutex* loop_mtx)
    : IpFollowJointTrajectoryExecutor(pnh, hw, loop_mtx, Config{}) {}

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, CanopenRobotHwRos* hw, std::mutex* loop_mtx,
    Config config)
    : hw_(hw),
      loop_mtx_(loop_mtx),
      config_(std::move(config)),
      otg_(1.0 / std::max(1.0, config_.command_rate_hz)) {
  if (pnh == nullptr) {
    return;
  }

  server_ = std::make_unique<Server>(
      *pnh, config_.action_ns,
      [this](const GoalConstPtr& goal) { ExecuteGoal(goal); }, false);
  server_->start();
}

void IpFollowJointTrajectoryExecutor::update(const ros::Time& /*now*/,
                                             const ros::Duration& period) {
  if (hw_ == nullptr) {
    return;
  }

  cycle_remainder_sec_ += period.toSec();
  const double cycle = 1.0 / std::max(1.0, config_.command_rate_hz);
  if (cycle_remainder_sec_ + 1e-9 < cycle) {
    return;
  }
  cycle_remainder_sec_ = std::fmod(cycle_remainder_sec_, cycle);

  State actual;
  actual.position = hw_->joint_position(config_.joint_index);
  actual.velocity = hw_->joint_velocity(config_.joint_index);

  State command;
  std::string error;
  const StepStatus status = step(actual, &command, &error);

  if (status == StepStatus::kWorking || status == StepStatus::kFinished) {
    hw_->SetExternalPositionCommand(config_.joint_index, command.position);
  }

  if (status == StepStatus::kFinished || status == StepStatus::kError) {
    std::lock_guard<std::mutex> lk(exec_mtx_);
    last_terminal_status_ = status;
    last_terminal_error_ = error;
    exec_cv_.notify_all();
  }
}

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

bool IpFollowJointTrajectoryExecutor::startGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal, const State& actual,
    std::string* error) {
  if (error) {
    error->clear();
  }

  if (!ValidateGoal(goal, config_.joint_name, error)) {
    return false;
  }

  const auto& target_point = goal.trajectory.points.back();
  if (target_point.positions.size() != 1u) {
    if (error) {
      *error = "last point must provide exactly one position";
    }
    return false;
  }

  std::lock_guard<std::mutex> lk(exec_mtx_);
  active_goal_ = goal;
  last_terminal_status_.reset();
  last_terminal_error_.clear();
  otg_.reset();

  input_.current_position[0] = actual.position;
  input_.current_velocity[0] = actual.velocity;
  input_.current_acceleration[0] = actual.acceleration;
  input_.target_position[0] = target_point.positions[0];
  input_.target_velocity[0] =
      target_point.velocities.empty() ? 0.0 : target_point.velocities[0];
  input_.target_acceleration[0] =
      target_point.accelerations.empty() ? 0.0 : target_point.accelerations[0];
  input_.max_velocity[0] = config_.max_velocity;
  input_.max_acceleration[0] = config_.max_acceleration;
  input_.max_jerk[0] = config_.max_jerk;

  if (!otg_.validate_input(input_)) {
    active_goal_.reset();
    if (error) {
      *error = "ruckig rejected goal input";
    }
    return false;
  }

  return true;
}

void IpFollowJointTrajectoryExecutor::cancelGoal() {
  std::lock_guard<std::mutex> lk(exec_mtx_);
  active_goal_.reset();
  last_terminal_status_ = StepStatus::kIdle;
  last_terminal_error_.clear();
  otg_.reset();
  exec_cv_.notify_all();
}

IpFollowJointTrajectoryExecutor::StepStatus IpFollowJointTrajectoryExecutor::step(
    const State& actual, State* command, std::string* error) {
  if (error) {
    error->clear();
  }
  if (command == nullptr) {
    if (error) {
      *error = "command output is null";
    }
    return StepStatus::kError;
  }

  std::lock_guard<std::mutex> lk(exec_mtx_);
  if (!active_goal_) {
    *command = actual;
    return StepStatus::kIdle;
  }

  input_.current_position[0] = actual.position;
  input_.current_velocity[0] = actual.velocity;
  input_.current_acceleration[0] = actual.acceleration;

  const auto result = otg_.update(input_, output_);
  if (result == ruckig::Result::ErrorInvalidInput ||
      result == ruckig::Result::Error ||
      result == ruckig::Result::ErrorTrajectoryDuration ||
      result == ruckig::Result::ErrorSynchronizationCalculation ||
      result == ruckig::Result::ErrorExecutionTimeCalculation) {
    active_goal_.reset();
    if (error) {
      *error = "ruckig step failed";
    }
    last_terminal_status_ = StepStatus::kError;
    last_terminal_error_ = error ? *error : std::string();
    exec_cv_.notify_all();
    *command = actual;
    return StepStatus::kError;
  }

  command->position = output_.new_position[0];
  command->velocity = output_.new_velocity[0];
  command->acceleration = output_.new_acceleration[0];
  last_trajectory_time_ = output_.time;

  output_.pass_to_input(input_);

  const auto& target = active_goal_->trajectory.points.back().positions.front();
  const bool reached =
      std::abs(command->position - target) <= config_.goal_tolerance;
  if (result == ruckig::Result::Finished || reached) {
    active_goal_.reset();
    last_terminal_status_ = StepStatus::kFinished;
    last_terminal_error_.clear();
    exec_cv_.notify_all();
    return StepStatus::kFinished;
  }

  return StepStatus::kWorking;
}

bool IpFollowJointTrajectoryExecutor::hasActiveGoal() const {
  std::lock_guard<std::mutex> lk(exec_mtx_);
  return active_goal_.has_value();
}

void IpFollowJointTrajectoryExecutor::publishFeedback(
    const State& actual, const State& command) const {
  if (!server_ || !server_->isActive()) {
    return;
  }

  control_msgs::FollowJointTrajectoryFeedback feedback;
  feedback.joint_names = {config_.joint_name};
  feedback.desired.positions = {command.position};
  feedback.desired.velocities = {command.velocity};
  feedback.desired.accelerations = {command.acceleration};
  feedback.desired.time_from_start = ros::Duration(last_trajectory_time_);

  feedback.actual.positions = {actual.position};
  feedback.actual.velocities = {actual.velocity};
  feedback.actual.accelerations = {actual.acceleration};
  feedback.actual.time_from_start = ros::Duration(last_trajectory_time_);

  feedback.error.positions = {command.position - actual.position};
  feedback.error.velocities = {command.velocity - actual.velocity};
  feedback.error.accelerations = {command.acceleration - actual.acceleration};
  feedback.error.time_from_start = ros::Duration(0.0);
  server_->publishFeedback(feedback);
}

void IpFollowJointTrajectoryExecutor::holdCurrentPosition() {
  if (hw_ == nullptr || loop_mtx_ == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> loop_lock(*loop_mtx_);
  hw_->SetExternalPositionCommand(config_.joint_index,
                                  hw_->joint_position(config_.joint_index));
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

  if (hw_ == nullptr || loop_mtx_ == nullptr) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code =
        control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "executor runtime not initialized";
    server_->setAborted(result, result.error_string);
    return;
  }

  State actual;
  {
    std::lock_guard<std::mutex> loop_lock(*loop_mtx_);
    actual.position = hw_->joint_position(config_.joint_index);
    actual.velocity = hw_->joint_velocity(config_.joint_index);
  }
  std::string start_error;
  if (!startGoal(*goal, actual, &start_error)) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code =
        control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = start_error.empty() ? "failed to start goal"
                                              : start_error;
    server_->setAborted(result, result.error_string);
    return;
  }

  CANOPEN_LOG_INFO("IpFollowJointTrajectoryExecutor: accepted goal for {}",
                   config_.joint_name);

  std::unique_lock<std::mutex> lk(exec_mtx_);
  while (!last_terminal_status_.has_value()) {
    lk.unlock();
    if (server_->isPreemptRequested()) {
      cancelGoal();
      holdCurrentPosition();
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      result.error_string = "goal preempted";
      server_->setPreempted(result, result.error_string);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    lk.lock();
  }

  control_msgs::FollowJointTrajectoryResult result;
  if (*last_terminal_status_ == StepStatus::kFinished) {
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    result.error_string = "goal completed";
    server_->setSucceeded(result, result.error_string);
  } else {
    holdCurrentPosition();
    result.error_code =
        control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    result.error_string =
        last_terminal_error_.empty() ? "goal execution failed"
                                     : last_terminal_error_;
    server_->setAborted(result, result.error_string);
  }
}

}  // namespace canopen_hw
