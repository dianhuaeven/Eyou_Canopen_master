#include "canopen_hw/ip_follow_joint_trajectory_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>
#include <vector>

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

namespace {

void SetError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

bool HasDuplicateNames(const std::vector<std::string>& names,
                       std::string* duplicate_name) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    for (std::size_t j = i + 1; j < names.size(); ++j) {
      if (names[i] == names[j]) {
        if (duplicate_name != nullptr) {
          *duplicate_name = names[i];
        }
        return true;
      }
    }
  }
  return false;
}

bool HasDuplicateIndices(const std::vector<std::size_t>& indices,
                         std::size_t* duplicate_index) {
  for (std::size_t i = 0; i < indices.size(); ++i) {
    for (std::size_t j = i + 1; j < indices.size(); ++j) {
      if (indices[i] == indices[j]) {
        if (duplicate_index != nullptr) {
          *duplicate_index = indices[i];
        }
        return true;
      }
    }
  }
  return false;
}

bool ValidateVectorSize(const std::vector<double>& values,
                        std::size_t expected_size,
                        const std::string& field_name,
                        std::string* error) {
  if (values.size() != expected_size) {
    SetError(error, field_name + " size mismatch: expected " +
                        std::to_string(expected_size) + ", got " +
                        std::to_string(values.size()));
    return false;
  }
  return true;
}

bool ValidateNonNegativeVector(const std::vector<double>& values,
                               const std::string& field_name,
                               bool allow_zero,
                               std::string* error) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    const bool ok = allow_zero ? (values[i] >= 0.0) : (values[i] > 0.0);
    if (!ok) {
      SetError(error, field_name + "[" + std::to_string(i) +
                          "] must be " + (allow_zero ? ">= 0" : "> 0"));
      return false;
    }
  }
  return true;
}

bool ValidateConfig(const IpFollowJointTrajectoryExecutor::Config& config,
                    std::string* error) {
  if (config.action_ns.empty()) {
    SetError(error, "action_ns must not be empty");
    return false;
  }
  if (config.command_rate_hz <= 0.0) {
    SetError(error, "command_rate_hz must be > 0");
    return false;
  }

  const std::size_t dofs = config.joint_names.size();
  if (dofs == 0) {
    SetError(error, "joint_names must not be empty");
    return false;
  }
  if (config.joint_indices.size() != dofs) {
    SetError(error, "joint_indices size mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(config.joint_indices.size()));
    return false;
  }
  if (!ValidateVectorSize(config.max_velocities, dofs, "max_velocities",
                          error) ||
      !ValidateVectorSize(config.max_accelerations, dofs,
                          "max_accelerations", error) ||
      !ValidateVectorSize(config.max_jerks, dofs, "max_jerks", error) ||
      !ValidateVectorSize(config.goal_tolerances, dofs, "goal_tolerances",
                          error)) {
    return false;
  }

  std::string duplicate_name;
  if (HasDuplicateNames(config.joint_names, &duplicate_name)) {
    SetError(error, "joint_names contains duplicate joint: " + duplicate_name);
    return false;
  }

  std::size_t duplicate_index = 0;
  if (HasDuplicateIndices(config.joint_indices, &duplicate_index)) {
    SetError(error, "joint_indices contains duplicate axis index: " +
                        std::to_string(duplicate_index));
    return false;
  }

  if (!ValidateNonNegativeVector(config.max_velocities, "max_velocities", false,
                                 error) ||
      !ValidateNonNegativeVector(config.max_accelerations,
                                 "max_accelerations", false, error) ||
      !ValidateNonNegativeVector(config.max_jerks, "max_jerks", false, error) ||
      !ValidateNonNegativeVector(config.goal_tolerances, "goal_tolerances",
                                 true, error)) {
    return false;
  }

  return true;
}

bool ValidateState(const IpFollowJointTrajectoryExecutor::State& state,
                   std::size_t dofs, std::string* error) {
  if (state.positions.size() != dofs) {
    SetError(error, "state.positions size mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(state.positions.size()));
    return false;
  }
  if (!state.velocities.empty() && state.velocities.size() != dofs) {
    SetError(error, "state.velocities size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(state.velocities.size()));
    return false;
  }
  if (!state.accelerations.empty() && state.accelerations.size() != dofs) {
    SetError(error, "state.accelerations size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(state.accelerations.size()));
    return false;
  }
  return true;
}

double StateValueOrZero(const std::vector<double>& values, std::size_t index) {
  return index < values.size() ? values[index] : 0.0;
}

void EnsureStateArrays(IpFollowJointTrajectoryExecutor::State* state,
                       std::size_t dofs) {
  if (state == nullptr) {
    return;
  }

  state->positions.resize(dofs, 0.0);
  state->velocities.resize(dofs, 0.0);
  state->accelerations.resize(dofs, 0.0);
}

bool BuildGoalToConfigIndices(const std::vector<std::string>& goal_joint_names,
                              const std::vector<std::string>& config_joint_names,
                              std::vector<std::size_t>* goal_to_config_indices,
                              std::string* error) {
  if (goal_to_config_indices == nullptr) {
    SetError(error, "goal_to_config_indices is null");
    return false;
  }

  goal_to_config_indices->assign(goal_joint_names.size(), 0);
  for (std::size_t goal_index = 0; goal_index < goal_joint_names.size();
       ++goal_index) {
    const auto it = std::find(config_joint_names.begin(), config_joint_names.end(),
                              goal_joint_names[goal_index]);
    if (it == config_joint_names.end()) {
      SetError(error, "goal contains unknown joint: " +
                          goal_joint_names[goal_index]);
      return false;
    }
    (*goal_to_config_indices)[goal_index] =
        static_cast<std::size_t>(std::distance(config_joint_names.begin(), it));
  }

  return true;
}

bool ValidateTrajectoryPoint(
    const trajectory_msgs::JointTrajectoryPoint& point,
    std::size_t dofs, std::size_t point_index, std::string* error) {
  if (point.positions.size() != dofs) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] positions size mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(point.positions.size()));
    return false;
  }
  if (!point.velocities.empty() && point.velocities.size() != dofs) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] velocities size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(point.velocities.size()));
    return false;
  }
  if (!point.accelerations.empty() && point.accelerations.size() != dofs) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] accelerations size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(point.accelerations.size()));
    return false;
  }
  if (point.time_from_start.toSec() < 0.0) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] time_from_start must be >= 0");
    return false;
  }

  return true;
}

bool IsTerminalRuckigError(const ruckig::Result result) {
  return result == ruckig::Result::ErrorInvalidInput ||
         result == ruckig::Result::Error ||
         result == ruckig::Result::ErrorTrajectoryDuration ||
         result == ruckig::Result::ErrorSynchronizationCalculation ||
         result == ruckig::Result::ErrorExecutionTimeCalculation;
}

}  // namespace

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, CanopenRobotHwRos* hw, std::mutex* loop_mtx)
    : IpFollowJointTrajectoryExecutor(pnh, hw, loop_mtx, Config{}) {}

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, CanopenRobotHwRos* hw, std::mutex* loop_mtx,
    Config config)
    : hw_(hw),
      loop_mtx_(loop_mtx),
      config_(std::move(config)),
      otg_(config_.joint_names.size(),
           1.0 / std::max(1.0, config_.command_rate_hz)),
      input_(config_.joint_names.size()),
      output_(config_.joint_names.size()) {
  config_valid_ = ValidateConfig(config_, &config_error_);
  if (!config_valid_) {
    CANOPEN_LOG_ERROR("IpFollowJointTrajectoryExecutor: invalid config: {}",
                      config_error_);
    return;
  }

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
  if (!config_valid_ || hw_ == nullptr) {
    return;
  }

  cycle_remainder_sec_ += period.toSec();
  const double cycle = 1.0 / std::max(1.0, config_.command_rate_hz);
  if (cycle_remainder_sec_ + 1e-9 < cycle) {
    return;
  }
  cycle_remainder_sec_ = std::fmod(cycle_remainder_sec_, cycle);

  const std::size_t dofs = config_.joint_indices.size();
  State actual;
  EnsureStateArrays(&actual, dofs);
  for (std::size_t i = 0; i < dofs; ++i) {
    actual.positions[i] = hw_->joint_position(config_.joint_indices[i]);
    actual.velocities[i] = hw_->joint_velocity(config_.joint_indices[i]);
  }

  State command;
  std::string error;
  const StepStatus status = step(actual, &command, &error);

  if (status == StepStatus::kWorking || status == StepStatus::kFinished) {
    for (std::size_t i = 0; i < dofs; ++i) {
      hw_->SetExternalPositionCommand(config_.joint_indices[i],
                                      command.positions[i]);
    }
    publishFeedback(actual, command);
  }
}

bool IpFollowJointTrajectoryExecutor::ValidateGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::vector<std::string>& joint_names, std::string* error) {
  if (error != nullptr) {
    error->clear();
  }

  const std::size_t dofs = joint_names.size();
  if (dofs == 0) {
    SetError(error, "executor has no configured joints");
    return false;
  }

  const auto& goal_joint_names = goal.trajectory.joint_names;
  if (goal_joint_names.size() != dofs) {
    SetError(error, "goal joint count mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(goal_joint_names.size()));
    return false;
  }

  std::string duplicate_name;
  if (HasDuplicateNames(goal_joint_names, &duplicate_name)) {
    SetError(error, "goal contains duplicate joint: " + duplicate_name);
    return false;
  }

  std::vector<std::size_t> goal_to_config_indices;
  if (!BuildGoalToConfigIndices(goal_joint_names, joint_names,
                                &goal_to_config_indices, error)) {
    return false;
  }

  const auto& points = goal.trajectory.points;
  if (points.empty()) {
    SetError(error, "goal contains no trajectory points");
    return false;
  }

  double previous_time = -1.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!ValidateTrajectoryPoint(points[i], dofs, i, error)) {
      return false;
    }

    const double current_time = points[i].time_from_start.toSec();
    if (previous_time > current_time) {
      SetError(error,
               "trajectory points must be time-ordered and nondecreasing");
      return false;
    }
    previous_time = current_time;
  }

  return true;
}

bool IpFollowJointTrajectoryExecutor::startGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal, const State& actual,
    std::string* error) {
  if (error != nullptr) {
    error->clear();
  }

  if (!config_valid_) {
    SetError(error, config_error_);
    return false;
  }

  const std::size_t dofs = config_.joint_names.size();
  if (!ValidateState(actual, dofs, error)) {
    return false;
  }
  if (!ValidateGoal(goal, config_.joint_names, error)) {
    return false;
  }

  std::vector<std::size_t> goal_to_config_indices;
  if (!BuildGoalToConfigIndices(goal.trajectory.joint_names, config_.joint_names,
                                &goal_to_config_indices, error)) {
    return false;
  }

  std::lock_guard<std::mutex> lk(exec_mtx_);
  active_goal_ = goal;
  goal_to_config_indices_ = std::move(goal_to_config_indices);
  waypoint_index_ = 0;
  last_terminal_status_.reset();
  last_terminal_error_.clear();
  last_trajectory_time_ = 0.0;
  otg_.reset();

  for (std::size_t axis_index = 0; axis_index < dofs; ++axis_index) {
    input_.current_position[axis_index] = actual.positions[axis_index];
    input_.current_velocity[axis_index] =
        StateValueOrZero(actual.velocities, axis_index);
    input_.current_acceleration[axis_index] =
        StateValueOrZero(actual.accelerations, axis_index);
    input_.max_velocity[axis_index] = config_.max_velocities[axis_index];
    input_.max_acceleration[axis_index] =
        config_.max_accelerations[axis_index];
    input_.max_jerk[axis_index] = config_.max_jerks[axis_index];
  }

  const auto& first_point = active_goal_->trajectory.points.front();
  for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
    const std::size_t axis_index = goal_to_config_indices_[goal_index];
    input_.target_position[axis_index] = first_point.positions[goal_index];
    input_.target_velocity[axis_index] =
        first_point.velocities.empty() ? 0.0 : first_point.velocities[goal_index];
    input_.target_acceleration[axis_index] = first_point.accelerations.empty()
                                                 ? 0.0
                                                 : first_point.accelerations[goal_index];
  }

  const double first_duration = first_point.time_from_start.toSec();
  if (first_duration > 0.0) {
    input_.minimum_duration = first_duration;
  } else {
    input_.minimum_duration.reset();
  }

  if (!otg_.validate_input(input_)) {
    active_goal_.reset();
    goal_to_config_indices_.clear();
    SetError(error, "ruckig rejected goal input");
    return false;
  }

  return true;
}

void IpFollowJointTrajectoryExecutor::cancelGoal() {
  std::lock_guard<std::mutex> lk(exec_mtx_);
  active_goal_.reset();
  goal_to_config_indices_.clear();
  waypoint_index_ = 0;
  last_trajectory_time_ = 0.0;
  last_terminal_status_ = StepStatus::kIdle;
  last_terminal_error_.clear();
  otg_.reset();
  exec_cv_.notify_all();
}

IpFollowJointTrajectoryExecutor::StepStatus
IpFollowJointTrajectoryExecutor::step(const State& actual, State* command,
                                      std::string* error) {
  if (error != nullptr) {
    error->clear();
  }
  if (command == nullptr) {
    SetError(error, "command output is null");
    return StepStatus::kError;
  }
  if (!config_valid_) {
    SetError(error, config_error_);
    return StepStatus::kError;
  }

  const std::size_t dofs = config_.joint_names.size();
  if (!ValidateState(actual, dofs, error)) {
    return StepStatus::kError;
  }
  EnsureStateArrays(command, dofs);

  std::lock_guard<std::mutex> lk(exec_mtx_);
  if (!active_goal_) {
    command->positions = actual.positions;
    command->velocities.assign(dofs, 0.0);
    command->accelerations.assign(dofs, 0.0);
    for (std::size_t i = 0; i < dofs; ++i) {
      command->velocities[i] = StateValueOrZero(actual.velocities, i);
      command->accelerations[i] = StateValueOrZero(actual.accelerations, i);
    }
    return StepStatus::kIdle;
  }

  const ruckig::Result result = otg_.update(input_, output_);
  if (IsTerminalRuckigError(result)) {
    active_goal_.reset();
    goal_to_config_indices_.clear();
    waypoint_index_ = 0;
    last_trajectory_time_ = 0.0;
    last_terminal_status_ = StepStatus::kError;
    last_terminal_error_ = "ruckig step failed";
    exec_cv_.notify_all();
    command->positions = actual.positions;
    command->velocities.assign(dofs, 0.0);
    command->accelerations.assign(dofs, 0.0);
    for (std::size_t i = 0; i < dofs; ++i) {
      command->velocities[i] = StateValueOrZero(actual.velocities, i);
      command->accelerations[i] = StateValueOrZero(actual.accelerations, i);
    }
    SetError(error, last_terminal_error_);
    return StepStatus::kError;
  }

  for (std::size_t axis_index = 0; axis_index < dofs; ++axis_index) {
    command->positions[axis_index] = output_.new_position[axis_index];
    command->velocities[axis_index] = output_.new_velocity[axis_index];
    command->accelerations[axis_index] = output_.new_acceleration[axis_index];
  }
  last_trajectory_time_ = output_.time;
  output_.pass_to_input(input_);

  if (result != ruckig::Result::Finished) {
    return StepStatus::kWorking;
  }

  const auto& points = active_goal_->trajectory.points;
  const bool is_last_segment = (waypoint_index_ + 1 >= points.size());
  if (!is_last_segment) {
    ++waypoint_index_;
    const auto& previous_point = points[waypoint_index_ - 1];
    const auto& next_point = points[waypoint_index_];

    for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
      const std::size_t axis_index = goal_to_config_indices_[goal_index];
      input_.target_position[axis_index] = next_point.positions[goal_index];
      input_.target_velocity[axis_index] =
          next_point.velocities.empty() ? 0.0 : next_point.velocities[goal_index];
      input_.target_acceleration[axis_index] =
          next_point.accelerations.empty()
              ? 0.0
              : next_point.accelerations[goal_index];
    }

    const double segment_duration =
        next_point.time_from_start.toSec() - previous_point.time_from_start.toSec();
    if (segment_duration > 0.0) {
      input_.minimum_duration = segment_duration;
    } else {
      input_.minimum_duration.reset();
    }
    otg_.reset();
    return StepStatus::kWorking;
  }

  const auto& last_point = points.back();
  bool all_axes_reached = true;
  for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
    const std::size_t axis_index = goal_to_config_indices_[goal_index];
    if (std::abs(actual.positions[axis_index] - last_point.positions[goal_index]) >
        config_.goal_tolerances[axis_index]) {
      all_axes_reached = false;
      break;
    }
  }

  if (all_axes_reached) {
    active_goal_.reset();
    goal_to_config_indices_.clear();
    waypoint_index_ = 0;
    last_terminal_status_ = StepStatus::kFinished;
    last_terminal_error_.clear();
    exec_cv_.notify_all();
    return StepStatus::kFinished;
  }

  for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
    const std::size_t axis_index = goal_to_config_indices_[goal_index];
    command->positions[axis_index] = last_point.positions[goal_index];
    command->velocities[axis_index] = 0.0;
    command->accelerations[axis_index] = 0.0;
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

  const std::size_t dofs = config_.joint_names.size();
  control_msgs::FollowJointTrajectoryFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.joint_names = config_.joint_names;

  feedback.desired.positions = command.positions;
  feedback.desired.velocities = command.velocities;
  feedback.desired.accelerations = command.accelerations;
  feedback.desired.time_from_start = ros::Duration(last_trajectory_time_);

  feedback.actual.positions = actual.positions;
  feedback.actual.velocities.assign(dofs, 0.0);
  feedback.actual.accelerations.assign(dofs, 0.0);
  for (std::size_t i = 0; i < dofs; ++i) {
    feedback.actual.velocities[i] = StateValueOrZero(actual.velocities, i);
    feedback.actual.accelerations[i] = StateValueOrZero(actual.accelerations, i);
  }

  feedback.error.positions.resize(dofs, 0.0);
  feedback.error.velocities.resize(dofs, 0.0);
  feedback.error.accelerations.resize(dofs, 0.0);
  for (std::size_t i = 0; i < dofs; ++i) {
    feedback.error.positions[i] = command.positions[i] - actual.positions[i];
    feedback.error.velocities[i] =
        command.velocities[i] - feedback.actual.velocities[i];
    feedback.error.accelerations[i] =
        command.accelerations[i] - feedback.actual.accelerations[i];
  }
  feedback.error.time_from_start = ros::Duration(0.0);

  server_->publishFeedback(feedback);
}

void IpFollowJointTrajectoryExecutor::holdCurrentPosition() {
  if (hw_ == nullptr) {
    return;
  }

  for (std::size_t i = 0; i < config_.joint_indices.size(); ++i) {
    const double pos = hw_->joint_position(config_.joint_indices[i]);
    hw_->SetExternalPositionCommand(config_.joint_indices[i], pos);
  }
}

void IpFollowJointTrajectoryExecutor::ExecuteGoal(
    const GoalConstPtr& goal_ptr) {
  if (goal_ptr == nullptr || server_ == nullptr) {
    return;
  }

  if (!config_valid_) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = config_error_;
    server_->setAborted(result, result.error_string);
    return;
  }

  if (hw_ == nullptr || loop_mtx_ == nullptr) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "executor runtime not initialized";
    server_->setAborted(result, result.error_string);
    return;
  }

  const std::size_t dofs = config_.joint_indices.size();
  State actual;
  EnsureStateArrays(&actual, dofs);
  {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    for (std::size_t i = 0; i < dofs; ++i) {
      actual.positions[i] = hw_->joint_position(config_.joint_indices[i]);
      actual.velocities[i] = hw_->joint_velocity(config_.joint_indices[i]);
    }
  }

  std::string start_error;
  if (!startGoal(*goal_ptr, actual, &start_error)) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string =
        start_error.empty() ? "failed to start goal" : start_error;
    server_->setAborted(result, result.error_string);
    return;
  }

  CANOPEN_LOG_INFO(
      "IpFollowJointTrajectoryExecutor: accepted goal for {} joints", dofs);

  std::unique_lock<std::mutex> lk(exec_mtx_);
  while (!last_terminal_status_.has_value()) {
    if (server_->isPreemptRequested()) {
      lk.unlock();
      cancelGoal();
      holdCurrentPosition();

      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      result.error_string = "goal preempted";
      server_->setPreempted(result, result.error_string);
      return;
    }

    exec_cv_.wait_for(lk, std::chrono::milliseconds(5));
  }

  control_msgs::FollowJointTrajectoryResult result;
  if (*last_terminal_status_ == StepStatus::kFinished) {
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    result.error_string = "goal completed";
    server_->setSucceeded(result, result.error_string);
    return;
  }

  holdCurrentPosition();
  result.error_code =
      control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
  result.error_string =
      last_terminal_error_.empty() ? "goal execution failed"
                                   : last_terminal_error_;
  server_->setAborted(result, result.error_string);
}

}  // namespace canopen_hw
