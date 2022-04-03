/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak, Stephanie Eng */

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace trajectory_processing
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_trajectory_processing.ruckig_traj_smoothing");
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 1000;        // rad/s^3
constexpr double MAX_DURATION_EXTENSION_FACTOR = 10.0;
constexpr double DURATION_EXTENSION_FRACTION = 1.1;
}  // namespace

bool RuckigSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                     const double max_velocity_scaling_factor,
                                     const double max_acceleration_scaling_factor)
{
  moveit::core::JointModelGroup const* const group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    RCLCPP_WARN(LOGGER,
                "Trajectory does not have enough points to smooth with Ruckig. Returning an unmodified trajectory.");
    return true;
  }

  // Cache the trajectory in case we need to reset it
  robot_trajectory::RobotTrajectory original_trajectory =
      robot_trajectory::RobotTrajectory(trajectory, true /* deep copy */);

  const size_t num_dof = group->getVariableCount();

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  // Instantiate the smoother
  double timestep = trajectory.getAverageSegmentDuration();
  std::unique_ptr<ruckig::Ruckig<ruckig::DynamicDOFs>> ruckig_ptr;
  ruckig_ptr = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(num_dof, timestep);
  ruckig::InputParameter<ruckig::DynamicDOFs> ruckig_input{ num_dof };
  ruckig::OutputParameter<ruckig::DynamicDOFs> ruckig_output{ num_dof };

  // Initialize the smoother
  initializeRuckigState(ruckig_input, ruckig_output, *trajectory.getFirstWayPointPtr(), group);

  // Kinematic limits (vel/accel/jerk)
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  for (size_t i = 0; i < num_dof; ++i)
  {
    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars.at(i));

    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * bounds.max_velocity_;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(LOGGER,
                              "Joint velocity limits are not defined. Using the default "
                                  << DEFAULT_MAX_VELOCITY
                                  << " rad/s. You can define velocity limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_velocity.at(i) = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * bounds.max_acceleration_;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(LOGGER,
                              "Joint acceleration limits are not defined. Using the default "
                                  << DEFAULT_MAX_ACCELERATION
                                  << " rad/s^2. You can define acceleration limits in the URDF or joint_limits.yaml.");
      ruckig_input.max_acceleration.at(i) = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }
    ruckig_input.max_jerk.at(i) = bounds.jerk_bounded_ ? bounds.max_jerk_ : DEFAULT_MAX_JERK;
    if (bounds.jerk_bounded_)
    {
      ruckig_input.max_jerk.at(i) = bounds.max_jerk_;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(LOGGER, "Joint jerk limits are not defined. Using the default "
                                          << DEFAULT_MAX_JERK
                                          << " rad/s^3. You can define jerk limits in joint_limits.yaml.");
      ruckig_input.max_jerk.at(i) = DEFAULT_MAX_JERK;
    }
  }

  ruckig::Result ruckig_result;
  double duration_extension_factor = 1;
  bool smoothing_complete = false;
  while ((duration_extension_factor < MAX_DURATION_EXTENSION_FACTOR) && !smoothing_complete)
  {
    for (size_t waypoint_idx = 0; waypoint_idx < num_waypoints - 1; ++waypoint_idx)
    {
      moveit::core::RobotStatePtr next_waypoint = trajectory.getWayPointPtr(waypoint_idx + 1);

      getNextRuckigInput(ruckig_output, next_waypoint, group, ruckig_input);

      // Run Ruckig
      ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);

      if ((waypoint_idx == num_waypoints - 2) && ruckig_result == ruckig::Result::Finished)
      {
        smoothing_complete = true;
        break;
      }

      // Extend the trajectory duration if Ruckig could not reach the waypoint successfully
      if (ruckig_result != ruckig::Result::Finished)
      {
        duration_extension_factor *= DURATION_EXTENSION_FRACTION;
        // Reset the trajectory
        trajectory = robot_trajectory::RobotTrajectory(original_trajectory, true /* deep copy */);
        for (size_t time_stretch_idx = 1; time_stretch_idx < num_waypoints; ++time_stretch_idx)
        {
          trajectory.setWayPointDurationFromPrevious(
              time_stretch_idx,
              duration_extension_factor * original_trajectory.getWayPointDurationFromPrevious(time_stretch_idx));
          // re-calculate waypoint velocity and acceleration
          auto target_state = trajectory.getWayPointPtr(time_stretch_idx);
          const auto prev_state = trajectory.getWayPointPtr(time_stretch_idx - 1);
          timestep = trajectory.getAverageSegmentDuration();
          for (size_t joint = 0; joint < num_dof; ++joint)
          {
            target_state->setVariableVelocity(move_group_idx.at(joint),
                                              (1 / duration_extension_factor) *
                                                  target_state->getVariableVelocity(move_group_idx.at(joint)));

            double prev_velocity = prev_state->getVariableVelocity(move_group_idx.at(joint));
            double curr_velocity = target_state->getVariableVelocity(move_group_idx.at(joint));
            target_state->setVariableAcceleration(move_group_idx.at(joint), (curr_velocity - prev_velocity) / timestep);
          }
          target_state->update();
        }
        ruckig_ptr = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(num_dof, timestep);
        initializeRuckigState(ruckig_input, ruckig_output, *trajectory.getFirstWayPointPtr(), num_dof, move_group_idx);
        // Begin the while() loop again
        break;
      }
    }
  }

  if (ruckig_result != ruckig::Result::Finished)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Ruckig trajectory smoothing failed. Ruckig error: " << ruckig_result);
    return false;
  }

  return true;
}

void RuckigSmoothing::initializeRuckigState(ruckig::InputParameter<0>& ruckig_input,
                                            ruckig::OutputParameter<0>& ruckig_output,
                                            const moveit::core::RobotState& first_waypoint,
                                            const moveit::core::JointModelGroup* joint_group)
{
  const size_t num_dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  std::vector<double> current_positions_vector(num_dof);
  std::vector<double> current_velocities_vector(num_dof);
  std::vector<double> current_accelerations_vector(num_dof);

  for (size_t i = 0; i < num_dof; ++i)
  {
    current_positions_vector.at(i) = first_waypoint.getVariablePosition(idx.at(i));
    current_velocities_vector.at(i) = first_waypoint.getVariableVelocity(idx.at(i));
    current_accelerations_vector.at(i) = first_waypoint.getVariableAcceleration(idx.at(i));
  }
  std::copy_n(current_positions_vector.begin(), num_dof, ruckig_input.current_position.begin());
  std::copy_n(current_velocities_vector.begin(), num_dof, ruckig_input.current_velocity.begin());
  std::copy_n(current_accelerations_vector.begin(), num_dof, ruckig_input.current_acceleration.begin());
  // Initialize output data struct
  ruckig_output.new_position = ruckig_input.current_position;
  ruckig_output.new_velocity = ruckig_input.current_velocity;
  ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

void RuckigSmoothing::getNextRuckigInput(const ruckig::OutputParameter<0>& ruckig_output,
                                         const moveit::core::RobotStatePtr& next_waypoint,
                                         const moveit::core::JointModelGroup* joint_group,
                                         ruckig::InputParameter<0>& ruckig_input)
{
  // TODO(andyz): https://github.com/ros-planning/moveit2/issues/766
  // ruckig_output.pass_to_input(ruckig_input);

  const size_t num_dof = joint_group->getVariableCount();
  const std::vector<int>& idx = joint_group->getVariableIndexList();

  for (size_t joint = 0; joint < num_dof; ++joint)
  {
    // Feed output from the previous timestep back as input
    ruckig_input.current_position.at(joint) = ruckig_output.new_position.at(joint);
    ruckig_input.current_velocity.at(joint) = ruckig_output.new_velocity.at(joint);
    ruckig_input.current_acceleration.at(joint) = ruckig_output.new_acceleration.at(joint);

    // Target state is the next waypoint
    ruckig_input.target_position.at(joint) = next_waypoint->getVariablePosition(idx.at(joint));
    ruckig_input.target_velocity.at(joint) = next_waypoint->getVariableVelocity(idx.at(joint));
    ruckig_input.target_acceleration.at(joint) = next_waypoint->getVariableAcceleration(idx.at(joint));
  }
}
}  // namespace trajectory_processing
