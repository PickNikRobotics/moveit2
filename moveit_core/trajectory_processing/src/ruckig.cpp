/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <limits>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <moveit/trajectory_processing/ruckig.h>
#include <ruckig/ruckig.hpp>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <ruckig/ruckig.hpp>

namespace trajectory_processing
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_trajectory_processing.time_optimal_trajectory_generation");
constexpr double DEFAULT_TIMESTEP = 0.001;
constexpr double DEFAULT_MAX_VELOCITY = 10;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 100;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 1e6;          // rad/s^3

bool RuckigSmoothing::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                        const double max_velocity_scaling_factor,
                                        const double max_acceleration_scaling_factor) const
{
  if (trajectory.getWayPointCount() < 2)
  {
    RCLCPP_WARN_STREAM(LOGGER, "Trajectory does not have enough points to smooth with Ruckig");
    return true;
  }

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  const size_t num_dof = group->getVariableCount();

  // Instantiate the smoother
  ruckig::Ruckig<0> ruckig{ num_dof, DEFAULT_TIMESTEP };
  ruckig::InputParameter<0> ruckig_input{ num_dof };
  ruckig::OutputParameter<0> ruckig_output{ num_dof };

  // Initialize the smoother
  std::vector<double> current_positions_vector;
  current_positions_vector.assign(trajectory.getFirstWayPoint().getVariablePositions(),
                                  trajectory.getFirstWayPoint().getVariablePositions() + num_dof);
  std::copy_n(current_positions_vector.begin(), num_dof, ruckig_input.current_position.begin());
  // Assume starting from rest
  std::vector<double> zero_vector(num_dof, 0.0);
  std::copy_n(zero_vector.begin(), num_dof, ruckig_input.current_velocity.begin());
  std::copy_n(zero_vector.begin(), num_dof, ruckig_input.current_acceleration.begin());
  // Initialize output data struct
  ruckig_output.new_velocity = ruckig_input.current_velocity;
  ruckig_output.new_acceleration = ruckig_input.current_acceleration;
  // Kinematic limits (vel/accel/jerk)
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  for (size_t i = 0; i < num_dof; ++i)
  {
    // TODO(andyz): read this from the joint group if/when jerk becomes supported
    ruckig_input.max_jerk[i] = DEFAULT_MAX_JERK;

    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars[i]);
    //    ruckig_input.max_acceleration[i] = max_acceleration_scaling_factor * MAX_ACCEL;

    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input.max_velocity[i] = max_velocity_scaling_factor * bounds.max_velocity_;
    }
    else
    {
      ruckig_input.max_velocity[i] = max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input.max_acceleration[i] = max_acceleration_scaling_factor * bounds.max_acceleration_;
    }
    else
    {
      ruckig_input.max_acceleration[i] = max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION;
    }
  }




  moveit_msgs::msg::RobotTrajectory traj_msg;
  trajectory.getRobotTrajectoryMsg(traj_msg);
  for (size_t waypoint = 0; waypoint < NUM_WAYPOINTS - 1; ++waypoint){
    // TODO: NUM_WAYPOINTS???


    for (size_t joint = 0; joint < NUM_DOF; ++joint){
      // TODO: NUM_DOF???


      // Feed output from the previous timestep back as input
      ruckig_input.current_position[joint] = ruckig_output.new_position[joint];
      ruckig_input.current_velocity[joint] = ruckig_output.new_velocity[joint];
      ruckig_input.current_acceleration[joint] = ruckig_output.new_acceleration[joint];

      // Target state is the next waypoint
      ruckig_input.target_position[joint] = traj_msg.points[waypoint + 1].positions[joint];
      ruckig_input.target_velocity[joint] = traj_msg.points[waypoint + 1].velocities[joint];
      ruckig_input.target_acceleration[joint] = traj_msg.points[waypoint + 1].accelerations[joint];
    }
    auto ruckig_result = ruckig.update(ruckig_input, ruckig_output);

    if (ruckig_result != ruckig::Result::Working) {

      RCLCPP_INFO(get_logger(), "Ruckig failed at waypoint " << waypoint);
      continue;   // skip to next waypoint

    }

    // Store result back in the trajectory data structure
    for (size_t joint = 0; jount < NUM_DOF; ++joint) {
      // TODO: NUM_DOF ???
      traj_msg.points[waypoint + 1].velocities[joint] = ruckig_output.new_velocity[joint];
      traj_msg.ppints[waypoint + 1].positions[joint] = ruckig_output.new_position[joint];
    }
  }

  RCLCPP_INFO(get_logger(), "Done smoothing");

  return true;
}
}  // namespace trajectory_processing
