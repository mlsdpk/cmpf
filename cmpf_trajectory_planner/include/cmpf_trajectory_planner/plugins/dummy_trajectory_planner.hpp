/************************************************************************
  Copyright 2021 Phone Thiha Kyaw

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
************************************************************************/

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

#include <pluginlib/class_loader.hpp>

#include <string>

#include "cmpf_core/base_trajectory_planner.hpp"
#include "cmpf_msgs/Route.h"
#include "cmpf_msgs/Trajectory.h"

namespace cmpf
{
namespace trajectory_planner
{
class DummyTrajectoryPlanner : public cmpf_core::BaseTrajectoryPlanner
{
public:
  /**
   * @brief Constructor
   */
  DummyTrajectoryPlanner();

  /**
   * @brief Destructor
   */
  virtual ~DummyTrajectoryPlanner();

  /**
   * @brief Method to initialize the trajectory planner
   * @param name trajectory planner plugin name
   * @param nh private nodehandler
   * @param tf tf transform
   */
  virtual void initialize(const std::string& name, ros::NodeHandle nh, tf2_ros::Buffer* tf) override;

  /**
   * @brief Set the route to follow
   * @param route The route given by the global route planner
   */
  virtual void setRoute(const cmpf_msgs::Route& route) override;

  /**
   * @brief Method to compute trajectory (assuming that current route is already given)
   * @param trajectory The computed trajectory
   * @param pose Current pose of the vehicle in global frame
   */
  virtual void computeTrajectory(cmpf_msgs::Trajectory& trajectory, const geometry_msgs::PoseStamped& pose) override;

private:
  bool transformGlobalRoute(cmpf_msgs::Trajectory& trajectory, const geometry_msgs::PoseStamped& pose);
  bool transformPose(const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose,
                     const std::string& frame);

  cmpf_msgs::Route route_;
  tf2_ros::Buffer* tf_;
  double max_distance_{ 100.0 };
};

}  // namespace trajectory_planner
}  // namespace cmpf