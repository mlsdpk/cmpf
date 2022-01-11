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

#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <string>

#include "cmpf_msgs/Route.h"
#include "cmpf_msgs/Trajectory.h"

namespace cmpf
{
namespace cmpf_core
{
/**
 * @class BaseTrajectoryPlanner
 * @brief Abstract planner interface that acts as a virtual base class for
 * all trajectory planning plugins
 */
class BaseTrajectoryPlanner
{
public:
  using Ptr = std::shared_ptr<cmpf::cmpf_core::BaseTrajectoryPlanner>;

  /**
   * @brief Virtual destructor
   */
  virtual ~BaseTrajectoryPlanner()
  {
  }

  /**
   * @brief Method to initialize the trajectory planner
   * @param name trajectory planner plugin name
   * @param nh private nodehandler
   * @param tf tf transform
   */
  virtual void initialize(const std::string& name, ros::NodeHandle nh, tf2_ros::Buffer* tf) = 0;

  /**
   * @brief Set the route to follow
   * @param route The route given by the global route planner
   */
  virtual bool setRoute(const cmpf_msgs::Route& route) = 0;

  /**
   * @brief Method to compute trajectory (assuming that current route is already given)
   * @param trajectory The computed trajectory
   */
  virtual bool computeTrajectory(cmpf_msgs::Trajectory& trajectory) = 0;
};

}  // namespace cmpf_core
}  // namespace cmpf