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

#include "cmpf_trajectory_planner/plugins/dummy_trajectory_planner.hpp"

namespace cmpf
{
namespace trajectory_planner
{
DummyTrajectoryPlanner::DummyTrajectoryPlanner()
{
}

DummyTrajectoryPlanner::~DummyTrajectoryPlanner()
{
}

void DummyTrajectoryPlanner::initialize(const std::string& name, ros::NodeHandle nh, tf2_ros::Buffer* tf)
{
  std::cout << "Dummy Trajectory Planner initialize." << std::endl;
}

bool DummyTrajectoryPlanner::setRoute(const cmpf_msgs::Route& route)
{
}

bool DummyTrajectoryPlanner::computeTrajectory(cmpf_msgs::Trajectory& trajectory)
{
}

}  // namespace trajectory_planner
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
// register this controller as a BaseController plugin
PLUGINLIB_EXPORT_CLASS(cmpf::trajectory_planner::DummyTrajectoryPlanner, cmpf::cmpf_core::BaseTrajectoryPlanner)