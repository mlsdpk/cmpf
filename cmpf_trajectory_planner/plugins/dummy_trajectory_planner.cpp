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
#include "cmpf_utils/transform_utils.hpp"
#include "cmpf_utils/distance_utils.hpp"

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
  tf_ = tf;

  // ros params
  // TODO: Implement better approach
  std::string max_distance_param_name = name + "/max_distance";
  nh.param<double>(max_distance_param_name, max_distance_, 30.0);
}

void DummyTrajectoryPlanner::setRoute(const cmpf_msgs::Route& route)
{
  if (route.waypoints.empty())
  {
    throw std::runtime_error("Route waypoints cannot be empty");
  }

  if (route.header.frame_id.empty())
  {
    throw std::runtime_error("Route message frame_id cannot be empty");
  }

  route_ = route;
}

void DummyTrajectoryPlanner::computeTrajectory(cmpf_msgs::Trajectory& trajectory,
                                               const geometry_msgs::PoseStamped& pose)
{
  // This dummy trajectory planner compute two things
  // 1. Only create trajectory for some user-defined horizon
  // 2. Add fine trajectory points to the result
  // Doesn't care about any collision or kinematic feasibility

  // make sure trajectory is empty
  trajectory.poses.clear();

  // transform the global route into local robot frame
  if (!transformGlobalRoute(trajectory, pose))
  {
    throw std::runtime_error("Could not transform the global route to the frame of the planner");
  }
}

bool DummyTrajectoryPlanner::transformGlobalRoute(cmpf_msgs::Trajectory& trajectory,
                                                  const geometry_msgs::PoseStamped& pose)
{
  // make sure the pose is in the same frame as the global route
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(pose, robot_pose, route_.header.frame_id))
  {
    ROS_ERROR("Unable to transform robot pose into global plan's frame");
    return false;
  }

  // First find the closest waypoint on the route to the robot
  auto transformation_begin = cmpf_utils::min_by(
      route_.waypoints.begin(), route_.waypoints.end(),
      [&robot_pose](const cmpf_msgs::Waypoint& wp) { return cmpf_utils::euclidean_distance(robot_pose, wp); });

  // Find waypoints definitely outside of the max distance so we won't transform them.
  auto transformation_end =
      std::find_if(transformation_begin, std::end(route_.waypoints), [&](const auto& global_route_wp) {
        return cmpf_utils::euclidean_distance(robot_pose, global_route_wp) > max_distance_;
      });

  // Lambda to transform a PoseStamped from global frame to local
  auto transform_global_pose_to_local = [&](const auto& global_waypoint) {
    geometry_msgs::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = route_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose.position.x = global_waypoint.x;
    stamped_pose.pose.position.y = global_waypoint.y;
    transformPose(stamped_pose, transformed_pose, "ego_vehicle");
    return transformed_pose;
  };

  // Transform the route into the robot's frame of reference
  std::vector<geometry_msgs::PoseStamped> transformed_route_poses;
  std::transform(transformation_begin, transformation_end, std::back_inserter(transformed_route_poses),
                 transform_global_pose_to_local);

  // add route waypoints into trajectory poses
  for (const auto& ps : transformed_route_poses)
  {
    trajectory.poses.emplace_back(ps.pose);
  }
  trajectory.header.frame_id = "ego_vehicle";
  trajectory.header.stamp = robot_pose.header.stamp;

  // Prune the portion of the global route that we've already passed so we don't
  // process it on the next iteration
  route_.waypoints.erase(std::begin(route_.waypoints), transformation_begin);

  if (trajectory.poses.empty())
  {
    ROS_ERROR("Resulting trajectory has 0 poses in it.");
    return false;
  }
  return true;
}

bool DummyTrajectoryPlanner::transformPose(const geometry_msgs::PoseStamped& in_pose,
                                           geometry_msgs::PoseStamped& out_pose, const std::string& frame)
{
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf_->transform(in_pose, out_pose, frame);
    out_pose.header.frame_id = frame;
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace trajectory_planner
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
// register this controller as a BaseController plugin
PLUGINLIB_EXPORT_CLASS(cmpf::trajectory_planner::DummyTrajectoryPlanner, cmpf::cmpf_core::BaseTrajectoryPlanner)