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

#include <actionlib/server/simple_action_server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <pluginlib/class_loader.hpp>

#include "cmpf_msgs/Trajectory.h"
#include "cmpf_msgs/ComputeTrajectoryAction.h"
#include "cmpf_core/base_trajectory_planner.hpp"
#include "cmpf_utils/transform_utils.hpp"
#include "cmpf_utils/debug.hpp"

namespace cmpf
{
namespace trajectory_planner
{
class TrajectoryPlannerNodelet : public nodelet::Nodelet
{
public:
  TrajectoryPlannerNodelet()
  {
  }

  virtual ~TrajectoryPlannerNodelet()
  {
  }

  void onInit() override
  {
    NODELET_DEBUG(
        "Initializing "
        "cmpf::trajectory_planner::TrajectoryPlannerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();
    mt_prv_nh_ = getMTPrivateNodeHandle();

    // ros params
    private_nh_.param("plugin", planner_plugin_, std::string("cmpf_trajectory_planner/DummyTrajectoryPlanner"));
    private_nh_.param("transform_tolerance", transform_tolerance_, 0.3);

    // tfs
    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    // init pluginlib loader
    plugin_loader_ = std::make_unique<BasePluginLoader>("cmpf_core", "cmpf::cmpf_core::BaseTrajectoryPlanner");

    // create a controller
    try
    {
      planner_ = plugin_loader_->createUniqueInstance(planner_plugin_);
      ROS_INFO("Created trajectory_planner %s", planner_plugin_.c_str());
      planner_->initialize(plugin_loader_->getName(planner_plugin_), private_nh_, tf_.get());
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL(
          "Failed to create the %s trajectory_planner, are you sure it is properly "
          "registered and that the containing library is built? Exception: %s",
          planner_plugin_.c_str(), ex.what());
      exit(1);
    }

    // create action server
    action_server_ = std::make_unique<ActionServer>(
        private_nh_, "plan_trajectory",
        std::bind(&TrajectoryPlannerNodelet::actionServerCallBack, this, std::placeholders::_1), false);

    // start action server
    action_server_->start();
  }

  void actionServerCallBack(const cmpf_msgs::ComputeTrajectoryGoalConstPtr& goal)
  {
    ROS_DEBUG("[cmpf_trajectory_planner] Received a goal, begin computing trajectory.");

    cmpf_msgs::Trajectory trajectory;
    try
    {
      // update the route
      setPlannerRoute(goal->route);

      // compute trajectory
      computeTrajectory(trajectory);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR("[cmpf_trajectory_planner] Error occurs while computing trajectory. Exception: %s", ex.what());
      action_server_->setAborted();
      return;
    }

    ROS_DEBUG("[cmpf_trajectory_planner] Action Succeeded.");

    cmpf_msgs::ComputeTrajectoryResult result;
    result.trajectory = trajectory;
    action_server_->setSucceeded(result);
  }

  void setPlannerRoute(const cmpf_msgs::Route& route)
  {
    // TODO: Only update route if not the same
    planner_->setRoute(route);

    current_route_ = route;
  }

  void computeTrajectory(cmpf_msgs::Trajectory& trajectory)
  {
    // first get the current ego_vehicle pose in the map
    geometry_msgs::PoseStamped pose;
    if (!cmpf_utils::getRobotPose(pose, *tf_, "ego_vehicle", "map", transform_tolerance_))
    {
      throw std::runtime_error("Failed to obtain robot pose");
    }

    try
    {
      planner_->computeTrajectory(trajectory, pose);
    }
    catch (const std::exception& ex)
    {
      throw std::runtime_error(ex.what());
    }
  }

private:
  using ActionServer = actionlib::SimpleActionServer<cmpf_msgs::ComputeTrajectoryAction>;
  using BasePluginLoader = pluginlib::ClassLoader<cmpf_core::BaseTrajectoryPlanner>;

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle mt_prv_nh_;

  // tfs
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double transform_tolerance_;

  // controller related
  std::unique_ptr<BasePluginLoader> plugin_loader_;
  std::shared_ptr<cmpf_core::BaseTrajectoryPlanner> planner_;
  std::string planner_plugin_;
  cmpf_msgs::Route current_route_;

  // main action server
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace trajectory_planner
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::trajectory_planner::TrajectoryPlannerNodelet, nodelet::Nodelet)