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

#include "cmpf_msgs/ComputeTrajectoryAction.h"
#include "cmpf_core/base_trajectory_planner.hpp"

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

    // ros params
    private_nh_.param("planner_frequency", planner_frequency_, 10.0);

    private_nh_.param("plugin", planner_plugin_, std::string("cmpf_trajectory_planner/DummyTrajectoryPlanner"));

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
        private_nh_, "plan_trajectory", std::bind(&TrajectoryPlannerNodelet::actionServerCallBack, this), false);
    action_server_->start();
  }

  void actionServerCallBack()
  {
    action_server_->setSucceeded();
  }

private:
  using ActionServer = actionlib::SimpleActionServer<cmpf_msgs::ComputeTrajectoryAction>;
  using BasePluginLoader = pluginlib::ClassLoader<cmpf_core::BaseTrajectoryPlanner>;

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // tfs
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // controller related
  std::unique_ptr<BasePluginLoader> plugin_loader_;
  std::shared_ptr<cmpf_core::BaseTrajectoryPlanner> planner_;
  std::string planner_plugin_;
  double planner_frequency_;

  // main action server
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace trajectory_planner
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::trajectory_planner::TrajectoryPlannerNodelet, nodelet::Nodelet)