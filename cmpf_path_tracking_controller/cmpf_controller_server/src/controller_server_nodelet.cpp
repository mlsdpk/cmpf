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

#include "cmpf_core/base_controller.hpp"
#include "cmpf_msgs/TrajectoryAction.h"

namespace cmpf
{
namespace path_tracking_controller
{
class ControllerServerNodelet : public nodelet::Nodelet
{
public:
  ControllerServerNodelet()
  {
  }

  virtual ~ControllerServerNodelet()
  {
  }

  void onInit() override
  {
    NODELET_DEBUG("Initializing "
                  "cmpf::path_tracking_controller::ControllerServerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // ros params
    private_nh_.param("controller_frequency", controller_frequency_, 20.0);

    private_nh_.param("controller_plugin", controller_plugin_,
                      std::string("cmpf_decoupled_controller/DecoupledController"));

    // tfs
    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    // init pluginlib loader
    bc_loader_ = std::make_unique<BaseControllerLoader>("cmpf_core", "cmpf::cmpf_core::BaseController");

    // create a controller
    try
    {
      controller_ = bc_loader_->createUniqueInstance(controller_plugin_);
      ROS_INFO("Created path_tracking_controller %s", controller_plugin_.c_str());
      controller_->initialize(bc_loader_->getName(controller_plugin_), private_nh_, tf_.get());
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s controller, are you sure it is properly "
                "registered and that the containing library is built? Exception: %s",
                controller_plugin_.c_str(), ex.what());
      exit(1);
    }

    // create action server
    action_server_ = std::make_unique<ActionServer>(
        private_nh_, "follow_trajectory", std::bind(&ControllerServerNodelet::actionServerCallBack, this), false);
    action_server_->start();
  }

  void actionServerCallBack()
  {
    action_server_->setSucceeded();
  }

private:
  using ActionServer = actionlib::SimpleActionServer<cmpf_msgs::TrajectoryAction>;
  using BaseControllerLoader = pluginlib::ClassLoader<cmpf_core::BaseController>;

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // tfs
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // controller related
  std::unique_ptr<BaseControllerLoader> bc_loader_;
  std::shared_ptr<cmpf_core::BaseController> controller_;
  std::string controller_plugin_;
  double controller_frequency_;

  // main action server
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace path_tracking_controller
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::path_tracking_controller::ControllerServerNodelet, nodelet::Nodelet)