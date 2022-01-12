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
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <pluginlib/class_loader.hpp>

#include "cmpf_msgs/ComputeControlsAction.h"
#include "cmpf_core/base_controller.hpp"
#include "cmpf_utils/transform_utils.hpp"

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
    NODELET_DEBUG(
        "Initializing "
        "cmpf::path_tracking_controller::ControllerServerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();
    mt_prv_nh_ = getMTPrivateNodeHandle();

    // ros params
    private_nh_.param("controller_frequency", controller_frequency_, 20.0);
    private_nh_.param("controller_plugin", controller_plugin_,
                      std::string("cmpf_decoupled_controller/DecoupledController"));
    private_nh_.param("transform_tolerance", transform_tolerance_, 0.3);
    private_nh_.param<bool>("publish_trajectory_marker", publish_trajectory_marker_, false);

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
      ROS_FATAL(
          "Failed to create the %s controller, are you sure it is properly "
          "registered and that the containing library is built? Exception: %s",
          controller_plugin_.c_str(), ex.what());
      exit(1);
    }

    // create publishers
    vehicle_control_cmd_pub_ = std::make_shared<ros::Publisher>(
        mt_nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1));
    trajectory_marker_pub_ =
        std::make_shared<ros::Publisher>(mt_prv_nh_.advertise<visualization_msgs::Marker>("trajectory_marker", 1));

    initTrajectoryMarker();
    current_trajectory_ = std::make_shared<cmpf_msgs::Trajectory>();

    // create action server
    action_server_ = std::make_unique<ActionServer>(
        mt_prv_nh_, "follow_trajectory",
        std::bind(&ControllerServerNodelet::actionServerCallBack, this, std::placeholders::_1), false);
    action_server_->start();
  }

  void actionServerCallBack(const cmpf_msgs::ComputeControlsGoalConstPtr& goal)
  {
    ROS_DEBUG("[cmpf_controller_server] Received a goal, begin computing control effort.");

    try
    {
      // update the goal trajectory
      setControllerTrajectory(goal->trajectory);

      ros::Rate loop_rate(controller_frequency_);
      while (ros::ok())
      {
        if (action_server_->isPreemptRequested())
        {
          if (action_server_->isNewGoalAvailable())
          {
            auto new_goal = action_server_->acceptNewGoal();
            ROS_DEBUG("[cmpf_controller_server] New goal accepted.");
            setControllerTrajectory(new_goal->trajectory);
          }
          else
          {
            ROS_INFO("[cmpf_controller_server] Goal was canceled. Stopping the robot.");
            action_server_->setPreempted();
            publishZeroVelocity();
            return;
          }
        }

        computeControl();

        if (isGoalReached())
        {
          ROS_INFO("[cmpf_controller_server] Reached the goal!");
          break;
        }

        if (!loop_rate.sleep())
        {
          ROS_WARN("[cmpf_controller_server] Control loop missed its desired rate of %.4fHz", controller_frequency_);
        }
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[cmpf_controller_server] Error occurs while executing control effort. Exception: %s", e.what());
      publishZeroVelocity();
      action_server_->setAborted();
      return;
    }

    ROS_INFO("[cmpf_controller_server] Action Succeeded!");
    publishZeroVelocity();
    action_server_->setSucceeded();
  }

  void setControllerTrajectory(const cmpf_msgs::Trajectory& trajectory)
  {
    // TODO: Handle goal error here
    controller_->setTrajectory(trajectory);

    // update current trajectory
    *current_trajectory_ = trajectory;
  }

  void publishZeroVelocity()
  {
    carla_msgs::CarlaEgoVehicleControl cmd_msg;
    cmd_msg.throttle = 0.0;
    cmd_msg.brake = 1.0;
    cmd_msg.steer = 0.0;
    cmd_msg.header.frame_id = "ego_vehicle";  // TODO: get from ros param
    cmd_msg.header.stamp = ros::Time::now();
    vehicle_control_cmd_pub_->publish(cmd_msg);
  }

  void computeControl()
  {
    // first get the current ego_vehicle pose in the map
    geometry_msgs::PoseStamped pose;
    if (!cmpf_utils::getRobotPose(pose, *tf_, "ego_vehicle", "map", transform_tolerance_))
    {
      throw std::runtime_error("Failed to obtain robot pose");
    }

    carla_msgs::CarlaEgoVehicleControl cmd_msg;
    try
    {
      controller_->computeVehicleControlCommands(pose, cmd_msg);
    }
    catch (const std::exception& ex)
    {
      throw std::runtime_error(ex.what());
    }

    // publish vehicle control commands
    // vehicle_control_cmd_pub_->publish(cmd_msg);

    // publish trajectory for visualization
    if (publish_trajectory_marker_)
      publishTrajectoryMarker();
  }

  bool isGoalReached()
  {
    return false;
  }

  void initTrajectoryMarker()
  {
    trajectory_marker_.ns = "trajectory";
    trajectory_marker_.id = 0;
    trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker_.action = visualization_msgs::Marker::ADD;
    trajectory_marker_.pose.orientation.w = 1.0;
    trajectory_marker_.scale.x = 1.5;
    std_msgs::ColorRGBA trajectory_color;
    trajectory_color.r = 0.0;
    trajectory_color.g = 1.0;
    trajectory_color.b = 0.0;
    trajectory_color.a = 1.0;
    trajectory_marker_.color = trajectory_color;
  }

  void publishTrajectoryMarker()
  {
    // publish trajectory marker
    if (trajectory_marker_pub_->getNumSubscribers() > 0)
    {
      // update trajectory marker
      trajectory_marker_.points.clear();
      for (std::size_t i = 0; i < current_trajectory_->poses.size(); ++i)
      {
        geometry_msgs::Point p;
        p.x = current_trajectory_->poses[i].position.x;
        p.y = current_trajectory_->poses[i].position.y;
        p.z = current_trajectory_->poses[i].position.z;

        trajectory_marker_.points.emplace_back(p);
      }

      trajectory_marker_.header.frame_id = current_trajectory_->header.frame_id;
      trajectory_marker_.header.stamp = ros::Time::now();

      trajectory_marker_pub_->publish(trajectory_marker_);
    }
  }

private:
  using ActionServer = actionlib::SimpleActionServer<cmpf_msgs::ComputeControlsAction>;
  using BaseControllerLoader = pluginlib::ClassLoader<cmpf_core::BaseController>;

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle mt_prv_nh_;

  // publisher
  std::shared_ptr<ros::Publisher> vehicle_control_cmd_pub_;
  std::shared_ptr<ros::Publisher> trajectory_marker_pub_;

  // tfs
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double transform_tolerance_;

  // controller related
  std::unique_ptr<BaseControllerLoader> bc_loader_;
  std::shared_ptr<cmpf_core::BaseController> controller_;
  std::string controller_plugin_;
  double controller_frequency_;
  std::shared_ptr<cmpf_msgs::Trajectory> current_trajectory_;
  visualization_msgs::Marker trajectory_marker_;
  bool publish_trajectory_marker_{ false };

  // main action server
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace path_tracking_controller
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::path_tracking_controller::ControllerServerNodelet, nodelet::Nodelet)