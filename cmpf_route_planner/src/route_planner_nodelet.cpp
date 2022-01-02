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
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "cmpf_msgs/ComputeRouteToPoseAction.h"
#include "cmpf_msgs/Lanelet2Map.h"
#include "cmpf_msgs/Lanelet2MapBin.h"

#include "cmpf_route_planner/route_planner_core.hpp"

namespace cmpf
{
namespace route_planner
{
class RoutePlannerNodelet : public nodelet::Nodelet
{
public:
  RoutePlannerNodelet()
  {
  }

  virtual ~RoutePlannerNodelet()
  {
  }

  void onInit() override
  {
    NODELET_DEBUG(
        "Initializing "
        "cmpf::route_planner::RoutePlannerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // ros parameters

    // tfs
    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    // we wait here until we have a map server available
    // TODO: changable private ns "lanelet2_map_server"
    ROS_INFO("[cmpf_route_planner] Waiting lanelet2 map service...");
    ros::service::waitForService("lanelet2_map_server/get_lanelet2_map");
    ROS_INFO("[cmpf_route_planner] Success: Connected to lanelet2 map service.");

    // create service client to lanelet2 map server
    map_client_ =
        std::make_shared<ros::ServiceClient>(mt_nh_.serviceClient<cmpf_msgs::Lanelet2Map>("lanelet2_map_server/"
                                                                                          "get_lanelet2_map"));

    // get lanelet2 map
    cmpf_msgs::Lanelet2Map lanelet2_map_srv_msg;
    if (map_client_->call(lanelet2_map_srv_msg))
    {
      lanelet2_map_bin_msg_ = lanelet2_map_srv_msg.response.map_bin;
    }
    else
    {
      ROS_ERROR("[cmpf_route_planner] Failed to call lanelet2 map service.");
      exit(1);
    }

    // initialize route planner
    route_planner_ = std::make_shared<RoutePlanner>();
    route_planner_->updateLanelet2Map(lanelet2_map_bin_msg_);

    // create action server
    action_server_ = std::make_unique<ActionT>(
        private_nh_, "plan_route", std::bind(&RoutePlannerNodelet::actionServerCallBack, this, std::placeholders::_1),
        false);
    action_server_->start();
  }

  void actionServerCallBack(const cmpf_msgs::ComputeRouteToPoseGoalConstPtr& goal)
  {
    ROS_INFO("[cmpf_route_planner] Reveived new GOAL.");

    // get the current vehicle pose in the map
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::Pose start_pose;
    try
    {
      transform_stamped = tf_->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
      start_pose.position.x = transform_stamped.transform.translation.x;
      start_pose.position.y = transform_stamped.transform.translation.y;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("[cmpf_route_planner] %s", ex.what());
      action_server_->setAborted();
      return;
    }

    nav_msgs::Path route;
    if (!route_planner_->planRoute(route, start_pose, goal->goal_pose.pose))
    {
      ROS_WARN("[cmpf_route_planner] GOAL aborted. Failed to find the route.");
      action_server_->setAborted();
      return;
    }

    ROS_INFO("[cmpf_route_planner] GOAL succeeded.");
    cmpf_msgs::ComputeRouteToPoseResult result;
    result.path = route;
    action_server_->setSucceeded(result);
  }

private:
  using ActionT = actionlib::SimpleActionServer<cmpf_msgs::ComputeRouteToPoseAction>;

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // params

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // main action server
  std::unique_ptr<ActionT> action_server_;

  // lanelet2 map client
  std::shared_ptr<ros::ServiceClient> map_client_;
  cmpf_msgs::Lanelet2MapBin lanelet2_map_bin_msg_;

  std::shared_ptr<RoutePlanner> route_planner_;
};

}  // namespace route_planner
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::route_planner::RoutePlannerNodelet, nodelet::Nodelet)