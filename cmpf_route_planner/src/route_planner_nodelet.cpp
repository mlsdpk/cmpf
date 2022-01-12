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
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <thread>

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
    map_client_thread_.join();
  }

  void onInit() override
  {
    NODELET_DEBUG(
        "Initializing "
        "cmpf::route_planner::RoutePlannerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    mt_prv_nh_ = getMTPrivateNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // ros parameters
    private_nh_.param<double>("route_resolution", route_resolution_, 2.0);
    private_nh_.param<bool>("publish_route_markers", publish_route_markers_, false);

    // tfs
    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    // since onInit() method of nodelet cannot be blocked, we wait for map server availiability on another thread
    map_client_thread_ = std::thread(&RoutePlannerNodelet::createServiceClient, this);

    // initialize route planner
    route_planner_ = std::make_shared<RoutePlanner>(route_resolution_);

    // create action server
    action_server_ = std::make_unique<ActionT>(
        mt_prv_nh_, "plan_route", std::bind(&RoutePlannerNodelet::actionServerCallBack, this, std::placeholders::_1),
        false);
    action_server_->start();

    // init route marker
    route_marker_.ns = "route";
    route_marker_.id = 0;
    route_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    route_marker_.action = visualization_msgs::Marker::ADD;
    route_marker_.pose.orientation.w = 1.0;
    route_marker_.scale.x = 0.5;
    route_marker_.scale.y = 0.5;
    route_marker_.scale.z = 0.5;
    std_msgs::ColorRGBA route_color;
    route_color.r = 0.58;
    route_color.g = 0.0;
    route_color.b = 0.83;
    route_color.a = 1.0;
    route_marker_.color = route_color;

    // create route marker publisher
    route_marker_publisher_ = mt_prv_nh_.advertise<visualization_msgs::Marker>("route_marker", 1);

    // create route publisher timer
    route_marker_publish_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(1.0), &RoutePlannerNodelet::routeMarkerPublisherTimerCB, this);
  }

  void createServiceClient()
  {
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

    route_planner_->updateLanelet2Map(lanelet2_map_bin_msg_);
  }

  void actionServerCallBack(const cmpf_msgs::ComputeRouteToPoseGoalConstPtr& goal)
  {
    ROS_INFO("[cmpf_route_planner] Reveived new GOAL.");

    // get the current vehicle pose in the map
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::Pose start_pose;
    try
    {
      // TODO: changeable base_frame and world frame ids
      transform_stamped = tf_->lookupTransform("map", "ego_vehicle", ros::Time(0), ros::Duration(2.0));
      start_pose.position.x = transform_stamped.transform.translation.x;
      start_pose.position.y = transform_stamped.transform.translation.y;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("[cmpf_route_planner] %s", ex.what());
      action_server_->setAborted();
      return;
    }

    cmpf_msgs::Route route;
    if (!route_planner_->planRoute(route, start_pose, goal->goal_pose.pose))
    {
      ROS_WARN("[cmpf_route_planner] GOAL aborted. Failed to find the route.");
      action_server_->setAborted();
      return;
    }

    route.header.frame_id = "map";
    route.header.stamp = ros::Time::now();

    ROS_INFO("[cmpf_route_planner] GOAL succeeded.");
    cmpf_msgs::ComputeRouteToPoseResult result;
    result.route = route;
    action_server_->setSucceeded(result);

    curr_route_ = std::make_shared<cmpf_msgs::Route>(route);
    route_marker_updated_ = false;
    route_exists_ = true;
  }

  void routeMarkerPublisherTimerCB(const ros::WallTimerEvent& event)
  {
    if (publish_route_markers_ && route_exists_)
    {
      if (!route_marker_updated_)
      {
        // update route marker
        route_marker_.points.clear();
        for (std::size_t i = 0; i < curr_route_->waypoints.size(); ++i)
        {
          geometry_msgs::Point p;
          p.x = curr_route_->waypoints[i].x;
          p.y = curr_route_->waypoints[i].y;
          p.z = 0.0;

          route_marker_.points.emplace_back(p);
        }

        route_marker_updated_ = true;
      }

      // publish route marker
      if (route_marker_publisher_.getNumSubscribers() > 0)
      {
        route_marker_.header.frame_id = curr_route_->header.frame_id;
        route_marker_.header.stamp = ros::Time::now();
        route_marker_publisher_.publish(route_marker_);
      }
    }
  }

private:
  using ActionT = actionlib::SimpleActionServer<cmpf_msgs::ComputeRouteToPoseAction>;

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle mt_prv_nh_;
  ros::NodeHandle private_nh_;

  // params
  double route_resolution_;
  bool publish_route_markers_{ false };

  // publisher
  ros::Publisher route_marker_publisher_;

  // timer
  ros::WallTimer route_marker_publish_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // main action server
  std::unique_ptr<ActionT> action_server_;

  // lanelet2 map client
  std::thread map_client_thread_;
  std::shared_ptr<ros::ServiceClient> map_client_;
  cmpf_msgs::Lanelet2MapBin lanelet2_map_bin_msg_;

  std::shared_ptr<RoutePlanner> route_planner_;
  std::atomic_bool route_exists_{ false };
  std::atomic_bool route_marker_updated_{ false };
  std::shared_ptr<cmpf_msgs::Route> curr_route_;
  visualization_msgs::Marker route_marker_;
};

}  // namespace route_planner
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::route_planner::RoutePlannerNodelet, nodelet::Nodelet)