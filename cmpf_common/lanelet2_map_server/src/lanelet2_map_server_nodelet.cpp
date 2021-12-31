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

#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_core/LaneletMap.h>

#include <memory>

#include "cmpf_msgs/Lanelet2Map.h"
#include "lanelet2_map_server/map_utils.hpp"
#include "lanelet2_map_server/lanelet2_map_visualizer.hpp"

namespace cmpf
{
namespace common
{
class Lanelet2MapServerNodelet : public nodelet::Nodelet
{
public:
  Lanelet2MapServerNodelet()
  {
  }

  virtual ~Lanelet2MapServerNodelet()
  {
  }

  void onInit() override
  {
    NODELET_DEBUG(
        "Initializing "
        "cmpf::common::Lanelet2MapServerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_prv_nh_ = getMTPrivateNodeHandle();
    prv_nh_ = getPrivateNodeHandle();

    // ros parameters
    prv_nh_.param<double>("map_markers_publish_frequency", map_markers_publish_frequency_, 1.0);

    // tfs
    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    if (getMyArgv().size() > 0)
      map_file_name_ = getMyArgv()[0];
    else
    {
      ROS_ERROR("[lanelet2_map_server] Lanelet2 map file name is not provided.");
      exit(1);
    }

    // load the map
    try
    {
      ROS_INFO("[lanelet2_map_server] Loading lanelet2 map from %s", map_file_name_.c_str());
      lanelet::projection::UtmProjector projector(lanelet::Origin({ 0.0, 0.0 }));
      lanelet2_map_ = lanelet::load(map_file_name_, projector);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR("[lanelet2_map_server] Error loading lanelet2 map. Exception: %s", ex.what());
      exit(1);
    }

    // initialize visualizer
    map_visualizer_ = std::make_shared<Lanelet2MapVisualizer>(lanelet2_map_, mt_prv_nh_, prv_nh_);

    // service server
    map_request_srv_ =
        mt_prv_nh_.advertiseService("get_lanelet2_map", &Lanelet2MapServerNodelet::mapRequestService, this);

    // timer
    map_markers_pub_timer_ = mt_prv_nh_.createWallTimer(ros::WallDuration(1.0 / map_markers_publish_frequency_),
                                                        &Lanelet2MapServerNodelet::mapMarkersPublisherTimerCB, this);
  }

private:
  bool mapRequestService(cmpf_msgs::Lanelet2MapRequest& req, cmpf_msgs::Lanelet2MapResponse& res)
  {
    ROS_INFO("Lanelet2 map request received.");

    res.map_bin.header.stamp = ros::Time::now();
    res.map_bin.header.frame_id = "map";
    map_utils::toMsg(lanelet2_map_, &res.map_bin);
    return true;
  }

  void mapMarkersPublisherTimerCB(const ros::WallTimerEvent& event)
  {
    map_visualizer_->renderMap();
  }

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_prv_nh_;
  ros::NodeHandle prv_nh_;

  // Service server
  ros::ServiceServer map_request_srv_;

  // tfs
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // lanelet2 related
  lanelet::LaneletMapPtr lanelet2_map_;
  std::string map_file_name_;

  // lanelet2 map markers related
  std::shared_ptr<Lanelet2MapVisualizer> map_visualizer_;
  ros::WallTimer map_markers_pub_timer_;
  double map_markers_publish_frequency_;
};

}  // namespace common
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::common::Lanelet2MapServerNodelet, nodelet::Nodelet)