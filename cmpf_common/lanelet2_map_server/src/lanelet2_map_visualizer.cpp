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

#include "lanelet2_map_server/lanelet2_map_visualizer.hpp"

namespace cmpf
{
namespace common
{
Lanelet2MapVisualizer::Lanelet2MapVisualizer(const lanelet::LaneletMapPtr& lanelet2_map, ros::NodeHandle& mt_prv_nh,
                                             ros::NodeHandle& prv_nh)
{
  setMap(lanelet2_map);

  // register publishers
  markers_pub_ = mt_prv_nh.advertise<visualization_msgs::MarkerArray>("lanelet2_map_markers", 1);
}

Lanelet2MapVisualizer::~Lanelet2MapVisualizer()
{
}

void Lanelet2MapVisualizer::setMap(const lanelet::LaneletMapPtr& lanelet2_map)
{
  if (!lanelet2_map)
  {
    ROS_ERROR("[lanelet2_map_server] Error occured in lanelet2 map visualizer.");
    exit(1);
  }

  lanelet2_map_ = lanelet2_map;
  updateMapMarkers();
}

void Lanelet2MapVisualizer::renderMap()
{
  if (markers_pub_.getNumSubscribers() > 0)
  {
    markers_pub_.publish(map_marker_array_);
  }
}

void Lanelet2MapVisualizer::updateMapMarkers()
{
  // get all the lanelets in laneletLayer
  lanelet::ConstLanelets lanelets;
  for (auto li = lanelet2_map_->laneletLayer.begin(); li != lanelet2_map_->laneletLayer.end(); li++)
  {
    lanelets.push_back(*li);
  }

  // get the road lanelets
  lanelet::ConstLanelets road_lanelets;
  for (auto li = lanelets.begin(); li != lanelets.end(); li++)
  {
    lanelet::ConstLanelet ll = *li;

    if (ll.hasAttribute(lanelet::AttributeName::Subtype))
    {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == lanelet::AttributeValueString::Road)
      {
        road_lanelets.push_back(ll);
      }
    }
  }

  map_marker_array_.markers.clear();
  updateRoadLaneletsMarkers(road_lanelets, false);
}

void Lanelet2MapVisualizer::updateRoadLaneletsMarkers(const lanelet::ConstLanelets& road_lanelets,
                                                      bool render_centerline)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0f;
  color.g = 1.0f;
  color.b = 1.0f;
  color.a = 1.0f;

  for (auto li = road_lanelets.begin(); li != road_lanelets.end(); li++)
  {
    lanelet::ConstLanelet ll = *li;

    lanelet::ConstLineString3d left_ls = ll.leftBound();
    lanelet::ConstLineString3d right_ls = ll.rightBound();
    lanelet::ConstLineString3d center_ls = ll.centerline();

    visualization_msgs::Marker left_line_strip, right_line_strip, center_line_strip;

    lineStringToMarker(left_ls, left_line_strip, "map", "left_lane_bound", color, 0.2);
    lineStringToMarker(right_ls, right_line_strip, "map", "right_lane_bound", color, 0.2);
    map_marker_array_.markers.push_back(left_line_strip);
    map_marker_array_.markers.push_back(right_line_strip);
    if (render_centerline)
    {
      lineStringToMarker(center_ls, center_line_strip, "map", "center_lane_line", color, 0.2 * 0.5);
      map_marker_array_.markers.push_back(center_line_strip);
    }
  }
}

void Lanelet2MapVisualizer::lineStringToMarker(const lanelet::ConstLineString3d ls, visualization_msgs::Marker& marker,
                                               const std::string& frame_id, const std::string& ns,
                                               const std_msgs::ColorRGBA& color, const double width)
{
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.id = ls.id();

  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = width;

  marker.color = color;

  // fill out lane line
  for (auto i = ls.begin(); i != ls.end(); i++)
  {
    geometry_msgs::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = (*i).z();
    marker.points.push_back(p);
  }
}
}  // namespace common
}  // namespace cmpf