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

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>

namespace cmpf
{
namespace common
{
class Lanelet2MapVisualizer
{
public:
  /**
   * @brief Constructor for cmpf::common::Lanelet2MapVisualizer
   * @param lanelet2_map Lanelet2 Map
   * @param mt_prv_nh Multi-threaded private nodehandler
   * @param prv_nh Private nodehandler
   */
  Lanelet2MapVisualizer(const lanelet::LaneletMapPtr& lanelet2_map, ros::NodeHandle& mt_prv_nh,
                        ros::NodeHandle& prv_nh);

  /**
   * @brief Destructor
   */
  ~Lanelet2MapVisualizer();

  void setMap(const lanelet::LaneletMapPtr& lanelet2_map);

  void renderMap();

private:
  void updateMapMarkers();
  void updateRoadLaneletsMarkers(const lanelet::ConstLanelets& road_lanelets, bool render_centerline = false);

  void lineStringToMarker(const lanelet::ConstLineString3d ls, visualization_msgs::Marker& marker,
                          const std::string& frame_id, const std::string& ns, const std_msgs::ColorRGBA& color,
                          const double width);

  visualization_msgs::MarkerArray map_marker_array_;
  ros::Publisher markers_pub_;
  lanelet::LaneletMapPtr lanelet2_map_;
};
}  // namespace common
}  // namespace cmpf