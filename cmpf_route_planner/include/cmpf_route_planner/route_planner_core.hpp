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

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include "cmpf_msgs/Lanelet2MapBin.h"

namespace cmpf
{
namespace route_planner
{
class RoutePlanner
{
public:
  RoutePlanner(double route_resolution);
  ~RoutePlanner();

  void updateLanelet2Map(const cmpf_msgs::Lanelet2MapBin& map_bin_msg);

  /**
   * @brief Plan the route in the lanelet2 map. This function assumes lanelet2 map is already updated
   * (i.e., using "updateLanelet2Map" method).
   * @param route Planned route
   * @param start_pose Starting point
   * @param goal_pose Destination point
   * @return True if successfully find the route otherwise false
   */
  bool planRoute(nav_msgs::Path& route, const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose);

private:
  std::vector<lanelet::Id> getRoute(const std::vector<lanelet::Id>& from_id, const std::vector<lanelet::Id>& to_id);
  std::size_t getNearestLaneletIdx(const lanelet::ConstLanelets& lanelets, const lanelet::Point3d& point);
  lanelet::Lanelet getNearestLanelet(const lanelet::Point3d& point);
  geometry_msgs::PoseStamped fromPoseToPoseStamped(const geometry_msgs::Pose& p);
  lanelet::Point3d fromPoseStampedToLLPoint3D(const geometry_msgs::PoseStamped& ps);
  std::pair<size_t, size_t> findNearestIndexPair(const std::vector<double>& accumulated_lengths,
                                                 const double target_length);
  std::vector<lanelet::BasicPoint3d> resamplePoints(const lanelet::ConstLineString3d& line_string,
                                                    const unsigned int num_segments);
  lanelet::LineString3d generateCenterline(const lanelet::ConstLanelet& lanelet_obj);

  /**
   * Find element in iterator with the minimum calculated value
   */
  template <typename Iter, typename Getter>
  Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end)
    {
      return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it)
    {
      auto comp = getCompareVal(*it);
      if (comp < lowest)
      {
        lowest = comp;
        lowest_it = it;
      }
    }
    return lowest_it;
  }

  double route_resolution_{ 2.0 };
  lanelet::LaneletMapPtr lanelet2_map_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphUPtr routing_graph_;
};
}  // namespace route_planner
}  // namespace cmpf