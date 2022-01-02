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
#include <ros/ros.h>

#include "cmpf_route_planner/route_planner_core.hpp"
#include "lanelet2_map_server/map_utils.hpp"

namespace cmpf
{
namespace route_planner
{
RoutePlanner::RoutePlanner()
{
}

RoutePlanner::~RoutePlanner()
{
}

void RoutePlanner::updateLanelet2Map(const cmpf_msgs::Lanelet2MapBin& map_bin_msg)
{
  lanelet2_map_ = std::make_shared<lanelet::LaneletMap>();
  common::map_utils::fromMsg(map_bin_msg, lanelet2_map_);
  traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::instance().create(lanelet::Locations::Germany,
                                                                                  lanelet::Participants::Vehicle);
  routing_graph_ = lanelet::routing::RoutingGraph::build(*lanelet2_map_, *traffic_rules_);
}

bool RoutePlanner::planRoute(nav_msgs::Path& route, const geometry_msgs::Pose& start_pose,
                             const geometry_msgs::Pose& goal_pose)
{
  lanelet::BasicPoint2d start_point(start_pose.position.x, start_pose.position.y);
  lanelet::BasicPoint2d goal_point(goal_pose.position.x, goal_pose.position.y);

  // find the nearest lanelets to the start and goal points
  lanelet::Lanelet start_lanelet = getNearestLanelet(start_point);
  lanelet::Lanelet goal_lanelet = getNearestLanelet(goal_point);

  // find the route excluding lane changes
  lanelet::Optional<lanelet::routing::LaneletPath> shortest_path_opt =
      routing_graph_->shortestPath(start_lanelet, goal_lanelet, 0u, false);

  if (!shortest_path_opt)
  {
    ROS_WARN("[cmpf_route_planner] Could not find path in lanelet map");
    return false;
  }

  lanelet::routing::LaneletPath shortest_path = shortest_path_opt.value();

  for (const auto& ll : shortest_path)
  {
    // const lanelet::traffic_rules::SpeedLimitInformation speed_limit = traffic_rules_->speedLimit(ll);
    const lanelet::ConstLineString3d centerline = ll.centerline();

    for (std::size_t i = 0; i < centerline.size(); ++i)
    {
      auto point = centerline[i];

      geometry_msgs::PoseStamped point_pose;
      point_pose.pose.position.x = point.x();
      point_pose.pose.position.y = point.y();
      point_pose.pose.position.z = point.z();

      route.poses.push_back(point_pose);
    }
  }

  return true;
}

lanelet::Lanelet RoutePlanner::getNearestLanelet(const lanelet::BasicPoint2d& point)
{
  std::vector<std::pair<double, lanelet::Lanelet>> closest_lanelets =
      lanelet::geometry::findNearest(lanelet2_map_->laneletLayer, point, 1);

  if (closest_lanelets.size() > 1)
  {
    ROS_WARN(
        "[cmpf_route_planner] Vehicle is positioned inside multiple lanelets, choosing the first lanelet as the "
        "starting point");
  }

  lanelet::Lanelet nearest_lanelet = closest_lanelets[0].second;

  return nearest_lanelet;
}
}  // namespace route_planner
}  // namespace cmpf