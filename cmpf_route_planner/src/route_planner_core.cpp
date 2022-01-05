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
  const lanelet::Point3d start_point(lanelet::utils::getId(), start_pose.position.x, start_pose.position.y, 0.0);
  const lanelet::Point3d goal_point(lanelet::utils::getId(), goal_pose.position.x, goal_pose.position.y, 0.0);

  std::vector<lanelet::Id> lane_start, lane_end;

  ROS_INFO("start x: %s , start y: %s", std::to_string(start_pose.position.x).c_str(),
           std::to_string(start_pose.position.y).c_str());
  ROS_INFO("goal x: %s , goal y: %s", std::to_string(goal_pose.position.x).c_str(),
           std::to_string(goal_pose.position.y).c_str());

  auto nearest_lanelets_to_start = lanelet2_map_->laneletLayer.nearest(start_point, 2);
  if (nearest_lanelets_to_start.empty())
  {
    ROS_ERROR("[cmpf_route_planner] Could not find nearest lanelet to start pose.");
  }
  else
  {
    for (const auto& ll : nearest_lanelets_to_start)
    {
      lane_start.push_back(ll.id());
    }
  }

  auto nearest_lanelets_to_goal = lanelet2_map_->laneletLayer.nearest(goal_point, 2);
  if (nearest_lanelets_to_goal.empty())
  {
    ROS_ERROR("[cmpf_route_planner] Could not find nearest lanelet to goal pose.");
  }
  else
  {
    for (const auto& ll : nearest_lanelets_to_goal)
    {
      lane_end.push_back(ll.id());
    }
  }

  // find the shortest route
  std::vector<lanelet::Id> shortest_route = getRoute(lane_start, lane_end);

  if (!(shortest_route.size() > 0))
  {
    ROS_WARN("[cmpf_route_planner] Could not find path in lanelet map");
    return false;
  }

  lanelet::ConstLanelets lanelets;
  for (const auto& route_id : shortest_route)
  {
    const auto lane = lanelet2_map_->laneletLayer.get(route_id);
    lanelets.push_back(lane);
  }

  const auto start_idx = getNearestLaneletIdx(lanelets, start_point);
  for (std::size_t i = start_idx; i < lanelets.size(); ++i)
  {
    const auto& ll = lanelets.at(i);
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
  /*
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
    lanelet::LaneletSequence continuous_lane = shortest_path.getRemainingLane(shortest_path.begin());

    if (continuous_lane.size() != shortest_path.size())
    {
      ROS_WARN("[cmpf_route_planner] This route contains a lane change which is currently unsupported");
      return false;
    }

    ROS_INFO("[cmpf_route_planner] Found a path containing %lu lanelets", shortest_path.size());

    for (const auto& ll : continuous_lane.lanelets())
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

    int wp_id = 0;
    int smallest_goal_wp_id = 0;
    double smallest_goal_dist = std::numeric_limits<double>::infinity();
    for (const auto& wp : route.poses)
    {
      // Calculate distance to goal point
      double goal_dist = std::hypot(goal_point.x() - wp.pose.position.x, goal_point.y() - wp.pose.position.y);
      if (goal_dist < smallest_goal_dist)
      {
        smallest_goal_dist = goal_dist;
        smallest_goal_wp_id = wp_id;
      }
      wp_id++;
    }

    // Trim waypoints to stop at goal point
    route.poses.resize(smallest_goal_wp_id);
    */

  return true;
}

std::vector<lanelet::Id> RoutePlanner::getRoute(const std::vector<lanelet::Id>& from_id,
                                                const std::vector<lanelet::Id>& to_id)
{
  double shortest_len = std::numeric_limits<double>::infinity();
  std::vector<lanelet::Id> shortest_route;

  for (auto start_id : from_id)
  {
    for (auto end_id : to_id)
    {
      lanelet::ConstLanelet from_ll = lanelet2_map_->laneletLayer.get(start_id);
      lanelet::ConstLanelet to_ll = lanelet2_map_->laneletLayer.get(end_id);
      lanelet::Optional<lanelet::routing::Route> route = routing_graph_->getRoute(from_ll, to_ll, 0);

      if (route)
      {
        lanelet::routing::LaneletPath shortest_path = route->shortestPath();
        lanelet::LaneletSequence full_lane = route->fullLane(from_ll);
        const auto route_len = route->length2d();
        if (!shortest_path.empty() && !full_lane.empty() && shortest_len > route_len)
        {
          shortest_len = route_len;
          shortest_route = full_lane.ids();
        }
      }
    }
  }

  return shortest_route;
}

std::size_t RoutePlanner::getNearestLaneletIdx(const lanelet::ConstLanelets& lanelets, const lanelet::Point3d& point)
{
  double min_dist = std::numeric_limits<double>::infinity();
  std::size_t min_idx = 0;
  for (std::size_t i = 0; i < lanelets.size(); ++i)
  {
    const auto& llt = lanelets.at(i);
    const auto& p2d = lanelet::Point2d(lanelet::InvalId, point.x(), point.y()).basicPoint2d();
    const double dist = lanelet::geometry::distanceToCenterline2d(llt, p2d);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
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