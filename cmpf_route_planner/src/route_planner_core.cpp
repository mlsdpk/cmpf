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
RoutePlanner::RoutePlanner(double route_resolution) : route_resolution_{ route_resolution }
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

    // generate centerline with fine route resolution
    const auto centerline = generateCenterline(ll);

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

  // now we need to remove points that stay outside of start and goal points
  // find the closest pose on the path to the robot (start pose)
  auto transformation_begin = min_by(route.poses.begin(), route.poses.end(), [&](const geometry_msgs::PoseStamped& ps) {
    return lanelet::geometry::distance2d(start_point, fromPoseStampedToLLPoint3D(ps));
  });

  // prune the route before start point
  route.poses.erase(std::begin(route.poses), transformation_begin);

  // find the closest pose on the path to the robot (goal pose)
  auto transformation_end = min_by(route.poses.begin(), route.poses.end(), [&](const geometry_msgs::PoseStamped& ps) {
    return lanelet::geometry::distance2d(goal_point, fromPoseStampedToLLPoint3D(ps));
  });

  // prune the route after goal point
  if (transformation_end != std::end(route.poses))
    route.poses.erase(std::next(transformation_end, 1), std::end(route.poses));

  // insert start point to route
  route.poses.insert(std::begin(route.poses), fromPoseToPoseStamped(start_pose));

  // add last goal point to route
  route.poses.push_back(fromPoseToPoseStamped(goal_pose));

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

lanelet::Lanelet RoutePlanner::getNearestLanelet(const lanelet::Point3d& point)
{
  std::vector<std::pair<double, lanelet::Lanelet>> closest_lanelets =
      lanelet::geometry::findNearest(lanelet2_map_->laneletLayer, point.basicPoint2d(), 1);

  if (closest_lanelets.size() > 1)
  {
    ROS_WARN(
        "[cmpf_route_planner] Vehicle is positioned inside multiple lanelets, choosing the first lanelet as the "
        "starting point");
  }

  lanelet::Lanelet nearest_lanelet = closest_lanelets[0].second;

  return nearest_lanelet;
}

lanelet::Point3d RoutePlanner::fromPoseStampedToLLPoint3D(const geometry_msgs::PoseStamped& ps)
{
  return lanelet::Point3d(lanelet::InvalId, ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
}

geometry_msgs::PoseStamped RoutePlanner::fromPoseToPoseStamped(const geometry_msgs::Pose& p)
{
  geometry_msgs::PoseStamped ps;
  ps.pose.position.x = p.position.x;
  ps.pose.position.y = p.position.y;
  ps.pose.position.z = p.position.z;
  return ps;
}

std::pair<size_t, size_t> RoutePlanner::findNearestIndexPair(const std::vector<double>& accumulated_lengths,
                                                             const double target_length)
{
  // List size
  const auto N = accumulated_lengths.size();

  // Front
  if (target_length < accumulated_lengths.at(1))
  {
    return std::make_pair(0, 1);
  }

  // Back
  if (target_length > accumulated_lengths.at(N - 2))
  {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (auto i = 1; i < N; ++i)
  {
    if (accumulated_lengths.at(i - 1) <= target_length && target_length <= accumulated_lengths.at(i))
    {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  throw std::runtime_error("No nearest point found.");
}

std::vector<lanelet::BasicPoint3d> RoutePlanner::resamplePoints(const lanelet::ConstLineString3d& line_string,
                                                                const unsigned int num_segments)
{
  // calculate length
  const auto line_length = lanelet::geometry::length(line_string);

  // calculate segment distances
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);

  for (size_t i = 1; i < line_string.size(); ++i)
  {
    const auto distance = lanelet::geometry::distance2d(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }

  // calculate accumulated length
  std::vector<double> accumulated_lengths{ 0 };
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(std::begin(segment_distances), std::end(segment_distances), std::back_inserter(accumulated_lengths));

  // Create each segment
  std::vector<lanelet::BasicPoint3d> resampled_points;
  for (auto i = 0; i <= num_segments; ++i)
  {
    // Find two nearest points
    const auto target_length = (static_cast<double>(i) / num_segments) * line_length;
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point = back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.push_back(target_point);
  }

  return resampled_points;
}

lanelet::LineString3d RoutePlanner::generateCenterline(const lanelet::ConstLanelet& lanelet_obj)
{
  // Get length of longer border
  const double left_length = lanelet::geometry::length(lanelet_obj.leftBound());
  const double right_length = lanelet::geometry::length(lanelet_obj.rightBound());
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const auto num_segments = std::max(static_cast<unsigned>(std::ceil(longer_distance / route_resolution_)), 1u);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (int i = 0; i < num_segments + 1; i++)
  {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2;
    const lanelet::Point3d center_point(lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
                                        center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}
}  // namespace route_planner
}  // namespace cmpf