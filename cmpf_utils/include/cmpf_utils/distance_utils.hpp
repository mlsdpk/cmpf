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

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include "cmpf_msgs/Waypoint.h"

namespace cmpf
{
namespace cmpf_utils
{
/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(const geometry_msgs::Pose& pos1, const geometry_msgs::Pose& pos2)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;
  double dz = pos1.position.z - pos2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(const geometry_msgs::PoseStamped& pos1, const geometry_msgs::PoseStamped& pos2)
{
  return euclidean_distance(pos1.pose, pos2.pose);
}

inline double euclidean_distance(const double& x1, const double& y1, const double& x2, const double& y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

inline double euclidean_distance(const geometry_msgs::PoseStamped& pos, const cmpf_msgs::Waypoint& wp)
{
  return euclidean_distance(pos.pose.position.x, pos.pose.position.y, wp.x, wp.y);
}

}  // namespace cmpf_utils
}  // namespace cmpf