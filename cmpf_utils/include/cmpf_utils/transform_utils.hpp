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

namespace cmpf
{
namespace cmpf_utils
{
bool getRobotPose(geometry_msgs::PoseStamped& global_pose, tf2_ros::Buffer& tf_buffer, const std::string& base_frame,
                  const std::string& target_frame, double tolerance)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = base_frame;
  robot_pose.header.stamp = ros::Time();
  auto current_time = ros::Time::now();

  try
  {
    // use current time if possible (makes sure it's not in the future)
    if (tf_buffer.canTransform(target_frame, base_frame, current_time))
    {
      geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(target_frame, base_frame, current_time);
      tf2::doTransform(robot_pose, global_pose, transform);
    }
    // use the latest otherwise
    else
    {
      tf_buffer.transform(robot_pose, global_pose, target_frame);
    }
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error looking up target frame: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR("Connectivity Error looking up target frame: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR("Extrapolation Error looking up target frame: %s\n", ex.what());
    return false;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Failed to transform from %s to %s", global_pose.header.frame_id.c_str(), target_frame.c_str());
    return false;
  }

  // check global_pose timeout
  if (current_time.toSec() - global_pose.header.stamp.toSec() > tolerance)
  {
    ROS_WARN("[cmpf_controller_server] transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
             current_time.toSec(), global_pose.header.stamp.toSec(), tolerance);
    return false;
  }
  return true;
}
}  // namespace cmpf_utils
}  // namespace cmpf