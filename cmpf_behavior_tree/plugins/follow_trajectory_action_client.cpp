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

#include "cmpf_behavior_tree/plugins/follow_trajectory_action_client.hpp"

#include <memory>
#include <string>

namespace cmpf
{
namespace behavior_tree
{
FollowTrajectoryActionClient::FollowTrajectoryActionClient(const std::string& xml_tag_name,
                                                           const std::string& action_server_name,
                                                           const BT::NodeConfiguration& conf)
  : BTActionClientNode<cmpf_msgs::ComputeControlsAction>(xml_tag_name, action_server_name, conf)
{
}

void FollowTrajectoryActionClient::on_tick()
{
  getInput("trajectory", goal_.trajectory);
}

void FollowTrajectoryActionClient::on_wait_for_result()
{
  // Grab the new path
  cmpf_msgs::Trajectory new_trajectory;
  getInput("trajectory", new_trajectory);

  // Check if it is not same with the current one
  if (goal_.trajectory != new_trajectory)
  {
    // the action server on the next loop iteration
    goal_.trajectory = new_trajectory;
    goal_updated_ = true;
  }
}

}  // namespace behavior_tree
}  // namespace cmpf

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<cmpf::behavior_tree::FollowTrajectoryActionClient>(name, "follow_trajectory", config);
  };

  factory.registerBuilder<cmpf::behavior_tree::FollowTrajectoryActionClient>("FollowTrajectory", builder);
}