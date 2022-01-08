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

#include "cmpf_behavior_tree/plugins/compute_route_action_client.hpp"

#include <memory>
#include <string>

namespace cmpf
{
namespace behavior_tree
{
ComputeRouteActionClient::ComputeRouteActionClient(const std::string& xml_tag_name,
                                                   const std::string& action_server_name,
                                                   const BT::NodeConfiguration& conf)
  : BTActionClientNode<cmpf_msgs::ComputeRouteToPoseAction>(xml_tag_name, action_server_name, conf)
{
}

void ComputeRouteActionClient::on_tick()
{
  getInput("goal_pose", goal_.goal_pose);
}

BT::NodeStatus ComputeRouteActionClient::on_success()
{
  ROS_INFO("[cmpf_behavior_tree] Success %s action server", action_server_name_.c_str());
  setOutput("route", result_.route);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace behavior_tree
}  // namespace cmpf

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<cmpf::behavior_tree::ComputeRouteActionClient>(name, "compute_route", config);
  };

  factory.registerBuilder<cmpf::behavior_tree::ComputeRouteActionClient>("ComputeRoute", builder);
}