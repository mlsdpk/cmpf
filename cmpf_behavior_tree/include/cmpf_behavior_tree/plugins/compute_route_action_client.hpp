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

#include <geometry_msgs/PoseStamped.h>

#include "cmpf_msgs/ComputeRouteToPoseAction.h"
#include "cmpf_msgs/ComputeRouteToPoseActionGoal.h"
#include "cmpf_msgs/Route.h"

#include "cmpf_behavior_tree/bt_action_client.hpp"

namespace cmpf
{
namespace behavior_tree
{
class ComputeRouteActionClient : public BTActionClientNode<cmpf_msgs::ComputeRouteToPoseAction>
{
public:
  /**
   * @brief A constructor for cmpf::behavior_tree::ComputeRouteActionClient
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputeRouteActionClient(const std::string& xml_tag_name, const std::string& action_server_name,
                           const BT::NodeConfiguration& conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Put the output path on the blackboard upon successful
   * completion of the action.
   * @return BT::NodeStatus Returns SUCCESS
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<geometry_msgs::PoseStamped>("goal_pose", "Destination to plan the route"),
                                BT::OutputPort<cmpf_msgs::Route>("route", "Generated route in the map") });
  }
};

}  // namespace behavior_tree
}  // namespace cmpf