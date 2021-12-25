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

#include "cmpf_behavior_tree/bt_action_client.hpp"
#include "cmpf_msgs/TrajectoryAction.h"

namespace cmpf {
namespace behavior_tree {

class FollowTrajectoryActionClient
    : public BTActionClientNode<cmpf_msgs::TrajectoryAction> {
 public:
  /**
   * @brief A constructor for cmpf::behavior_tree::FollowTrajectoryActionClient
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FollowTrajectoryActionClient(const std::string& xml_tag_name,
                               const std::string& action_server_name,
                               const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }
};

}  // namespace behavior_tree
}  // namespace cmpf