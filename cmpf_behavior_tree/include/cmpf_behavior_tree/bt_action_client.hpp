/************************************************************************
  Copyright 2018 Intel Corporation

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

// Modified by Phone Thiha Kyaw

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace cmpf {
namespace behavior_tree {

/**
 * @brief Abstract class representing an action-client based BT node
 * @tparam ActionT Type of action
 */
template <class ActionT>
class BTActionClientNode : public BT::ActionNodeBase {
 public:
  BTActionClientNode(const std::string& xml_tag_name,
                     const std::string& action_server_name,
                     const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf),
        action_server_name_(action_server_name) {
    // get the global ns retrieved from blackboard
    std::string ns = config().blackboard->get<std::string>("ns");

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_server_name_ = remapped_action_name;
    }
    action_server_name_ = ns + action_server_name_;
    createActionClient(action_server_name_);
    ROS_INFO("\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BTActionClientNode() = delete;

  virtual ~BTActionClientNode() {}

  /**
   * @brief Create instance of an action client
   * @param action_server_name Action server name to create client for
   */
  void createActionClient(const std::string& action_server_name) {
    // Now that we have the ROS node to use, create the action client for this
    // BT action
    action_client_ = std::make_shared<actionlib::SimpleActionClient<ActionT>>(
        action_server_name);

    // Make sure the server is actually there before continuing
    ROS_INFO("Waiting for \"%s\" action server", action_server_name.c_str());
    action_client_->waitForServer();
  }

  /**
   * @brief Any subclass of BTActionClientNode that accepts parameters must
   * provide a providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Action server name"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the
   * blackboard
   */
  virtual void on_tick() {}

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   */
  virtual void on_wait_for_result() {}

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return
   * another value
   */
  virtual BT::NodeStatus on_success() {
    ROS_INFO("Success\"%s\" action server", action_server_name_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation whe the action is
   * aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return
   * another value
   */
  virtual BT::NodeStatus on_aborted() { return BT::NodeStatus::FAILURE; }

  /**
   * @brief Function to perform some user-defined operation when the action is
   * cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return
   * another value
   */
  virtual BT::NodeStatus on_cancelled() { return BT::NodeStatus::SUCCESS; }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override {
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // user defined callback
      on_tick();

      //   send_new_goal();
    }
    return on_success();
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case,
   * we make sure to cancel the ROS action if it is still running.
   */
  void halt() override { setStatus(BT::NodeStatus::IDLE); }

 private:
  std::string action_server_name_;
  typename std::shared_ptr<actionlib::SimpleActionClient<ActionT>>
      action_client_;
};

}  // namespace behavior_tree
}  // namespace cmpf
