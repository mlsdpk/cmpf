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

#include <actionlib/server/simple_action_server.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <memory>

#include "cmpf_msgs/NavigateToPoseAction.h"

namespace cmpf
{
namespace behavior_tree
{
class BTActionServerNodelet : public nodelet::Nodelet
{
public:
  BTActionServerNodelet()
  {
  }
  virtual ~BTActionServerNodelet()
  {
  }

  void onInit() override
  {
    NODELET_DEBUG(
        "Initializing "
        "cmpf::behavior_tree::BTActionServerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // ros parameters
    // get default bt xml file or retrieve from parameter server
    std::string pkg_path = ros::package::getPath("cmpf_behavior_tree");
    bt_xml_default_file_path_ = pkg_path + "/behavior_trees/default_bt.xml";
    private_nh_.param("bt_xml_file_path", bt_xml_file_path_, bt_xml_default_file_path_);

    private_nh_.param("bt_loop_frequency", bt_loop_frequency_, 100.0);

    // get all the BT plugins
    const std::vector<std::string> bt_plugin_libs = { "cmpf_compute_route_action_client_bt_node",
                                                      "cmpf_follow_trajectory_action_client_bt_node" };

    // register the BT plugin nodes in BehaviorTreeFactory
    BT::SharedLibrary loader;
    for (const auto& p : bt_plugin_libs)
    {
      factory_.registerFromPlugin(loader.getOSName(p));
    }

    // create blackboard to share across all the tree nodes
    black_board_ = BT::Blackboard::create();
    black_board_->set<std::string>("ns", nh_.getNamespace());

    // load the default behavior tree
    if (!loadBehaviorTree(bt_xml_file_path_))
    {
      exit(1);
    }

    // create action server
    action_server_ = std::make_unique<ActionT>(
        private_nh_, "bt_server", std::bind(&BTActionServerNodelet::actionServerCallBack, this, std::placeholders::_1),
        false);
    action_server_->start();
  }

  void actionServerCallBack(const cmpf_msgs::NavigateToPoseGoalConstPtr& goal)
  {
    // get the goal and process it
    ROS_INFO("[cmpf_behavior_tree] Reveived new GOAL.");

    // if the goal behavior tree path is empty, we use the previously loaded tree but warn the user
    if (goal->behavior_tree.empty())
    {
      ROS_WARN(
          "[cmpf_behavior_tree] Goal contains empty behavior tree path. Using the previously loaded behavior tree from "
          "%s...",
          bt_xml_file_path_.c_str());
    }
    // load the goal behavior tree
    else if (!loadBehaviorTree(goal->behavior_tree))
    {
      // if we cannot load the behavior tree, cancel the goal request
      ROS_INFO("[cmpf_behavior_tree] GOAL aborted.");
      action_server_->setAborted();
      return;
    }

    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    ros::Rate loop_rate(bt_loop_frequency_);
    while (ros::ok() && result == BT::NodeStatus::RUNNING)
    {
      if (action_server_->isPreemptRequested())
      {
        ROS_INFO("[cmpf_behavior_tree] GOAL canceled.");
        action_server_->setPreempted();
        tree_.rootNode()->halt();
        break;
      }

      result = tree_.tickRoot();
      loop_rate.sleep();
    }

    switch (result)
    {
      case BT::NodeStatus::SUCCESS:
        ROS_INFO("[cmpf_behavior_tree] GOAL succeeded.");
        action_server_->setSucceeded();
        break;

      case BT::NodeStatus::FAILURE:
        ROS_INFO("[cmpf_behavior_tree] GOAL failed.");
        action_server_->setAborted();
        break;

      default:
        break;
    }
  }

private:
  using ActionT = actionlib::SimpleActionServer<cmpf_msgs::NavigateToPoseAction>;

  bool loadBehaviorTree(const std::string& bt_xml_file_path)
  {
    auto file_path = bt_xml_file_path;
    // if behavior tree path is empty, we will just use the default one
    if (file_path.empty())
    {
      ROS_WARN(
          "[cmpf_behavior_tree] Empty behavior tree filename is given. Using the default behavior "
          "tree.");
      file_path = bt_xml_default_file_path_;
    }

    // Load the Behavior Tree from the XML input
    try
    {
      ROS_INFO("[cmpf_behavior_tree] Loading behavior tree from %s", file_path.c_str());
      tree_ = factory_.createTreeFromFile(file_path, black_board_);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR("[cmpf_behavior_tree] Failed to load the behavior tree. Exception: %s", ex.what());
      return false;
    }
    bt_xml_file_path_ = file_path;
    return true;
  }

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // params
  std::string bt_xml_default_file_path_, bt_xml_file_path_;
  double bt_loop_frequency_;

  // main action server
  std::unique_ptr<ActionT> action_server_;

  // The BehaviorTreeFactory to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;

  // BT blackboard to share data by all the nodes of the tree
  BT::Blackboard::Ptr black_board_;

  // Behavior Tree to be executed when goal is received
  BT::Tree tree_;
};

}  // namespace behavior_tree
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::behavior_tree::BTActionServerNodelet, nodelet::Nodelet)