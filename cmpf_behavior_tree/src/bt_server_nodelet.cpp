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

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <memory>

namespace cmpf {
namespace behavior_tree {

class BTServerNodelet : public nodelet::Nodelet {
 public:
  BTServerNodelet() {}
  virtual ~BTServerNodelet() {}

  void onInit() override {
    NODELET_DEBUG(
        "Initializing "
        "cmpf::behavior_tree::BTServerNodelet");

    // initialize node handlers
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // ros parameters
    // get default bt xml file or retrieve from parameter server
    std::string pkg_path = ros::package::getPath("cmpf_behavior_tree");
    bt_xml_file_path_ = pkg_path + "/behavior_trees/default_bt.xml";
    private_nh_.param("bt_xml_file_path", bt_xml_file_path_, bt_xml_file_path_);

    // get all the BT plugins
    const std::vector<std::string> bt_plugin_libs = {
        "cmpf_follow_trajectory_action_client_bt_node"};

    // register the BT plugin nodes in BehaviorTreeFactory
    BT::SharedLibrary loader;
    for (const auto& p : bt_plugin_libs) {
      factory_.registerFromPlugin(loader.getOSName(p));
    }

    // create blackboard to share across all the tree nodes
    black_board_ = BT::Blackboard::create();
    black_board_->set<std::string>("ns", nh_.getNamespace());

    // Create the Behavior Tree from the XML input
    try {
      ROS_INFO("Loading behavior tree from %s", bt_xml_file_path_.c_str());
      auto tree = factory_.createTreeFromFile(bt_xml_file_path_, black_board_);
      tree.tickRoot();
    } catch (const std::exception& ex) {
      ROS_FATAL("Failed to load the behavior tree. Exception: %s", ex.what());
      exit(1);
    }
  }

 private:
  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // params
  std::string bt_xml_file_path_;

  // The BehaviorTreeFactory to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;

  // BT blackboard to share data by all the nodes of the tree
  BT::Blackboard::Ptr black_board_;
};

}  // namespace behavior_tree
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cmpf::behavior_tree::BTServerNodelet, nodelet::Nodelet)