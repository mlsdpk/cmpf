#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

#include "cmpf_msgs/NavigateToPoseAction.h"

typedef actionlib::SimpleActionClient<cmpf_msgs::NavigateToPoseAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const cmpf_msgs::NavigateToPoseResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& nav_goal, Client* ac)
{
  cmpf_msgs::NavigateToPoseGoal bt_goal;
  bt_goal.pose = *nav_goal;

  ac->sendGoal(bt_goal, &doneCb);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_action_server_client_node");
  ros::NodeHandle nh;

  // Create the action client
  Client ac(nh, "behavior_tree/bt_server", true);

  ROS_INFO("Waiting for bt action server to exist.");
  ac.waitForServer();
  ROS_INFO("BT Action server started.");

  ros::Subscriber rviz_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1, std::bind(goalCallback, std::placeholders::_1, &ac));

  ros::spin();
  return 0;
}