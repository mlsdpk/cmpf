#pragma once

#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <string>

#include "cmpf_msgs/TrajectoryMsg.h"

namespace cmpf {
namespace cmpf_core {

/**
 * @class BaseController
 * @brief Abstract controller interface that acts as a virtual base class for
 * all path tracking controller plugins
 */
class BaseController {
 public:
  using Ptr = std::shared_ptr<cmpf::cmpf_core::BaseController>;

  /**
   * @brief Virtual destructor
   */
  virtual ~BaseController() {}

  /**
   * @brief Method to initialize the controller
   * @param name controller plugin name
   * @param nh private nodehandler
   * @param tf tf transform
   */
  virtual void initialize(const std::string& name, ros::NodeHandle nh,
                          tf2_ros::Buffer* tf) = 0;

  /**
   * @brief Set the trajectory to follow
   * @param trajectory The trajectory given by the local planner
   */
  virtual bool setTrajectory(const cmpf_msgs::TrajectoryMsg& trajectory) = 0;

  /**
   * @brief Method to compute vehicle control commands (assuming that current
   * trajectory is already given)
   * @param vehicle_control_cmd The best carla vehicle control commands to
   * follow the trajectory
   */
  virtual bool computeVehicleControlCommands(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) = 0;
};

}  // namespace cmpf_core
}  // namespace cmpf