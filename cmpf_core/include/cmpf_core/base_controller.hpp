#pragma once

#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <string>

namespace cmpf {
namespace cmpf_core {

class BaseController {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~BaseController() {}

  virtual void initialize(const std::string& name, ros::NodeHandle nh,
                          tf2_ros::Buffer* tf) = 0;

  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

  virtual bool computeVehicleControlCommands(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) = 0;
};

}  // namespace cmpf_core
}  // namespace cmpf