#pragma once

#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

#include <cmpf_core/base_controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <string>

namespace cmpf {
namespace path_tracking_controller {
namespace decoupled_controller {

class LongitudinalController {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~LongitudinalController() {}

  virtual void initialize(const std::string& name, tf2_ros::Buffer* tf) = 0;

  virtual void computeLongitudinalControl(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) = 0;

  virtual void end() = 0;
};

class LateralController {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~LateralController() {}

  virtual void initialize(const std::string& name, tf2_ros::Buffer* tf) = 0;

  virtual void computeLateralControl(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) = 0;

  virtual void end() = 0;
};

class DecoupledController : public cmpf_core::BaseController {
 public:
  DecoupledController();

  virtual ~DecoupledController();

  virtual void initialize(const std::string& name, ros::NodeHandle nh,
                          tf2_ros::Buffer* tf) override;

  virtual bool setPlan(
      const std::vector<geometry_msgs::PoseStamped>& plan) override;

  virtual bool computeVehicleControlCommands(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) override;

 protected:
  //   pluginlib::ClassLoader<LongitudinalController> lgc_loader_;
  //   pluginlib::ClassLoader<LateralController> latc_loader_;
  std::shared_ptr<LongitudinalController> lg_controller_;
  std::shared_ptr<LateralController> lat_controller_;
  std::string lg_controller_id_;
  std::string lat_controller_id_;
};

}  // namespace decoupled_controller
}  // namespace path_tracking_controller
}  // namespace cmpf