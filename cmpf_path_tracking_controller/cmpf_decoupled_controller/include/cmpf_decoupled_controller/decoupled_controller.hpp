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

#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

#include <pluginlib/class_loader.hpp>
#include <string>

#include "cmpf_core/base_controller.hpp"
#include "cmpf_msgs/TrajectoryMsg.h"

namespace cmpf {
namespace path_tracking_controller {
namespace decoupled_controller {

/**
 * @class LongitudinalController
 * @brief Abstract decoupled controller interface that acts as a virtual base
 * class for all longitudinal controller plugins
 */
class LongitudinalController {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~LongitudinalController() {}

  /**
   * @brief Method to initialize the controller
   * @param name controller plugin name
   * @param nh private nodehandler
   * @param tf tf transform
   */
  virtual void initialize(const std::string& name, ros::NodeHandle nh,
                          tf2_ros::Buffer* tf) = 0;

  /**
   * @brief Method to compute vehicle longitudinal control commands (assuming
   * that current trajectory is already given)
   * @param vehicle_control_cmd The best carla vehicle control commands to
   * follow the trajectory
   */
  virtual void computeLongitudinalControl(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) = 0;

  /**
   * @brief Method to cleanup the controller
   */
  virtual void end() = 0;
};

/**
 * @class LateralController
 * @brief Abstract decoupled controller interface that acts as a virtual base
 * class for all lateral controller plugins
 */
class LateralController {
 public:
  /**
   * @brief Virtual destructor
   */
  virtual ~LateralController() {}

  /**
   * @brief Method to initialize the controller
   * @param name controller plugin name
   * @param nh private nodehandler
   * @param tf tf transform
   */
  virtual void initialize(const std::string& name, ros::NodeHandle nh,
                          tf2_ros::Buffer* tf) = 0;

  /**
   * @brief Method to compute vehicle lateral control commands (assuming that
   * current trajectory is already given)
   * @param vehicle_control_cmd The best carla vehicle control commands to
   * follow the trajectory
   */
  virtual void computeLateralControl(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) = 0;

  /**
   * @brief Method to cleanup the controller
   */
  virtual void end() = 0;
};

/**
 * @class DecoupledController
 * @brief Path tracking controller plugin derived from base
 * cmpf_core::BaseController class.
 *
 * This controller plugin decouples the vehicle control into longitudinal and
 * lateral control and internally calls individual plugins for each controller.
 */
class DecoupledController : public cmpf_core::BaseController {
 public:
  /**
   * @brief Constructor
   */
  DecoupledController();

  /**
   * @brief Destructor
   */
  virtual ~DecoupledController();

  /**
   * @brief Method to initialize the controller
   * @param name controller plugin name
   * @param nh private nodehandler
   * @param tf tf transform
   */
  virtual void initialize(const std::string& name, ros::NodeHandle nh,
                          tf2_ros::Buffer* tf) override;

  /**
   * @brief Set the trajectory to follow
   * @param trajectory The trajectory given by the local planner
   */
  virtual bool setTrajectory(
      const cmpf_msgs::TrajectoryMsg& trajectory) override;

  /**
   * @brief Method to compute vehicle control commands (assuming that current
   * trajectory is already given)
   * @param vehicle_control_cmd The best carla vehicle control commands to
   * follow the trajectory
   */
  virtual bool computeVehicleControlCommands(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) override;

 protected:
  // pluginlib loaders for longitudianl and lateral control
  //   pluginlib::ClassLoader<LongitudinalController> lgc_loader_;
  pluginlib::ClassLoader<LateralController> latc_loader_;

  // controllers
  std::shared_ptr<LongitudinalController> lg_controller_;
  std::shared_ptr<LateralController> lat_controller_;
  std::string lg_controller_id_;
  std::string lat_controller_id_;
};

}  // namespace decoupled_controller
}  // namespace path_tracking_controller
}  // namespace cmpf