#pragma once

#include "cmpf_decoupled_controller/decoupled_controller.hpp"

namespace cmpf {
namespace path_tracking_controller {
namespace decoupled_controller {

class PurePursuit : public LateralController {
 public:
  PurePursuit();

  ~PurePursuit();

  virtual void initialize(const std::string& name, ros::NodeHandle nh,
                          tf2_ros::Buffer* tf) override;

  virtual void computeLateralControl(
      carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) override;

  virtual void end() override;
};

}  // namespace decoupled_controller
}  // namespace path_tracking_controller
}  // namespace cmpf