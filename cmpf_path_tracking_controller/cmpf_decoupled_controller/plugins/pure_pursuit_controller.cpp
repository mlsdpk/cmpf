#include "cmpf_decoupled_controller/plugins/pure_pursuit_controller.hpp"

namespace cmpf {
namespace path_tracking_controller {
namespace decoupled_controller {

PurePursuit::PurePursuit() {}

PurePursuit::~PurePursuit() {}

void PurePursuit::initialize(const std::string& name, ros::NodeHandle nh,
                             tf2_ros::Buffer* tf) {
  //
}

void PurePursuit::computeLateralControl(
    carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) {
  //
}

void PurePursuit::end() {}

}  // namespace decoupled_controller
}  // namespace path_tracking_controller
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
// register this controller as a LateralController plugin
PLUGINLIB_EXPORT_CLASS(
    cmpf::path_tracking_controller::decoupled_controller::PurePursuit,
    cmpf::path_tracking_controller::decoupled_controller::LateralController)