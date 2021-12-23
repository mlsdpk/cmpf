#include "cmpf_decoupled_controller/decoupled_controller.hpp"

namespace cmpf {
namespace path_tracking_controller {
namespace decoupled_controller {

DecoupledController::DecoupledController()
    : latc_loader_("cmpf_decoupled_controller",
                   "cmpf::path_tracking_controller::decoupled_controller::"
                   "LateralController") {}

DecoupledController::~DecoupledController() {}

void DecoupledController::initialize(const std::string& name,
                                     ros::NodeHandle nh, tf2_ros::Buffer* tf) {
  nh.param("/" + name + "/longitudinal_controller/plugin", lg_controller_id_,
           std::string("path_tracking_controller/decoupled_controller/"
                       "longitudinal_controller/PID"));
  nh.param("/" + name + "/lateral_controller_plugin", lat_controller_id_,
           std::string("cmpf_decoupled_controller/PurePursuit"));

  /*
  // load longitudinal controller
  try {
    lg_controller_ = lgc_loader_.createUniqueInstance(lg_controller_id_);
    ROS_INFO("Created path_tracking_controller %s", lg_controller_id_.c_str());
    lg_controller_->initialize(lgc_loader_.getName(lg_controller_id_), tf);
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_FATAL(
        "Failed to create the %s controller, are you sure it is properly "
        "registered and that the containing library is built? Exception: %s",
        lg_controller_id_.c_str(), ex.what());
    exit(1);
  }
  */

  // load lateral controller
  try {
    lat_controller_ = latc_loader_.createUniqueInstance(lat_controller_id_);
    ROS_INFO("Created lateral_controller %s", lat_controller_id_.c_str());
    lat_controller_->initialize(latc_loader_.getName(lat_controller_id_), nh,
                                tf);
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_FATAL(
        "Failed to create the %s lateral_controller, are you sure it is "
        "properly registered and that the containing library is built? "
        "Exception: %s",
        lat_controller_id_.c_str(), ex.what());
    exit(1);
  }
}

bool DecoupledController::setTrajectory(
    const cmpf_msgs::TrajectoryMsg& trajectory) {
  // /
}

bool DecoupledController::computeVehicleControlCommands(
    carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) {
  //
}

}  // namespace decoupled_controller
}  // namespace path_tracking_controller
}  // namespace cmpf

#include <pluginlib/class_list_macros.h>
// register this controller as a BaseController plugin
PLUGINLIB_EXPORT_CLASS(
    cmpf::path_tracking_controller::decoupled_controller::DecoupledController,
    cmpf::cmpf_core::BaseController)