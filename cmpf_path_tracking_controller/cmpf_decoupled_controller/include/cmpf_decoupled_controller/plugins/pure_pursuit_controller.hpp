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

#include "cmpf_decoupled_controller/decoupled_controller.hpp"

namespace cmpf
{
namespace path_tracking_controller
{
namespace decoupled_controller
{
class PurePursuit : public LateralController
{
public:
  PurePursuit();

  ~PurePursuit();

  virtual void initialize(const std::string& name, ros::NodeHandle nh, tf2_ros::Buffer* tf) override;

  virtual void computeLateralControl(const geometry_msgs::PoseStamped& pose,
                                     carla_msgs::CarlaEgoVehicleControl& vehicle_control_cmd) override;

  virtual void end() override;
};

}  // namespace decoupled_controller
}  // namespace path_tracking_controller
}  // namespace cmpf