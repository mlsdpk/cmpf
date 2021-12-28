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

#include <ros/ros.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <sstream>
#include <string>

#include "cmpf_msgs/Lanelet2MapBin.h"

namespace cmpf
{
namespace common
{
namespace map_utils
{
// https://github.com/Autoware-AI/common/blob/master/lanelet2_extension/lib/message_conversion.cpp
inline void toMsg(const lanelet::LaneletMapPtr& map, cmpf_msgs::Lanelet2MapBin* msg)
{
  if (msg == nullptr)
  {
    ROS_ERROR("Error converting to lanelet2 msg.");
    return;
  }

  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

void fromMsg(const cmpf_msgs::Lanelet2MapBin& msg, lanelet::LaneletMapPtr map)
{
  if (!map)
  {
    ROS_ERROR("Error converting to lanelet2 map.");
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());

  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *map;
  lanelet::Id id_counter;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
}
}  // namespace map_utils
}  // namespace common
}  // namespace cmpf
