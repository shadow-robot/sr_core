/*
* Copyright 2022-2024 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef SR_MST_HAND_ARROW_PUBLISHER_H
#define SR_MST_HAND_ARROW_PUBLISHER_H

#include "ros/ros.h"
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sr_robot_msgs/MST.h>
#include <geometry_msgs/Point.h>
#include "sr_tactile_sensors/sr_mst_arrow_publisher.hpp"

const std::vector<std::string> finger_index_str {"ff", "mf", "rf", "lf", "th"};

class MstHandArrowPublisher: public MstArrowPublisher
{
public:
  explicit MstHandArrowPublisher(ros::NodeHandle *node_handle, std::string frame_prefix, std::string input_topic_name,
                             int number_zero_readings, double linear_scaling_value, std::string scaling_method,
                             float marker_scale_x, float marker_scale_y, float marker_scale_z);

private:
  virtual void updateMarkers(const sr_robot_msgs::MST::ConstPtr& sensor_reading);
};

#endif  // SR_MST_HAND_ARROW_PUBLISHER_H
