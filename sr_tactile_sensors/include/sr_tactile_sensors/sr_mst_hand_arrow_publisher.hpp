/*
* Copyright (C) 2023 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
**/


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
