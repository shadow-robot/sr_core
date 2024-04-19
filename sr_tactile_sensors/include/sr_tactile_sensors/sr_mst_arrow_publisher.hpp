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


#ifndef SR_MST_ARROW_PUBLISHER_H
#define SR_MST_ARROW_PUBLISHER_H

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

class MstArrowPublisher
{
public:
  explicit MstArrowPublisher(ros::NodeHandle *node_handle, std::string frame_prefix, std::string input_topic_name,
                             int number_zero_readings, double linear_scaling_value, std::string scaling_method,
                             float marker_scale_x, float marker_scale_y, float marker_scale_z);
  void run();

protected:
  // Method
  void initialiseMarkers(std::string frame_prefix, float marker_scale_x, float marker_scale_y,
                                 float marker_scale_z);
  // Callback method
  virtual void updateMarkers(const sr_robot_msgs::MST::ConstPtr& sensor_reading);

  // MST constants that might change in future designs
  const uint8_t NUMBER_TAXELS = 17;
  const uint8_t NUMBER_FIELD_PER_TAXEL = 3;

  // Constants set by the user
  int number_zero_readings_;
  double linear_scaling_value_;
  std::string scaling_method_;

  // Vectors that will contain the boundaries of the zero headband for each field of each taxel
  std::vector<int> min_zero_deadband_ = std::vector<int>(NUMBER_TAXELS * NUMBER_FIELD_PER_TAXEL, INT_MAX);
  std::vector<int> max_zero_deadband_ = std::vector<int>(NUMBER_TAXELS * NUMBER_FIELD_PER_TAXEL, INT_MIN);
  // Value by which a the magnitude of a vector is scaled
  double magnitude_scaler_;
  // Define a single Point msg that will be used to store the data provided by one taxel of the sensor
  geometry_msgs::Point taxel_msg_;
  // MarkerArray publisher
  ros::Publisher arrow_publisher_;
  // Array of markers that is going to be published
  visualization_msgs::MarkerArray taxel_marker_array_;

private:
  // Callback method
  void getZeroDeadband(const sr_robot_msgs::MST::ConstPtr& sensor_reading);

  // Name of the topic to subscribe to
  std::string input_topic_name_;
  // Counter of the number of messages read to determine the zero dead-band
  int current_number_zero_reading_ = 0;

  // ROS Node handle
  ros::NodeHandle *node_handle_;
  // MST msg data subscriber
  ros::Subscriber mst_data_subscriber_;
};

#endif  // SR_MST_ARROW_PUBLISHER_H
