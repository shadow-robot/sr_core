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

#include <vector>
#include <string>
#include "sr_tactile_sensors/sr_mst_arrow_publisher.hpp"

/**
 * Constructor of the class using Member Initializer List (MIL) for the MarkerArray publisher
 * @param node_handle: Pointer to a ros NodeHandle instance
 * @param frame_prefix: Any prefix that should represent the sensor being observed (e.g. mst, or mstXL)
 * @param input_topic_name: Name of the topic where the MST data is published
 * @param number_zero_readings: Number of zero readings to be used to compute the dead-band
 * @param linear_scaling_value: Value to be used to scale the MST data
 * @param scaling_method: Method to be used to scale the MST data
 * @param marker_scale_x: scales the arrow markers shaft diameter
 * @param marker_scale_y: scales the arrow markers head width
 * @param marker_scale_z: scales the arrow markers head length
 */
MstArrowPublisher::MstArrowPublisher(ros::NodeHandle *node_handle, std::string frame_prefix,
                                     std::string input_topic_name, int number_zero_readings,
                                     double linear_scaling_value, std::string scaling_method,
                                     float marker_scale_x, float marker_scale_y, float marker_scale_z) :
  node_handle_(node_handle),
  input_topic_name_(input_topic_name),
  number_zero_readings_(number_zero_readings),
  linear_scaling_value_(linear_scaling_value),
  scaling_method_(scaling_method)
{
  // Initialise the marker publisher
  arrow_publisher_ = node_handle_->advertise<visualization_msgs::MarkerArray>(frame_prefix + "_arrows", 1);

  // Initialise the markers
  initialiseMarkers(frame_prefix, marker_scale_x, marker_scale_y, marker_scale_z);
  // Subscribe to the MST data topic to determine the zero dead-band
  mst_data_subscriber_ = node_handle_->subscribe(input_topic_name_, 1, &MstArrowPublisher::getZeroDeadband, this);

  // If the subscriber is not valid (i.e. the topic does not exist) stop here
  if (!mst_data_subscriber_)
  {
    ROS_ERROR_STREAM("There seems to be an issue while trying to access the topic " << input_topic_name_);
    exit(1);
  }
  // Make sure we compute the dead-band based on the amount of data provided as input
  while (current_number_zero_reading_ < number_zero_readings_)
  {
    ros::spinOnce();
  }
  // Shutdown the subscriber
  mst_data_subscriber_.shutdown();
  ROS_INFO("Sensor has been zeroed, you can start poking it!");
}

/**
 * Initialise all the Marker msgs part of the MarkerArray that is going to be published
 *  @param frame_prefix: Any prefix that should represent the sensor being observed (e.g. rh_ff, ff_mstXL, etc)
 */
void MstArrowPublisher::initialiseMarkers(std::string frame_prefix, float marker_scale_x, float marker_scale_y,
                                          float marker_scale_z)
{
  visualization_msgs::Marker marker;
  // By default, initialise all the fields to 0 (and that's what we want)
  geometry_msgs::Point zero_point;
  for (uint8_t taxel_index = 0; taxel_index < NUMBER_TAXELS; taxel_index++)
  {
    marker.id = taxel_index;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.header.frame_id = frame_prefix + "_taxel_" + std::to_string(taxel_index);
    // scale.x determines the shaft diameter
    marker.scale.x = marker_scale_x;
    // scale.y determines the head width
    marker.scale.y = marker_scale_y;
    // scale.z is the head length
    marker.scale.z = marker_scale_z;
    // Make the marker transparent when starting as we are not supposed to touch the sensor
    marker.color.a = 0;
    marker.pose.orientation.w = 1;
    // Create starting and ending point at the surface of the sensor
    marker.points = std::vector<geometry_msgs::Point>{ zero_point, zero_point };
    taxel_marker_array_.markers.push_back(marker);
  }
}

/**
 * Determine the minimal and maximal value of each taxel when the sensor is at rest (i.e. zero dead-band).
 * This method is meant to be a callback
 * @param sensor_reading: Constant pointer to a SensorReading msg, which should be provided by the subscriber
 */
void MstArrowPublisher::getZeroDeadband(const sr_robot_msgs::MST::ConstPtr& sensor_reading)
{
  // If the status is OK or if the information is not provided by the sensor, update the min and max values
  if (sensor_reading->status < 1)
  {
    for (uint8_t taxel_index = 0; taxel_index < NUMBER_TAXELS; taxel_index++)
    {
      taxel_msg_ = sensor_reading->magnetic_data[taxel_index];
      // Update the min and max value of the zero deadband
      if (taxel_msg_.x < min_zero_deadband_[3 * taxel_index])
      {
        min_zero_deadband_[3 * taxel_index] = taxel_msg_.x;
      }
      if (taxel_msg_.y < min_zero_deadband_[3 * taxel_index + 1])
      {
        min_zero_deadband_[3 * taxel_index + 1] = taxel_msg_.y;
      }
      if (taxel_msg_.z < min_zero_deadband_[3 * taxel_index + 2])
      {
        min_zero_deadband_[3 * taxel_index + 2] = taxel_msg_.z;
      }
      if (taxel_msg_.x > max_zero_deadband_[3 * taxel_index])
      {
        max_zero_deadband_[3 * taxel_index] = taxel_msg_.x;
      }
      if (taxel_msg_.y > max_zero_deadband_[3 * taxel_index + 1])
      {
        max_zero_deadband_[3 * taxel_index + 1] = taxel_msg_.y;
      }
      if (taxel_msg_.z > max_zero_deadband_[3 * taxel_index + 2])
      {
        max_zero_deadband_[3 * taxel_index + 2] = taxel_msg_.z;
      }
    }
    // Increment the number of data used to determine the zero dead-band
    current_number_zero_reading_++;
  }
}

/**
 * Subscribe to the MST data topic and for each message received republish a MarkerArray of arrows
 */
void MstArrowPublisher::run()
{
  mst_data_subscriber_ = node_handle_->subscribe(input_topic_name_, 1, &MstArrowPublisher::updateMarkers, this);
}

/**
 * Transform the raw magnetic data sent by the driver into arrows representing the vector of the change in magnetic data
 * This method is meant to be a callback
 * @param sensor_reading: Constant pointer to a SensorReading msg, which should be provided by the subscriber
 */
void MstArrowPublisher::updateMarkers(const sr_robot_msgs::MST::ConstPtr& sensor_reading)
{
  // For each taxel, retrieve the corresponding data
  for (uint8_t taxel_index = 0; taxel_index < NUMBER_TAXELS; taxel_index++)
  {
    taxel_msg_ = sensor_reading->magnetic_data[taxel_index];
    /*
      Create a zero vector and populate the value of the vector with respect to the zero dead-band .
      Note that the different operations are carried out on each axis. This is to ensures that the arrows somehow relate
      to the direction of the force applied to the sensor.
    */
    std::vector<float> vector_data { 0, 0, 0 };
    if (taxel_msg_.x < min_zero_deadband_[3 * taxel_index])
    {
      vector_data[0] = taxel_msg_.x - min_zero_deadband_[3 * taxel_index];
    }
    // The else ifs in all the following are very important as we want to have 0 if the value is in the zero dead-band
    else if (taxel_msg_.x > max_zero_deadband_[3 * taxel_index])
    {
      vector_data[0] = taxel_msg_.x - max_zero_deadband_[3 * taxel_index];
    }

    if (taxel_msg_.y < min_zero_deadband_[3 * taxel_index + 1])
    {
      vector_data[1] = min_zero_deadband_[3 * taxel_index + 1] - taxel_msg_.y;
    }
    else if (taxel_msg_.y > max_zero_deadband_[3 * taxel_index + 1])
    {
      vector_data[1] = max_zero_deadband_[3 * taxel_index + 1] - taxel_msg_.y;
    }

    if (taxel_msg_.z < min_zero_deadband_[3 * taxel_index + 2])
    {
      vector_data[2] = min_zero_deadband_[3 * taxel_index + 2] - taxel_msg_.z;
    }
    else if (taxel_msg_.z > max_zero_deadband_[3 * taxel_index + 2])
    {
      vector_data[2] = taxel_msg_.z - max_zero_deadband_[3 * taxel_index + 2];
    }

    // If the vector is in the zero dead-band, make sure the corresponding marker becomes transparent (i.e. not visible)
    if (std::all_of(vector_data.begin(), vector_data.end(), [](int sum_vector) { return sum_vector == 0; }))
    {
      taxel_marker_array_.markers[taxel_index].color.a = 0;
    }
    // Otherwise display the arrow corresponding to the resulting vector
    else
    {
      int norm_vector = std::sqrt(vector_data[0]*vector_data[0] + vector_data[1]*vector_data[1] +\
                                  vector_data[2]*vector_data[2]);
      // Normalise the vector
      for (uint8_t index=0; index < vector_data.size(); index++)
      {
        vector_data[index] /=norm_vector;
      }

      // Compute and apply the scaler factor
      magnitude_scaler_ = (scaling_method_ == "logarithmic") ? std::log(norm_vector) :
                          linear_scaling_value_* norm_vector;
      taxel_msg_.x = magnitude_scaler_ * vector_data[0];
      taxel_msg_.y = magnitude_scaler_ * vector_data[1];
      taxel_msg_.z = magnitude_scaler_ * vector_data[2];
      // Set the second point of the arrow (i.e. the tip) the computed value
      taxel_marker_array_.markers[taxel_index].points.back() = taxel_msg_;
      // Colourise the arrow with respect to the direction to where the vector points (additional information)
      // However, if the status of reported in the message is > 0, colour the arrow white and print a warning message
      if (sensor_reading->status < 1)
      {
        taxel_marker_array_.markers[taxel_index].color.r = vector_data[0];
        taxel_marker_array_.markers[taxel_index].color.g = vector_data[1];
        taxel_marker_array_.markers[taxel_index].color.b = vector_data[2];
      }
      else
      {
        taxel_marker_array_.markers[taxel_index].color.r = 1;
        taxel_marker_array_.markers[taxel_index].color.g = 1;
        taxel_marker_array_.markers[taxel_index].color.b = 1;
      }
      // Make sure the arrow is visible
      taxel_marker_array_.markers[taxel_index].color.a = 1;
    }
  }
  arrow_publisher_.publish(taxel_marker_array_);
}

/**
 * Main function called when this file is executed
 * @param argc: Integer corresponding to the number of arguments passed to the file
 * @param argv: Array of strings corresponding to the arguments passed to the script
 * @return: Integer 0 when the program finishes executing correctly. Otherwise, it returns 1.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_mst_arrow_publisher");
  ros::NodeHandle node_handle("~");

    // Get the different parameters that configure the publisher and make sure they are all good
    std::string sensor_index, frame_prefix, scaling_method, input_topic_name;
    int number_zero_readings;
    double linear_scaling_value;
    node_handle.getParam("number_zero_readings", number_zero_readings);
    node_handle.getParam("scaling", scaling_method);
    node_handle.getParam("id", sensor_index);
    node_handle.getParam("frame_prefix", frame_prefix);

    if (number_zero_readings <= 0)
    {
        ROS_ERROR("At least one sensor reading is required to determine the zero dead-band!");
        exit(1);
    }

    if (scaling_method.empty() || (scaling_method != "logarithmic" && scaling_method != "linear"))
    {
        ROS_ERROR_STREAM("Scaling method must be 'logarithmic' or 'linear' but the input value is " << scaling_method);
        exit(1);
    }

    if (scaling_method == "linear")
    {
      node_handle.getParam("scaling_value", linear_scaling_value);
      if (linear_scaling_value <= 0)
      {
        ROS_ERROR("The scaling factor applied to the magnitude of the vectors must be > 0!");
        exit(1);
      }
    }

    if (sensor_index.empty())
    {
      ROS_ERROR("A non-empty sensor index should be provided!");
      exit(1);
    }

    if (frame_prefix.empty())
    {
      ROS_ERROR("A non-empty frame prefix should be provided!");
      exit(1);
    }

    // Set the input topic name
    input_topic_name = "/sr_mst_driver_" + sensor_index + "/sr_mst_data";

    MstArrowPublisher mst_arrow_publisher(&node_handle, frame_prefix, input_topic_name, number_zero_readings,
                                          linear_scaling_value, scaling_method, 0.5, 0.8, 1);

    mst_arrow_publisher.run();
    ros::spin();
    return 0;
}
