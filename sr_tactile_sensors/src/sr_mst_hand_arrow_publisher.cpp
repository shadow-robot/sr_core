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
#include <memory>
#include "sr_tactile_sensors/sr_mst_hand_arrow_publisher.hpp"
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
MstHandArrowPublisher::MstHandArrowPublisher(ros::NodeHandle *node_handle, std::string frame_prefix,
                                             std::string input_topic_name, int number_zero_readings,
                                             double linear_scaling_value, std::string scaling_method,
                                             float marker_scale_x, float marker_scale_y, float marker_scale_z) :
                             MstArrowPublisher::MstArrowPublisher(node_handle, frame_prefix, input_topic_name,
                                                                  number_zero_readings, linear_scaling_value,
                                                                  scaling_method, marker_scale_x,
                                                                  marker_scale_y, marker_scale_z)
{
}

/**
 * (Override) Transform the raw magnetic data sent by the driver into arrows representing the vector of the change
 * in magnetic data. This method is meant to be a callback.
 * Scaling and colors are adusted to better match data visualisation on the hand
 * @param sensor_reading: Constant pointer to a SensorReading msg, which should be provided by the subscriber
 */
void MstHandArrowPublisher::updateMarkers(const sr_robot_msgs::MST::ConstPtr& sensor_reading)
{
  // For each taxel, retrieve the corresponding data
  for (uint8_t taxel_index = 0; taxel_index < NUMBER_TAXELS; taxel_index++)
  {
    taxel_msg_ = sensor_reading->magnetic_data[taxel_index];
    /*
      Create a zero vector and populate the vector values with respect to the zero dead-band .
      Note that the different operations are carried out on each channel. This is to ensure that the arrows somehow
      relate to the direction of the force applied to the sensor.
    */
    std::vector<float> vector_data {0, 0, 0};
    if (taxel_msg_.x < min_zero_deadband_[3*taxel_index])
    {
      vector_data[0] = taxel_msg_.x - min_zero_deadband_[3*taxel_index];
    }
    // The else ifs in all the following are very important as we want to have 0 if the value is in the zero dead-band
    else if (taxel_msg_.x > max_zero_deadband_[3*taxel_index])
    {
      vector_data[0] = taxel_msg_.x - max_zero_deadband_[3*taxel_index];
    }

    if (taxel_msg_.y < min_zero_deadband_[3*taxel_index+1])
    {
      vector_data[1] = min_zero_deadband_[3*taxel_index+1] - taxel_msg_.y;
    }
    else if (taxel_msg_.y > max_zero_deadband_[3*taxel_index+1])
    {
      vector_data[1] = max_zero_deadband_[3*taxel_index+1] - taxel_msg_.y;
    }

    if (taxel_msg_.z < min_zero_deadband_[3*taxel_index+2])
    {
      vector_data[2] = min_zero_deadband_[3*taxel_index+2] - taxel_msg_.z;
    }
    else if (taxel_msg_.z > max_zero_deadband_[3*taxel_index+2])
    {
      vector_data[2] = taxel_msg_.z - max_zero_deadband_[3*taxel_index+2];
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
      for (uint8_t index=0; index < vector_data.size(); index++) vector_data[index] /=norm_vector;

      // Compute and apply the scaler factor
      magnitude_scaler_ = (scaling_method_ == "logarithmic") ? 0.003 * std::log(norm_vector) :
                          linear_scaling_value_* norm_vector;
      taxel_msg_.x = magnitude_scaler_ * vector_data[0];
      taxel_msg_.y = magnitude_scaler_ * vector_data[1];
      taxel_msg_.z = magnitude_scaler_ * vector_data[2];
      // Set the second point of the arrow (i.e. the tip) the computed value
      taxel_marker_array_.markers[taxel_index].points.back() = taxel_msg_;
      // Colourise the arrow white
      taxel_marker_array_.markers[taxel_index].color.r = 1;
      taxel_marker_array_.markers[taxel_index].color.g = 1;
      taxel_marker_array_.markers[taxel_index].color.b = 1;
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
    ros::init(argc, argv, "sr_mst_hand_arrow_publisher");
    ros::NodeHandle node_handle("~");

    // Get the different parameters that configure the publisher and make sure they are all good
    std::string sensor_index, frame_prefix, hand_prefix, scaling_method, input_topic_name;
    int number_zero_readings, rviz_markers_publishing_frequency;
    double linear_scaling_value;
    node_handle.getParam("number_zero_readings", number_zero_readings);
    node_handle.getParam("scaling", scaling_method);
    node_handle.getParam("hand_prefix", hand_prefix);
    node_handle.getParam("rviz_markers_publishing_frequency", rviz_markers_publishing_frequency);

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

    if (hand_prefix.empty() || (hand_prefix != "rh" && hand_prefix != "lh"))
    {
        ROS_ERROR_STREAM("Hand prefix must set as 'rh' (right hand) or 'lh' (left hand), but the input value was "
                         << hand_prefix);
        exit(1);
    }

    if (rviz_markers_publishing_frequency <= 0)
    {
        ROS_ERROR("(rviz_markers_publishing_frequency) Pleased specify a publishing rate for the arrows above 0 Hz!");
        exit(1);
    }

    // Create a marker publisher for each finger/sensor
    std::vector<std::shared_ptr<MstHandArrowPublisher>> mst_hand_arrow_publisher;

    for (uint8_t sensor_index = 0; sensor_index < finger_index_str.size(); sensor_index++)
    {
      // Set the unique input topic name (and prefix) for each sensor
      input_topic_name = "/" + hand_prefix + "/sr_mst_driver_" + finger_index_str[sensor_index] + "/sr_mst_data";
      frame_prefix = hand_prefix + "_" + finger_index_str[sensor_index];
      // Initialise each publisher. Scaling values have been choosen empirically
      std::shared_ptr<MstHandArrowPublisher> finger_arrow_publisher (new MstHandArrowPublisher(&node_handle,
                                                                                               frame_prefix,
                                                                                               input_topic_name,
                                                                                               number_zero_readings,
                                                                                               linear_scaling_value,
                                                                                               scaling_method, 0.0015,
                                                                                               0.0024, 0.003));
      mst_hand_arrow_publisher.push_back(finger_arrow_publisher);

      mst_hand_arrow_publisher[sensor_index]->run();
    }

    // Define a ROS Rate to allow RViz to have enough time to handle the incoming data
    ros::Rate rate(rviz_markers_publishing_frequency);
    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
