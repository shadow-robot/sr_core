/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <math.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <boost/shared_ptr.hpp>

#include "sr_utilities/jacobian_utils.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_tests");

  ros::NodeHandle nh_priv("~");
  double des_force_x;
  double des_force_y;
  double des_force_z;
  double des_force_roll, des_force_pitch, des_force_yaw;

  geometry_msgs::WrenchStamped desired_wrench_;
  geometry_msgs::WrenchStamped desired_wrench_in_base_frame_;

  shadow_robot::SrJacobianUtils ju("robot_description", "rh_first_finger");

  if(!nh_priv.getParam("des_force_x", des_force_x))
  {
    des_force_x = 0;
  }
  if(!nh_priv.getParam("des_force_y", des_force_y))
  {
    des_force_y = 0;
  }
  if(!nh_priv.getParam("des_force_z", des_force_z))
  {
    des_force_z = 0;
  }
  if(!nh_priv.getParam("des_force_roll", des_force_roll))
  {
    des_force_roll = 0;
  }
  if(!nh_priv.getParam("des_force_pitch", des_force_pitch))
  {
    des_force_pitch = 0;
  }
  if(!nh_priv.getParam("des_force_yaw", des_force_yaw))
  {
    des_force_yaw = 0;
  }
    ROS_INFO_STREAM(des_force_x);
    ROS_INFO_STREAM(des_force_y);
    ROS_INFO_STREAM(des_force_z);
    ROS_INFO_STREAM(des_force_roll);
    ROS_INFO_STREAM(des_force_pitch);
    ROS_INFO_STREAM(des_force_yaw);

  desired_wrench_.wrench.force.x = des_force_x;
  desired_wrench_.wrench.force.y = des_force_y;
  desired_wrench_.wrench.force.z = des_force_z;
  desired_wrench_.wrench.torque.x = des_force_roll;
  desired_wrench_.wrench.torque.y = des_force_pitch;
  desired_wrench_.wrench.torque.z = des_force_yaw;
  desired_wrench_.header.frame_id = "rh_fftip";

  Eigen::VectorXd needed_torques = ju.get_torques_given_wrench(desired_wrench_);
  ROS_INFO_STREAM(std::endl << needed_torques);
}
