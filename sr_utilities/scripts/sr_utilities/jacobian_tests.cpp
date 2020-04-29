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
  ros::NodeHandle nh;

  ros::NodeHandle nh_priv("~");
  double des_force_x;
  double des_force_y;
  double des_force_z;
  double des_force_roll, des_force_pitch, des_force_yaw;

  shadow_robot::SrJacobianUtils ju("robot_description", "rh_first_finger", "rh_fftip");

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

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ju.desired_wrench_.wrench.force.x = des_force_x;
  ju.desired_wrench_.wrench.force.y = des_force_y;
  ju.desired_wrench_.wrench.force.z = des_force_z;
  ju.desired_wrench_.wrench.torque.x = des_force_roll;
  ju.desired_wrench_.wrench.torque.y = des_force_pitch;
  ju.desired_wrench_.wrench.torque.z = des_force_yaw;
  ju.desired_wrench_.header.frame_id = "rh_fftip";

  ju.transform_desired_wrench_to_base_frame();

  ROS_WARN_STREAM(ju.desired_wrench_in_base_frame_.wrench.force.x);
  ROS_WARN_STREAM(ju.desired_wrench_in_base_frame_.wrench.force.y);
  ROS_WARN_STREAM(ju.desired_wrench_in_base_frame_.wrench.force.z);
  ROS_WARN_STREAM(ju.desired_wrench_in_base_frame_.wrench.torque.x);
  ROS_WARN_STREAM(ju.desired_wrench_in_base_frame_.wrench.torque.y);
  ROS_WARN_STREAM(ju.desired_wrench_in_base_frame_.wrench.torque.z);

  Eigen::VectorXd desired_force_from_palm(6);
  desired_force_from_palm(0) = ju.desired_wrench_in_base_frame_.wrench.force.x;
  desired_force_from_palm(1) = ju.desired_wrench_in_base_frame_.wrench.force.y;
  desired_force_from_palm(2) = ju.desired_wrench_in_base_frame_.wrench.force.z;
  desired_force_from_palm(3) = ju.desired_wrench_in_base_frame_.wrench.torque.x;
  desired_force_from_palm(4) = ju.desired_wrench_in_base_frame_.wrench.torque.y;
  desired_force_from_palm(5) = ju.desired_wrench_in_base_frame_.wrench.torque.z;

    boost::shared_ptr<sensor_msgs::JointState const> joint_states_ptr;
    joint_states_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    ju.kinematic_state_->setVariableValues(*joint_states_ptr);
    Eigen::MatrixXd jacobian = ju.kinematic_state_->getJacobian(ju.joint_model_group_);
    Eigen::VectorXd needed_torques = jacobian.transpose() * desired_force_from_palm;
    ROS_INFO_STREAM(std::endl << jacobian.transpose());
    ROS_INFO_STREAM(std::endl << needed_torques);
}
