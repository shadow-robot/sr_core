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
#include <tf/transform_listener.h>

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

  tf::TransformListener listener_;

  tf::Stamped<tf::Vector3> lin_vector;

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


  tf2_ros::Buffer tfBuffer;
  geometry_msgs::Vector3Stamped des_force, des_torque;
  des_force.vector.x = des_force_x;
  des_force.vector.y = des_force_y;
  des_force.vector.z = des_force_z;
  des_force.header.frame_id = "rh_fftip";
  des_torque.vector.x = des_force_roll;
  des_torque.vector.y = des_force_pitch;
  des_torque.vector.z = des_force_yaw;
  des_torque.header.frame_id = "rh_fftip";

  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped transform_tip_to_palm, tf_des_force, tf_desired_force_in_palm_orientation;
  transform_tip_to_palm = tfBuffer.lookupTransform("rh_fftip", "rh_palm", ros::Time(0), ros::Duration(1.0) );

  geometry_msgs::TransformStamped transform_tip_to_palm_rot_only = transform_tip_to_palm;
  transform_tip_to_palm_rot_only.transform.translation.x = 0;
  transform_tip_to_palm_rot_only.transform.translation.y = 0;
  transform_tip_to_palm_rot_only.transform.translation.z = 0;
  transform_tip_to_palm_rot_only.header.stamp = ros::Time(0);
  transform_tip_to_palm_rot_only.child_frame_id = "rh_fftip_prim";
  transform_tip_to_palm_rot_only.header.frame_id = "rh_fftip";

  tfBuffer.setTransform(transform_tip_to_palm_rot_only, "");

  des_force = tfBuffer.transform(des_force, "rh_fftip_prim");
  des_torque = tfBuffer.transform(des_torque, "rh_fftip_prim");

  ROS_WARN_STREAM(des_force.vector.x);
  ROS_WARN_STREAM(des_force.vector.y);
  ROS_WARN_STREAM(des_force.vector.z);
  ROS_WARN_STREAM(des_torque.vector.x);
  ROS_WARN_STREAM(des_torque.vector.y);
  ROS_WARN_STREAM(des_torque.vector.z);

  Eigen::VectorXd desired_force_from_palm(6);
  desired_force_from_palm(0) = des_force.vector.x;
  desired_force_from_palm(1) = des_force.vector.y;
  desired_force_from_palm(2) = des_force.vector.z;

  // THIS DOESNT WORK - rtransform it with tf insread of adding!!!!
  desired_force_from_palm(3) = des_torque.vector.x;
  desired_force_from_palm(4) = des_torque.vector.y;
  desired_force_from_palm(5) = des_torque.vector.z;

    boost::shared_ptr<sensor_msgs::JointState const> joint_states_ptr;
    joint_states_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    ju.kinematic_state_->setVariableValues(*joint_states_ptr);
    Eigen::MatrixXd jacobian = ju.kinematic_state_->getJacobian(ju.joint_model_group_);
    Eigen::VectorXd needed_torques = jacobian.transpose() * desired_force_from_palm;
    ROS_INFO_STREAM(std::endl << jacobian.transpose());
    ROS_INFO_STREAM(std::endl << needed_torques);
}
