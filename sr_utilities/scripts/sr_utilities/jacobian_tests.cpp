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

sensor_msgs::JointState joints_states;

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J)
{
  return J.transpose() * (J * J.transpose()).inverse();
}

void jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  joints_states = *msg;

}

double constrain_angle(double x){
    x = fmod(x + M_PI/2, M_PI);
    if (x < 0)
        x += M_PI;
    return x - M_PI/2;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_tests");
  ros::NodeHandle nh;

  ros::NodeHandle nh_priv("~");
  double des_force_x;
  double des_force_y;
  double des_force_z;
  double des_force_roll, des_force_pitch, des_force_yaw;
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

  // ros::Subscriber joints_sub = nh.subscribe("/joint_states", 1, jointsCB);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  ROS_INFO_STREAM("robot model loader");
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
  robot_state::RobotStatePtr kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("rh_first_finger");

  bool rot_scaled = false;
  double scaling_factor = 1;

  Eigen::VectorXd desired_force(6);

  if (des_force_roll > 1 || des_force_pitch > 1 || des_force_yaw > 1)
  {
    rot_scaled= true;
    scaling_factor = std::max(std::max(des_force_roll, des_force_pitch), des_force_yaw);
  }

    desired_force(3) = des_force_roll / scaling_factor;
    desired_force(4) = des_force_pitch / scaling_factor;
    desired_force(5) = des_force_yaw / scaling_factor;

  desired_force(0) = des_force_x;
  desired_force(1) = des_force_y;
  desired_force(2) = des_force_z;
  
Eigen::Quaternionf q;
q = Eigen::AngleAxisf(desired_force(3), Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(desired_force(4), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(desired_force(5), Eigen::Vector3f::UnitZ());

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped transform_tip_to_palm, tf_des_force, tf_desired_force_in_palm_orientation;
  transform_tip_to_palm = tfBuffer.lookupTransform("rh_fftip", "rh_palm", ros::Time(0), ros::Duration(1.0) );

  tf_des_force.transform.translation.x = des_force_x;
  tf_des_force.transform.translation.y = des_force_y;
  tf_des_force.transform.translation.z = des_force_z;
  tf_des_force.transform.rotation.x = q.coeffs()[0];
  tf_des_force.transform.rotation.y = q.coeffs()[1];
  tf_des_force.transform.rotation.z = q.coeffs()[2];
  tf_des_force.transform.rotation.w = q.coeffs()[3];
  tf_des_force.header.stamp = ros::Time(0);
  tf_des_force.child_frame_id = "rh_desired_force";
  tf_des_force.header.frame_id = "rh_fftip";

  geometry_msgs::TransformStamped transform_tip_to_palm_rot_only = transform_tip_to_palm;
  transform_tip_to_palm_rot_only.transform.translation.x = 0;
  transform_tip_to_palm_rot_only.transform.translation.y = 0;
  transform_tip_to_palm_rot_only.transform.translation.z = 0;
  transform_tip_to_palm_rot_only.header.stamp = ros::Time(0);
  transform_tip_to_palm_rot_only.child_frame_id = "rh_fftip_prim";
  transform_tip_to_palm_rot_only.header.frame_id = "rh_fftip";

  tfBuffer.setTransform(tf_des_force, "");
  tfBuffer.setTransform(transform_tip_to_palm_rot_only, "");

  tf_desired_force_in_palm_orientation  = tfBuffer.lookupTransform("rh_fftip_prim", "rh_desired_force", ros::Time(0));

  ROS_WARN_STREAM(tf_desired_force_in_palm_orientation.transform.translation.x);
  ROS_WARN_STREAM(tf_desired_force_in_palm_orientation.transform.translation.y);
  ROS_WARN_STREAM(tf_desired_force_in_palm_orientation.transform.translation.z);

  Eigen::Quaternionf q_res(tf_desired_force_in_palm_orientation.transform.rotation.w,
                                tf_desired_force_in_palm_orientation.transform.rotation.x,
                                tf_desired_force_in_palm_orientation.transform.rotation.y,
                                tf_desired_force_in_palm_orientation.transform.rotation.z);
  auto rrr = q_res.toRotationMatrix();
  tf::Matrix3x3 rot_mat;

  for (int i = 0; i<3; i++)
  {
    for(int j =0; j<3; j++)
    {
      rot_mat[i][j] = rrr(i,j);
    }
  }


  Eigen::Quaternionf q_res2(transform_tip_to_palm.transform.rotation.w,
                                transform_tip_to_palm.transform.rotation.x,
                                transform_tip_to_palm.transform.rotation.y,
                                transform_tip_to_palm.transform.rotation.z);
  auto ppp = q_res2.toRotationMatrix();
  tf::Matrix3x3 rot_mat2;

  for (int i = 0; i<3; i++)
  {
    for(int j =0; j<3; j++)
    {
      rot_mat2[i][j] = ppp(i,j);
    }
  }

  Eigen::Vector3d euler;
  Eigen::Vector3d tip_palm_rot;
  rot_mat.getRPY(euler[0], euler[1], euler[2]);
  rot_mat2.getRPY(tip_palm_rot[0], tip_palm_rot[1], tip_palm_rot[2]);

  Eigen::VectorXd desired_force_from_palm(6);
  desired_force_from_palm(0) = tf_desired_force_in_palm_orientation.transform.translation.x;
  desired_force_from_palm(1) = tf_desired_force_in_palm_orientation.transform.translation.y;
  desired_force_from_palm(2) = tf_desired_force_in_palm_orientation.transform.translation.z;

  // THIS DOESNT WORK - rtransform it with tf insread of adding!!!!
  desired_force_from_palm(3) = euler[0]  + tip_palm_rot[0];
  desired_force_from_palm(4) = euler[1]  + tip_palm_rot[1];
  desired_force_from_palm(5) = euler[2]  + tip_palm_rot[2];

  if (rot_scaled)
  {
    desired_force_from_palm(3) *= scaling_factor;
    desired_force_from_palm(4) *= scaling_factor;
    desired_force_from_palm(5) *= scaling_factor;
  }


  ROS_WARN_STREAM(desired_force_from_palm(3));
  ROS_WARN_STREAM(desired_force_from_palm(4));
  ROS_WARN_STREAM(desired_force_from_palm(5));

    boost::shared_ptr<sensor_msgs::JointState const> joint_states_ptr;
    joint_states_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    kinematic_state->setVariableValues(*joint_states_ptr);
    Eigen::MatrixXd jacobian = kinematic_state->getJacobian(joint_model_group);
    Eigen::VectorXd needed_torques = jacobian.transpose() * desired_force_from_palm;
    ROS_INFO_STREAM(std::endl << desired_force);
    ROS_INFO_STREAM(std::endl << jacobian.transpose());
    ROS_INFO_STREAM(std::endl << needed_torques);
}
