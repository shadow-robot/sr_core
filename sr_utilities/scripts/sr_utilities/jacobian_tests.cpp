/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState joints_states;

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J)
{
  return J.transpose() * (J * J.transpose()).inverse();
}

void jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  joints_states = *msg;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_tests");
  ros::NodeHandle nh;

  ros::NodeHandle nh_priv("~");
  double des_force_x;
  double des_force_y;
  double des_force_z;
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
  // for(auto const& value: des_forces) 
  // {
    ROS_INFO_STREAM(des_force_x);
    ROS_INFO_STREAM(des_force_y);
    ROS_INFO_STREAM(des_force_z);
  // }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber joints_sub = nh.subscribe("/joint_states", 1, jointsCB);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  ROS_INFO_STREAM("robot model loader");
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
  robot_state::RobotStatePtr kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("rh_first_finger");

  Eigen::VectorXd desired_force(6);
  desired_force(0) = des_force_x;
  desired_force(1) = des_force_y;
  desired_force(2) = des_force_z;
  desired_force(3) = 0;
  desired_force(4) = 0;
  desired_force(5) = 0;

  //   while (ros::ok())
  // {

    // std::vector<double> joint_states;
    // joint_states.push_back(joints_states.position[0]);
    // joint_states.push_back(joints_states.position[1]);
    // joint_states.push_back(joints_states.position[2]);
    // joint_states.push_back(joints_states.position[3]);
    kinematic_state->setVariableValues(joints_states);
    Eigen::MatrixXd jacobian = kinematic_state->getJacobian(joint_model_group);
    Eigen::VectorXd needed_torques = jacobian.transpose() * desired_force;
    // ROS_INFO_STREAM(std::endl << jacobian.transpose());
    ROS_INFO_STREAM(std::endl << joints_states);
    ROS_INFO_STREAM(std::endl << desired_force);
    ROS_INFO_STREAM(std::endl << jacobian.transpose());
    ROS_INFO_STREAM(std::endl << needed_torques);
    // ros::Duration(0.5).sleep();

  // }

}
