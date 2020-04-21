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

void jointsCB(const sensor_msgs::JointStateConstPtr& msg)
{
  joints_states = *msg;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_tests");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber joints_sub = nh.subscribe("/joint_states", 1, jointsCB);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  ROS_INFO_STREAM("robot model loader");
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
  robot_state::RobotStatePtr kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");

    while (ros::ok())
  {
    kinematic_state->setVariableValues(joints_states);
    Eigen::MatrixXd jacobian = kinematic_state->getJacobian(joint_model_group);

    ROS_INFO_STREAM(std::endl << jacobian);
    // ROS_INFO_STREAM(joints_states);
    // ros::Duration(0.5).sleep();

  }

}
