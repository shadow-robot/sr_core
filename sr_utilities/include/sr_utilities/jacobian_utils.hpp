#ifndef SR_UTILITIES_JACOBIAN_UTILS_H
#define SR_UTILITIES_JACOBIAN_UTILS_H

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
#include <string>

namespace shadow_robot
{
class SrJacobianUtils
{
public:
  SrJacobianUtils(std::string, std::string);
  ~SrJacobianUtils();
  Eigen::MatrixXd get_jacobian();
  Eigen::MatrixXd get_jacobian(sensor_msgs::JointState);
  Eigen::VectorXd get_torques_given_wrench(geometry_msgs::WrenchStamped);
  Eigen::VectorXd get_torques_given_wrench(geometry_msgs::WrenchStamped,
                                           sensor_msgs::JointState);
  std::string get_robot_description_name();
  std::string get_model_group_name();
  std::string get_model_group_base_link_name();

private:
  geometry_msgs::WrenchStamped transform_wrench_to_base_frame(geometry_msgs::WrenchStamped);
  Eigen::VectorXd wrench_to_eigen_vector(geometry_msgs::WrenchStamped);
  Eigen::VectorXd get_torques_given_wrench_and_jacobian(geometry_msgs::WrenchStamped,
                                                        Eigen::MatrixXd);

  std::string robot_description_name_;
  std::string model_group_name_;
  std::string model_group_base_link_name_;
  robot_state::RobotStatePtr kinematic_state_;
  robot_state::JointModelGroup* joint_model_group_;
  ros::NodeHandle nh_;

};
}  // namespace shadow_robot

#endif  // SR_UTILITIES_JACOBIAN_UTILS_H