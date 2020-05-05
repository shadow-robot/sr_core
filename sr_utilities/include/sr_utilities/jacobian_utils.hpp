/*
* Copyright 2020 Shadow Robot Company Ltd.
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

#ifndef SR_UTILITIES_JACOBIAN_UTILS_H
#define SR_UTILITIES_JACOBIAN_UTILS_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
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
