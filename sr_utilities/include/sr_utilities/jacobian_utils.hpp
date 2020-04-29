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
  SrJacobianUtils(std::string, std::string, std::string);
  ~SrJacobianUtils();
  Eigen::VectorXd calculate_torques_given_force_vector(Eigen::VectorXd);

  const std::string robot_description_name;
  const std::string model_group_name;
  std::string ee_frame_name;

  // move to private
  robot_state::RobotStatePtr kinematic_state_;
  robot_state::JointModelGroup* joint_model_group_;

private:
  void setup_kinematic_state();
  Eigen::VectorXd transform_force_vector_to_base_frame(Eigen::VectorXd);


};
}  // namespace shadow_robot

#endif  // SR_UTILITIES_JACOBIAN_UTILS_H