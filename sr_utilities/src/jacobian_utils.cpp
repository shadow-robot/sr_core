#include "sr_utilities/jacobian_utils.hpp"

namespace shadow_robot
{

SrJacobianUtils::SrJacobianUtils(std::string robot_description_name,
                                 std::string model_group_name) :
  robot_description_name_(robot_description_name),
  model_group_name_(model_group_name)
{
  setup_kinematic_state();
}

SrJacobianUtils::~SrJacobianUtils()
{
}

void SrJacobianUtils::setup_kinematic_state()
{
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader.reset(new robot_model_loader::RobotModelLoader(robot_description_name_));
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader->getModel();
  joint_model_group_ = kinematic_model->getJointModelGroup(model_group_name_);
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();
}

Eigen::VectorXd SrJacobianUtils::get_torques_given_wrench(geometry_msgs::WrenchStamped wrench)
{
  Eigen::MatrixXd jacobian = get_jacobian();
  return get_torques_given_wrench_and_jacobian(wrench, jacobian);
}

Eigen::VectorXd SrJacobianUtils::get_torques_given_wrench(geometry_msgs::WrenchStamped wrench,
                                                          sensor_msgs::JointState joint_states)
{
  Eigen::MatrixXd jacobian = get_jacobian(joint_states);
  return get_torques_given_wrench_and_jacobian(wrench, jacobian);
}

Eigen::VectorXd SrJacobianUtils::get_torques_given_wrench_and_jacobian(geometry_msgs::WrenchStamped wrench,
                                                                       Eigen::MatrixXd jacobian)
{
  geometry_msgs::WrenchStamped wrench_in_base_frame = transform_wrench_to_base_frame(wrench);
  Eigen::VectorXd wrench_in_base_frame_vector = wrench_to_eigen_vector(wrench_in_base_frame);
  return jacobian.transpose() * wrench_in_base_frame_vector;
}

Eigen::MatrixXd SrJacobianUtils::get_jacobian(sensor_msgs::JointState joint_states)
{
  kinematic_state_->setVariableValues(joint_states);
  return kinematic_state_->getJacobian(joint_model_group_);
}

Eigen::MatrixXd SrJacobianUtils::get_jacobian()
{
  boost::shared_ptr<sensor_msgs::JointState const> joint_states_ptr;
  joint_states_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
  return get_jacobian(*joint_states_ptr);
}

geometry_msgs::WrenchStamped SrJacobianUtils::transform_wrench_to_base_frame(geometry_msgs::WrenchStamped wrench)
{
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener(tf2_buffer);
  tf2_buffer.lookupTransform("world", "rh_palm", ros::Time(0), ros::Duration(1.0));
  return tf2_buffer.transform(wrench, "rh_palm");
}

Eigen::VectorXd SrJacobianUtils::wrench_to_eigen_vector(geometry_msgs::WrenchStamped wrench)
{
  Eigen::VectorXd wrench_in_vector_form(6);
  wrench_in_vector_form(0) = wrench.wrench.force.x;
  wrench_in_vector_form(1) = wrench.wrench.force.y;
  wrench_in_vector_form(2) = wrench.wrench.force.z;
  wrench_in_vector_form(3) = wrench.wrench.torque.x;
  wrench_in_vector_form(4) = wrench.wrench.torque.y;
  wrench_in_vector_form(5) = wrench.wrench.torque.z;
  return wrench_in_vector_form;
}

}  // namespace shadow_robot