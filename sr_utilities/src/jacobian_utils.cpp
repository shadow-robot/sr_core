#include "sr_utilities/jacobian_utils.hpp"

namespace shadow_robot
{

SrJacobianUtils::SrJacobianUtils(std::string robot_description_name,
                                 std::string model_group_name) :
  robot_description_name_(robot_description_name),
  model_group_name_(model_group_name),
  kinematic_model_(get_kinematic_model()),
  kinematic_state_(set_kinematic_state())
{
  setup_variables();
}

SrJacobianUtils::~SrJacobianUtils()
{
}

void SrJacobianUtils::setup_variables()
{
  joint_model_group_ = kinematic_model_->getJointModelGroup(model_group_name_);
  model_group_base_link_name_ = joint_model_group_->getJointModels()[0]->getParentLinkModel()->getName();
}

robot_state::RobotStatePtr SrJacobianUtils::set_kinematic_state()
{
  robot_state::RobotStatePtr kinematic_state;
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model_);
  kinematic_state->setToDefaultValues();
  return kinematic_state;
}

robot_model::RobotModelPtr SrJacobianUtils::get_kinematic_model()
{
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader.reset(new robot_model_loader::RobotModelLoader(robot_description_name_));
  return robot_model_loader->getModel();
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

Eigen::MatrixXd SrJacobianUtils::get_jacobian()
{
  boost::shared_ptr<sensor_msgs::JointState const> joint_states_ptr;
  joint_states_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
  return get_jacobian(*joint_states_ptr);
}

Eigen::MatrixXd SrJacobianUtils::get_jacobian(sensor_msgs::JointState joint_states)
{
  kinematic_state_->setVariableValues(joint_states);
  return kinematic_state_->getJacobian(joint_model_group_);
}

geometry_msgs::WrenchStamped SrJacobianUtils::transform_wrench_to_base_frame(geometry_msgs::WrenchStamped wrench)
{
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener(tf2_buffer);
  tf2_buffer.lookupTransform("world", model_group_base_link_name_, ros::Time(0), ros::Duration(1.0));
  return tf2_buffer.transform(wrench, model_group_base_link_name_);
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