#include "sr_utilities/jacobian_utils.hpp"

namespace shadow_robot
{

SrJacobianUtils::SrJacobianUtils(std::string robot_description_name,
                                 std::string model_group_name,
                                 std::string ee_frame_name) :
  robot_description_name_(robot_description_name),
  model_group_name_(model_group_name),
  ee_frame_name_(ee_frame_name)
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

void SrJacobianUtils::transform_desired_wrench_to_base_frame()
{
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener(tf2_buffer);
  tf2_buffer.lookupTransform("world", "rh_palm", ros::Time(0), ros::Duration(1.0));

  desired_wrench_in_base_frame_ = tf2_buffer.transform(desired_wrench_, "rh_palm");
}

Eigen::VectorXd SrJacobianUtils::get_torques_given_wrench(geometry_msgs::WrenchStamped wrench)
{
  std::cout << "chuj";
}

Eigen::MatrixXd SrJacobianUtils::get_jacobian()
{
  boost::shared_ptr<sensor_msgs::JointState const> joint_states_ptr;
  joint_states_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
  kinematic_state_->setVariableValues(*joint_states_ptr);
  return kinematic_state_->getJacobian(joint_model_group_);
}

}  // namespace shadow_robot