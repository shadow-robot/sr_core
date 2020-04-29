#include "sr_utilities/jacobian_utils.hpp"

namespace shadow_robot
{

SrJacobianUtils::SrJacobianUtils(std::string robot_description_name,
                                 std::string model_group_name,
                                 std::string ee_frame_name) :
  robot_description_name(robot_description_name),
  model_group_name(model_group_name),
  ee_frame_name(ee_frame_name)
{
  setup_kinematic_state();
}

SrJacobianUtils::~SrJacobianUtils()
{
}

void SrJacobianUtils::setup_kinematic_state()
{
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader.reset(new robot_model_loader::RobotModelLoader(robot_description_name));
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader->getModel();
  joint_model_group_ = kinematic_model->getJointModelGroup(model_group_name);
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();
}

}  // namespace shadow_robot