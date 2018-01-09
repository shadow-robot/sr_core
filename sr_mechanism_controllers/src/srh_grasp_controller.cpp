/**
 * @file   srh_joint_position_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
 *
 * Copyright 2011 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @brief  Follows a position target. The position demand is converted into a
 * force
 * demand by a PID loop.
 *
 */

#include "sr_mechanism_controllers/srh_grasp_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include "sr_utilities/sr_math_utils.hpp"
#include <algorithm>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhGraspController,
                       controller_interface::ControllerBase)

using std::min;
using std::max;

namespace controller
{

SrhGraspController::SrhGraspController()
{
}

bool SrhGraspController::init(ros_ethercat_model::RobotStateInterface *robot,
                              ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  std::string robot_state_name;
  std::string gains_ns;
  node_.param<std::string>("robot_state_name", robot_state_name,
                           "unique_robot_hw");
  node_ = n;
  using XmlRpc::XmlRpcValue;
  XmlRpc::XmlRpcValue joint_names_xml;
  commanded_state_ = PRE_GRASP;
  new_command_ = true;

  try
  {
    robot_ = robot->getHandle(robot_state_name).getState();
  }
  catch (const hardware_interface::HardwareInterfaceException &e)
  {
    ROS_ERROR_STREAM("Could not find robot state: "
                     << robot_state_name << " Not loading the controller. "
                     << e.what());
    return false;
  }

  if (!node_.getParam("joints", joint_names_xml))
  {
    ROS_ERROR("No joints given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  if (joint_names_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  for (int i = 0; i < joint_names_xml.size(); ++i)
  {
    joint_names_.push_back(
        static_cast<std::string>(joint_names_xml[i]).c_str());
  }

  pids_.resize(joint_names_.size());
  max_force_demands_.resize(joint_names_.size());
  position_deadbands_.resize(joint_names_.size());
  friction_deadbands_.resize(joint_names_.size());
  lookup_pos_.resize(joint_names_.size());
  lookup_torq_.resize(joint_names_.size());
  mode_.resize(joint_names_.size());
  torque_direction_.resize(joint_names_.size());
  joints_.resize(joint_names_.size());
  position_command_.resize(joint_names_.size());

  if (!node_.getParam("gains", gains_ns))
  {
    gains_ns = node_.getNamespace() + "/gains";
  }

  for (int i = 0; i < joint_names_.size(); ++i)
  {
    if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joint_names_[i])))
    {
      return false;
    }
    node_.param<double>(gains_ns + "/" + joint_names_[i] + "/max_force",
                        max_force_demands_[i], 1023.0);
    node_.param<double>(gains_ns + "/" + joint_names_[i] + "/position_deadband",
                        position_deadbands_[i], 0.015);
    node_.param<int>(gains_ns + "/" + joint_names_[i] + "/friction_deadband",
                     friction_deadbands_[i], 5);

    has_j2 = is_joint_0(joint_names_[i]);
    if (has_j2)
    {
      get_joints_states_1_2(joint_names_[i], joints_[i]);
      if (!joints_[i][0])
      {
        ROS_ERROR("SrhGraspController could not find the first joint relevant "
                  "to \"%s\"\n",
                  joint_names_[i]);
        return false;
      }
      if (!joints_[i][1])
      {
        ROS_ERROR("SrhGraspController could not find the second joint relevant "
                  "to \"%s\"\n",
                  joint_names_[i]);
        return false;
      }
      if (!joints_[i][1]->calibrated_)
      {
        ROS_ERROR("Joint %s not calibrated for SrhGraspController",
                  joint_names_[i]);
        return false;
      }
      else
      {
        joints_[i][0]->calibrated_ = true;
      }
    }
    else
    {
      joints_[i].push_back(NULL);
      joints_[i][0] = robot_->getJointState(joint_names_[i]);
      if (!joints_[i][0])
      {
        ROS_ERROR("SrhGraspController could not find joint named \"%s\"\n",
                  joint_names_[i]);
        return false;
      }
      if (!joints_[i][0]->calibrated_)
      {
        ROS_ERROR("Joint %s not calibrated for SrhGraspnController",
                  joint_names_[i]);
        return false;
      }
    }
    friction_compensator.reset(
        new sr_friction_compensation::SrFrictionCompensator(joint_names_[i]));
  }

  get_min_max(robot_->robot_model_, joint_names_);
  grasp_command_ = node_.subscribe("grasp_cmd", 1,
                                   &SrhGraspController::grasp_command_CB, this);

  return true;
}

void SrhGraspController::starting(const ros::Time &time)
{
  resetJointState();
  for (int i = 0; i < pids_.size(); ++i)
  {
    pids_[i].reset();
  }
}

void SrhGraspController::update(const ros::Time &time,
                                const ros::Duration &period)
{
  double error_position;
  double commanded_effort;
  bool in_deadband;
  float current_torque;

  if (!initialized_)
  {
    resetJointState();
    initialized_ = true;
  }

  switch (commanded_state_)
  {
  case PRE_GRASP:
    if (new_command_ == true)
    {
      for (size_t i = 0; i < joints_.size(); ++i)
      {
        mode_[i] = POSITION;
      }
      new_command_ = false;
    }
    break;
  case GRASP:
    if (new_command_ == true)
    {
      control_stage_ = PHASE_ONE;
      phase1_start_time_ = ros::Time::now();

      for (size_t i = 0; i < joints_.size(); ++i)
      {
        mode_[i] = POSITION;
      }
      new_command_ = false;
    }

    if ((time - phase1_start_time_) > (ros::Duration)0.5)
    {
      control_stage_ = PHASE_TWO;
    }

    if (PHASE_TWO == control_stage_)
    {
      for (size_t i = 0; i < joints_.size(); ++i)
      {
        if (torque_direction_[i] != 0)
        {
          mode_[i] = TORQUE;
        }
        else
        {
          mode_[i] = POSITION;
        }
      }
      current_torque = max_torque_;
    }
    break;
  }

  for (int i = 0; i < joints_.size(); ++i)
  {
    error_position = 0.0;
    commanded_effort = 0.0;
    has_j2_ = (2 == joints_[i].size());

    if (!has_j2_ && !joints_[i][0]->calibrated_)
    {
      return;
    }

    if (mode_[i] == POSITION)
    {
      if (has_j2_)
      {
        position_command_[i] = joints_[i][0]->commanded_position_ +
                               joints_[i][1]->commanded_position_;
        position_command_[i] =
            clamp_command(position_command_[i], mins_[i], maxs_[i]);
        error_position = (joints_[i][0]->position_ + joints_[i][1]->position_) -
                         position_command_[i];
      }
      else
      {
        position_command_[i] = joints_[i][0]->commanded_position_;
        position_command_[i] =
            clamp_command(position_command_[i], mins_[i], maxs_[i]);
        error_position = joints_[i][0]->position_ - position_command_[i];
      }

      in_deadband = hysteresis_deadband.is_in_deadband(
          position_command_[i], error_position, position_deadbands_[i]);

      if (in_deadband)
      {
        error_position = 0.0;
      }

      commanded_effort = pids_[i].computeCommand(-error_position, period);

      commanded_effort =
          min(commanded_effort, (max_force_demands_[i] * max_force_factor_));
      commanded_effort =
          max(commanded_effort, -(max_force_demands_[i] * max_force_factor_));

      if (!in_deadband)
      {
        if (has_j2)
        {
          commanded_effort += friction_compensator->friction_compensation(
              joints_[i][0]->position_ + joints_[i][1]->position_,
              joints_[i][0]->velocity_ + joints_[i][1]->velocity_,
              static_cast<int>(commanded_effort), friction_deadbands_[i]);
        }
        else
        {
          commanded_effort += friction_compensator->friction_compensation(
              joints_[i][0]->position_, joints_[i][0]->velocity_,
              static_cast<int>(commanded_effort), friction_deadbands_[i]);
        }
      }
      joints_[i][0]->commanded_effort_ = commanded_effort;
    }
    else if (mode_[i] == TORQUE)
    {
      joints_[i][0]->commanded_effort_ = current_torque * torque_direction_[i];
    }
  }
}

void SrhGraspController::grasp_command_CB(
    const sr_manipulation_msgs::GraspCommand::ConstPtr &command)
{
  Grasp_State commanded_state;
  sr_manipulation_msgs::Grasp grasp_info;
  sr_manipulation_msgs::SqueezeDirection squeeze_direction;
  moveit_msgs::Grasp moveit_grasp;

  commanded_state = (Grasp_State)command->grasp_state;
  max_torque_ = command->max_torque;
  grasp_info = command->grasp_info;
  squeeze_direction = grasp_info.squeeze_direction;
  moveit_grasp = grasp_info.grasp;
  hand_id_ = grasp_info.hand_id;
  lookup_pos_ = std::vector<int>(lookup_pos_.size(), -1);
  lookup_torq_ = std::vector<int>(lookup_torq_.size(), -1);

  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (commanded_state == PRE_GRASP)
    {
      for (size_t k = 0; k < moveit_grasp.pre_grasp_posture.joint_names.size();
           ++k)
      {
        if (moveit_grasp.pre_grasp_posture.joint_names[k] == joint_names_[i])
        {
          lookup_pos_[i] = k;
          break;
        }
      }

      joints_[i][0]->commanded_position_ = moveit_grasp.pre_grasp_posture.points.back()
              .positions[lookup_pos_[i]];
      if (2 == joints_[i].size())
      {
        joints_[i][1]->commanded_position_ = 0.0;
      }
    }
    else
    {
      for (size_t k = 0; k < moveit_grasp.grasp_posture.joint_names.size(); ++k)
      {
        if (moveit_grasp.grasp_posture.joint_names[k] == joint_names_[i])
        {
          lookup_pos_[i] = k;
          break;
        }
      }
      for (size_t j = 0; j < squeeze_direction.joint_names.size(); ++j)
      {
        if (squeeze_direction.joint_names[j] == joint_names_[i])
        {
          lookup_torq_[i] = j;
          break;
        }
      }

      joints_[i][0]->commanded_position_ = moveit_grasp.grasp_posture.points.back().positions[lookup_pos_[i]];
      if (2 == joints_[i].size())
      {
        joints_[i][1]->commanded_position_ = 0.0;
      }
    }

    if (lookup_pos_[i] == -1)
    {
      ROS_ERROR_STREAM("Unable to locate joint " << joint_names_[i]
                                                 << " for position.");
      return;
    }
    if ((lookup_torq_[i] == -1) && (commanded_state == GRASP))
    {
      ROS_ERROR_STREAM("Unable to locate joint " << joint_names_[i]
                                                 << " for torque.");
      return;
    }

    torque_direction_[i] = squeeze_direction.squeeze_direction[lookup_torq_[i]];
  }
  commanded_state_ = commanded_state;
  new_command_ = true;
}

void SrhGraspController::resetJointState()
{
  for (int i = 0; i < joints_.size(); ++i)
  {
    if (2 == joints_[i].size())
    {
      joints_[i][0]->commanded_position_ = joints_[i][0]->position_;
      joints_[i][1]->commanded_position_ = joints_[i][1]->position_;
      position_command_[i] = joints_[i][0]->position_ + joints_[i][1]->position_;
    }
    else
    {
      joints_[i][0]->commanded_position_ = joints_[i][0]->position_;
      position_command_[i] = joints_[i][0]->position_;
    }
  }
}

bool SrhGraspController::is_joint_0(const std::string &joint_name)
{
  if (joint_name[joint_name.size() - 1] == '0')
  {
    return true;
  }
  return false;
}

void SrhGraspController::get_joints_states_1_2(
    const std::string &joint_name,
    std::vector<ros_ethercat_model::JointState *> &joint)
{
  std::string j1 = joint_name, j2 = joint_name;
  j1[j1.size() - 1] = '1';
  j2[j2.size() - 1] = '2';

  ROS_DEBUG_STREAM("Joint 0: " << j1 << " " << j2);

  joint.push_back(NULL);
  joint[0] = robot_->getJointState(j1);
  joint.push_back(NULL);
  joint[1] = robot_->getJointState(j2);
}

void SrhGraspController::get_min_max(
    urdf::Model model, const std::vector<std::string> &joint_names)
{
  std::string joint_name;
  for (int i = 0; i < joint_names.size(); ++i)
  {
    joint_name = joint_names[i];
    if (joint_name[joint_name.size() - 1] == '0')
    {
      joint_name[joint_name.size() - 1] = '1';
      std::string j1 = joint_name;
      joint_name[joint_name.size() - 1] = '2';
      std::string j2 = joint_name;

      boost::shared_ptr<const urdf::Joint> joint1 = model.getJoint(j1);
      boost::shared_ptr<const urdf::Joint> joint2 = model.getJoint(j2);

      mins_.push_back(joint1->limits->lower + joint2->limits->lower);
      maxs_.push_back(joint1->limits->upper + joint2->limits->upper);
      vel_maxs_.push_back(joint1->limits->velocity + joint2->limits->velocity);
      vel_mins_.push_back(-1 * vel_maxs_[i]);
      eff_maxs_.push_back(joint1->limits->effort + joint2->limits->effort);
      eff_mins_.push_back(-1 * eff_maxs_[i]);
    }
    else
    {
      boost::shared_ptr<const urdf::Joint> joint = model.getJoint(joint_name);

      mins_.push_back(joint->limits->lower);
      maxs_.push_back(joint->limits->upper);
      vel_maxs_.push_back(joint->limits->velocity);
      vel_mins_.push_back(-1 * vel_maxs_[i]);
      eff_maxs_.push_back(joint->limits->effort);
      eff_mins_.push_back(-1 * eff_maxs_[i]);
    }
  }
}
}  // namespace controller
