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
 * @brief  Follows a position target. The position demand is converted into a force
 * demand by a PID loop.
 *
 */

#include "sr_mechanism_controllers/srh_grasp_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <string>
#include <sstream>
#include <algorithm>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhGraspController, controller_interface::ControllerBase)

using std::min;
using std::max;

namespace controller
{

  SrhGraspController::SrhGraspController()
          : position_deadband(0.015)
  {
  }

  bool SrhGraspController::init(ros_ethercat_model::RobotStateInterface *robot, ros::NodeHandle &n)
  {
    ROS_ASSERT(robot);
    std::string robot_state_name;
    node_.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");
    node_ = n;
    using XmlRpc::XmlRpcValue;
    XmlRpc::XmlRpcValue joint_names_xml;
    std::vector<std::string> joint_names;
    
    try
    {
      robot_ = robot->getHandle(robot_state_name).getState();
    }
    catch(const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Could not find robot state: " << robot_state_name << " Not loading the controller. " << e.what());
      return false;
    }

    if (!node_.getParam("joints", joint_names_xml))
    {
      ROS_ERROR("No joints given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    if (joint_names_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    for (int i = 0; i < joint_names_xml.size(); ++i)
    {
        joint_names.push_back(static_cast<std::string>(joint_names_xml[i]).c_str());
    }

    std::string gains_ns;
    if (!node_.getParam("gains", gains_ns))
    {
      gains_ns = node_.getNamespace() + "/gains";
    }
    
    pids_.resize(joint_names.size());
    max_force_demands_.resize(joint_names.size());
    position_deadbands_.resize(joint_names.size());
    friction_deadbands_.resize(joint_names.size());
    for (int i = 0; i < joint_names.size(); ++i)
    {
      if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joint_names[i])))
      {
        return false;
      }
      node_.param<double>(gains_ns + "/" + joint_names[i] + "/max_force", max_force_demands_[i], 1023.0);
      node_.param<double>(gains_ns + "/" + joint_names[i] + "/position_deadband", position_deadbands_[i], 0.015);
      node_.param<int>(gains_ns + "/" + joint_names[i] + "/friction_deadband", friction_deadbands_[i], 5);
    }

    //controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node_, "state", 1));

    //************** debug code ************************
    double p, i, d, i_max, i_min;
    for (int k = 0; k < joint_names.size(); ++k)
    {
        ROS_INFO_STREAM("Joint " << k << " name: " << joint_names[k]);
        pids_[k].getGains(p, i, d, i_max, i_min);
        ROS_INFO_STREAM("P: " << p << " I: " << i << " D: " << d << std::endl);
        ROS_INFO_STREAM("mfd: " << max_force_demands_[k] << std::endl);
        ROS_INFO_STREAM("pd: " << position_deadbands_[k] << std::endl);
        ROS_INFO_STREAM("fd: " << friction_deadbands_[k] << std::endl);
        
    }
    //**************************************************

    joints_.resize(joint_names.size());
    position_command_.resize(joint_names.size());
    for (int i = 0; i < joint_names.size(); ++i)
    {
        // joint 0s e.g. FFJ0
        has_j2 = is_joint_0(joint_names[i]);
        if (has_j2)
        {
            get_joints_states_1_2(joint_names[i], joints_[i]);
            if (!joints_[i][0])
            {
                ROS_ERROR("SrhGraspController could not find the first joint relevant to \"%s\"\n",
                        joint_names[i]);
                return false;
            }
            if (!joints_[i][1])
            {
                ROS_ERROR("SrhGraspController could not find the second joint relevant to \"%s\"\n",
                        joint_names[i]);
                return false;
            }
            if (!joints_[i][1]->calibrated_)
            {
                ROS_ERROR("Joint %s not calibrated for SrhGraspController", joint_names[i]);
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
            joints_[i][0] = robot_->getJointState(joint_names[i]);
            if (!joints_[i][0])
            {
                ROS_ERROR("SrhGraspController could not find joint named \"%s\"\n", joint_names[i]);
                return false;
            }
            if (!joints_[i][0]->calibrated_)
            {
                ROS_ERROR("Joint %s not calibrated for SrhGraspnController", joint_names[i]);
                return false;
            }
        }
    }
    
 
    // *********** DEBUG CODE ****************************
    for (int i = 0; i < joint_names.size(); ++i)
    {
        for (int k = 0; k < joints_[i].size(); ++k)
        ROS_INFO_STREAM("Joint " << joint_names[i] << " position: " << joints_[i][k]->position_);
    }
    //**************************************************

    get_min_max(robot_->robot_model_, joint_names);

    //**************** DEBUG CODE *****************************
    for (int i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO_STREAM("Limits: " << mins_[i] << " " << maxs_[i] << " " << vel_mins_[i] << " " << vel_maxs_[i] << " " << eff_mins_[i] << " " << eff_maxs_[i]);
    }
    //****************************************************************
    for (int i = 0; i < joint_names.size(); ++i)
    {
        friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_names[i]));
    }

    sub_command_ = node_.subscribe<std_msgs::Float64MultiArray>("command", 1, &SrhGraspController::setCommandCB, this);

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

  void SrhGraspController::update(const ros::Time &time, const ros::Duration &period)
  {
    if (!initialized_)
    {
      resetJointState();
      initialized_ = true;
    }
    
    for (int i = 0; i < joints_.size(); ++i)
    {
    
      if (2 == joints_[i].size())
      {
          has_j2_=true;
      }
        
      if (!has_j2_ && !joints_[i][0]->calibrated_)
      {
        return;
      }
    }
    
    if (has_j2_)
    {
      command_ = joints_[i][0]->commanded_position_ + joints_[i][1]->commanded_position_;
    }
    else
    {
      command_ = joints_[i][0]->commanded_position_;
    }
    command_ = clamp_command(command_);  // CHECK!!!!!!
    
    // Compute position demand from position error:
    double error_position = 0.0;
    double commanded_effort = 0.0;

    if (has_j2)
    {
      error_position = (joints_[i][0]->position_ + joints_[i][1]->position_) - command_;
    }
    else
    {
      error_position = joints_[i][0]->position_ - command_;
    }

    bool in_deadband = hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband);

    // don't compute the error if we're in the deadband.
    if (in_deadband)
    {
      error_position = 0.0;
    }

    commanded_effort = pid_controller_position_->computeCommand(-error_position, period);

    // clamp the result to max force
    commanded_effort = min(commanded_effort, (max_force_demand * max_force_factor_));
    commanded_effort = max(commanded_effort, -(max_force_demand * max_force_factor_));

    if (!in_deadband)
    {
      if (has_j2)
      {
        commanded_effort += friction_compensator->friction_compensation(
                joint_state_->position_ + joint_state_2->position_,
                joint_state_->velocity_ + joint_state_2->velocity_,
                static_cast<int>(commanded_effort),
                friction_deadband);
      }
      else
      {
        commanded_effort += friction_compensator->friction_compensation(joint_state_->position_,
                                                                        joint_state_->velocity_,
                                                                        static_cast<int>(commanded_effort),
                                                                        friction_deadband);
      }
    }

    if ("rh_FFJ3" == joint_name_)
    {
        if (i < 10000)
        {
            commanded_effort = 250;
            i++;
        }
        else
        {
            commanded_effort = -250;
            i++;
            if (i > 20000)
            {
                i = 0;
            }
        }
        
        
    }
    joint_state_->commanded_effort_ = commanded_effort;
/*
    if (loop_count_ % 10 == 0)
    {
      if (controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;
        if (has_j2)
        {
          controller_state_publisher_->msg_.process_value = joint_state_->position_ + joint_state_2->position_;
          controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_ + joint_state_2->velocity_;
        }
        else
        {
          controller_state_publisher_->msg_.process_value = joint_state_->position_;
          controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
        }

        controller_state_publisher_->msg_.error = error_position;
        controller_state_publisher_->msg_.time_step = period.toSec();
        controller_state_publisher_->msg_.command = joint_state_->commanded_position_;

        double dummy;
        getGains(controller_state_publisher_->msg_.p,
                 controller_state_publisher_->msg_.i,
                 controller_state_publisher_->msg_.d,
                 controller_state_publisher_->msg_.i_clamp,
                 dummy);
        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;   
*/
  }

  void SrhGraspController::setCommandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
  {
    
    for (int i = 0; i < joints_.size(); ++i)
    {
        joints_[i][0]->commanded_position_ = msg->data[i];
        if (2 == joints_[i].size())
        {
        joints_[i][1]->commanded_position_ = 0.0;
        }
    }
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
  
  bool SrhGraspController::is_joint_0(const std::string & joint_name)
  {
    if (joint_name[joint_name.size() - 1] == '0')
    {
      return true;
    }
    return false;
  }
  
  void SrhGraspController::get_joints_states_1_2(const std::string & joint_name, std::vector<ros_ethercat_model::JointState*> & joint)
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
  
  void SrhGraspController::get_min_max(urdf::Model model, const std::vector<std::string> & joint_names)
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

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */


