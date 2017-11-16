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

    try
    {
      robot_ = robot->getHandle(robot_state_name).getState();
    }
    catch(const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Could not find robot state: " << robot_state_name << " Not loading the controller. " <<
        e.what());
      return false;
    }
    
    ROS_INFO("Getting joint names");
    if (!node_.getParam("joints", joint_names_))
    {
      ROS_ERROR("No joints given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    //¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬
    std::string tmp_joint_names;
    if (!node_.getParam("joints", tmp_joint_names))
    {
      ROS_ERROR("No joints given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    
    for (int i = 0; i < joint_names_.size(); ++i)
    {
        ROS_INFO_STREAM(" >>>>>>>>>>>>>>>>> Joint states: " << tmp_joint_names);
    }
    //¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬
    
    // check for message type
    if (joint_names_.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    
    mins_.resize(joint_names_.size());
    maxs_.resize(joint_names_.size());
    vel_mins_.resize(joint_names_.size());
    vel_maxs_.resize(joint_names_.size());
    eff_mins_.resize(joint_names_.size());
    eff_maxs_.resize(joint_names_.size());

    ROS_INFO("Getting pid values");
    
    std::string gains_ns;
    if (!node_.getParam("gains", gains_ns))
    {
      gains_ns = node_.getNamespace() + "/gains";
    }
    
    pids_.resize(joint_names_.size());
    for (int i = 0; i < joint_names_.size(); ++i)
    {
      if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + static_cast<std::string>(joint_names_[i]).c_str())))
      {
        return false;
      }
    }
    
    /*
    pid_controller_position_.reset(new control_toolbox::Pid());
    if (!pid_controller_position_->init(ros::NodeHandle(node_, "pid")))
    {
      return false;
    }
    */

    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                                              (node_, "state", 1));

/*    double p, i, d, i_max, i_min;
    for (int k = 0; k < joint_names_.size(); ++k)
    {
        ROS_INFO_STREAM("Joint " << k << "name: " << joint_names_[k]);
        pids_[k].getGains(p, i, d, i_max, i_min);
        ROS_INFO_STREAM("P: " << p << " I: " << i << " D: " << d);
        
    }
*/

    joints_.resize(joint_names_.size());
    std::string joint_name;
    for (int i = 0; i < joint_names_.size(); ++i)
    {
        // joint 0s e.g. FFJ0
        joint_name = static_cast<std::string>(joint_names_[i]).c_str();
        has_j2 = is_joint_0(joint_name);
        if (has_j2)
        {
            get_joints_states_1_2(joint_name, joints_[i]);
            if (!joints_[i][0])
            {
                ROS_ERROR("SrhGraspController could not find the first joint relevant to \"%s\"\n",
                        joint_name);
                return false;
            }
            if (!joints_[i][1])
            {
                ROS_ERROR("SrhGraspController could not find the second joint relevant to \"%s\"\n",
                        joint_name);
                return false;
            }
            if (!joints_[i][1]->calibrated_)
            {
                ROS_ERROR("Joint %s not calibrated for SrhGraspController", joint_name);
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
            joints_[i][0] = robot_->getJointState(joint_name);
            if (!joints_[i][0])
            {
                ROS_ERROR("SrhGraspController could not find joint named \"%s\"\n", joint_name);
                return false;
            }
            if (!joints_[i][0]->calibrated_)
            {
                ROS_ERROR("Joint %s not calibrated for SrhGraspnController", joint_name);
                return false;
            }
        }
    }
    
 
    // *********** DEBUG CODE ****************************
    for (int i = 0; i < joint_names_.size(); ++i)
    {
        for (int k = 0; k < joints_[i].size(); ++k)
        ROS_INFO_STREAM("Joint " << static_cast<std::string>(joint_names_[i]).c_str() << " position: " << joints_[i][k]->position_);
    }
    //**************************************************

    // get the min and max value for the current joint:
    //for (int i = 0; i < joint_names_.size(); ++i)
   // {
   //     get_min_max(robot_->robot_model_, static_cast<std::string>(joint_names_[i]).c_str());
   // }

    friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

    //serve_set_gains_ = node_.advertiseService("set_gains", &SrhGraspController::setGains, this);
    //serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhGraspController::resetGains, this);

    after_init();
    return true;
  }

  void SrhGraspController::starting(const ros::Time &time)
  {
    resetJointState();
    pid_controller_position_->reset();
    read_parameters();

    if (has_j2)
      ROS_WARN_STREAM(
              "Reseting PID for joints " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
    else
      ROS_WARN_STREAM("Reseting PID for joint  " << joint_state_->joint_->name);
  }

/*  bool SrhGraspController::setGains(sr_robot_msgs::SetPidGains::Request &req,
                                            sr_robot_msgs::SetPidGains::Response &resp)
  {
    ROS_INFO_STREAM("Setting new PID parameters. P:" << req.p << " / I:" << req.i <<
                    " / D:" << req.d << " / IClamp:" << req.i_clamp << ", max force: " <<
                    req.max_force << ", friction deadband: " << req.friction_deadband <<
                    " pos deadband: " << req.deadband);

    pid_controller_position_->setGains(req.p, req.i, req.d, req.i_clamp, -req.i_clamp);
    max_force_demand = req.max_force;
    friction_deadband = req.friction_deadband;
    position_deadband = req.deadband;

    // Setting the new parameters in the parameter server
    node_.setParam("pid/p", req.p);
    node_.setParam("pid/i", req.i);
    node_.setParam("pid/d", req.d);
    node_.setParam("pid/i_clamp", req.i_clamp);
    node_.setParam("pid/max_force", max_force_demand);
    node_.setParam("pid/position_deadband", position_deadband);
    node_.setParam("pid/friction_deadband", friction_deadband);

    return true;
  }
*/
  bool SrhGraspController::resetGains(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    resetJointState();

    if (!pid_controller_position_->init(ros::NodeHandle(node_, "pid")))
    {
      return false;
    }

    read_parameters();

    if (has_j2)
      ROS_WARN_STREAM(
              "Reseting controller gains: " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
    else
      ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name);

    return true;
  }


  void SrhGraspController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_position_->getGains(p, i, d, i_max, i_min);
  }

  void SrhGraspController::update(const ros::Time &time, const ros::Duration &period)
  {

    if (!has_j2 && !joint_state_->calibrated_)
    {
      return;
    }

    ROS_ASSERT(robot_);
    ROS_ASSERT(joint_state_->joint_);

    if (!initialized_)
    {
      resetJointState();
      initialized_ = true;
    }
    if (has_j2)
    {
      command_ = joint_state_->commanded_position_ + joint_state_2->commanded_position_;
    }
    else
    {
      command_ = joint_state_->commanded_position_;
    }
    command_ = clamp_command(command_);
    
    // Compute position demand from position error:
    double error_position = 0.0;
    double commanded_effort = 0.0;

    if (has_j2)
    {
      error_position = (joint_state_->position_ + joint_state_2->position_) - command_;
    }
    else
    {
      error_position = joint_state_->position_ - command_;
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
  }

  void SrhGraspController::read_parameters()
  {
    node_.param<double>("pid/max_force", max_force_demand, 1023.0);
    node_.param<double>("pid/position_deadband", position_deadband, 0.015);
    node_.param<int>("pid/friction_deadband", friction_deadband, 5);
  }

  void SrhGraspController::setCommandCB(const std_msgs::Float64ConstPtr &msg)
  {
    joint_state_->commanded_position_ = msg->data;
    if (has_j2)
    {
      joint_state_2->commanded_position_ = 0.0;
    }
  }

  void SrhGraspController::resetJointState()
  {
    if (has_j2)
    {
      joint_state_->commanded_position_ = joint_state_->position_;
      joint_state_2->commanded_position_ = joint_state_2->position_;
      command_ = joint_state_->position_ + joint_state_2->position_;
    }
    else
    {
      joint_state_->commanded_position_ = joint_state_->position_;
      command_ = joint_state_->position_;
    }
  }
  
  bool SrhGraspController::is_joint_0(const std::string & joint_name)
  {
    // joint_name_ has unknown length
    // it is assumed that last char is the joint number
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
  
/*  void SrhGraspController::get_min_max(urdf::Model model, std::string joint_name)
  {
    int pos = find(joint_names_.begin(), joint_names_.end(), joint_name) - joint_names_.begin();  
    
    if (joint_name_[joint_name.size() - 1] == '0')
    {
      joint_name[joint_name.size() - 1] = '1';
      std::string j1 = joint_name;
      joint_name[joint_name.size() - 1] = '2';
      std::string j2 = joint_name;

      boost::shared_ptr<const urdf::Joint> joint1 = model.getJoint(j1);
      boost::shared_ptr<const urdf::Joint> joint2 = model.getJoint(j2);

      mins_[pos] = joint1->limits->lower + joint2->limits->lower;
      maxs_[pos] = joint1->limits->upper + joint2->limits->upper;
      vel_maxs_[pos] = joint1->limits->velocity + joint2->limits->velocity;
      vel_mins_[pos] = -1 * vel_maxs_[pos];
      eff_maxs_[pos] = joint1->limits->effort + joint2->limits->effort;
      eff_mins_[pos] = -1 * eff_maxs_[pos];
    }
    else
    {
      boost::shared_ptr<const urdf::Joint> joint = model.getJoint(joint_name);

      mins_[pos] = joint->limits->lower;
      maxs_[pos] = joint->limits->upper;
      vel_maxs_[pos] = joint->limits->velocity;
      vel_mins_[pos] = -1 * vel_maxs_[pos];
      eff_maxs_[pos] = joint->limits->effort;
      eff_mins_[pos] = -1 * eff_maxs_[pos];
    }
  }
*/
  
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */


