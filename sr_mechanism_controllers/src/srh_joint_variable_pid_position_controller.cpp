/*
* Copyright 2011 Shadow Robot Company Ltd.
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

/**
 * @file   srh_joint_variable_pid_position_controller.cpp
 * @author Anastasios <anastasios@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
 * @brief  Follows a position target. The position demand is converted into a force
 * demand by a PID loop.
 *
 * The code is base on srh_joint_position_controller.cpp
 *
 */

#include "sr_mechanism_controllers/srh_joint_variable_pid_position_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <string>
#include <sstream>
#include <algorithm>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhJointVariablePidPositionController, controller_interface::ControllerBase)

using std::min;
using std::max;

namespace controller
{

  SrhJointVariablePidPositionController::SrhJointVariablePidPositionController()
          : position_deadband_(0.015)
  {
  }

  bool SrhJointVariablePidPositionController::init(ros_ethercat_model::RobotStateInterface *robot, ros::NodeHandle &n)
  {
    ROS_ASSERT(robot);

    set_point_old_ = 0.0;
    error_old_ = 0.0;
    frequency_ = 1000.0;
    smoothing_velocity_min_ = 2.08;
    smoothing_velocity_max_ = 4.2;
    smoothing_factor_p_ = 0.8;
    smoothing_factor_i_ = 0.8;
    smoothing_factor_d_ = 0.8;

    std::string robot_state_name;
    node_.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");

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

    node_ = n;

    if (!node_.getParam("joint", joint_name_))
    {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    pid_controller_position_.reset(new PlainPid());
    if (!pid_controller_position_->init(ros::NodeHandle(node_, "pid")))
    {
      return false;
    }

    double dummy;
    pid_controller_position_->getGains(p_init_, i_init_, d_init_, i_clamp_, dummy);

    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                                              (node_, "state", 1));

    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name_);

    // joint 0s e.g. FFJ0
    has_j2 = is_joint_0();
    if (has_j2)
    {
      get_joints_states_1_2();
      if (!joint_state_)
      {
        ROS_ERROR("SrhJointVariablePidPositionController could not find the first joint relevant to \"%s\"\n",
                  joint_name_.c_str());
        return false;
      }
      if (!joint_state_2)
      {
        ROS_ERROR("SrhJointVariablePidPositionController could not find the second joint relevant to \"%s\"\n",
                  joint_name_.c_str());
        return false;
      }
    }
    else
    {
      joint_state_ = robot_->getJointState(joint_name_);
      if (!joint_state_)
      {
        ROS_ERROR("SrhJointVariablePidPositionController could not find joint named \"%s\"\n", joint_name_.c_str());
        return false;
      }
    }

    // get the min and max value for the current joint:
    get_min_max(robot_->robot_model_, joint_name_);

    friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhJointVariablePidPositionController::setGains, this);
    serve_reset_gains_ = node_.advertiseService("reset_gains",
                                                &SrhJointVariablePidPositionController::resetGains,
                                                this);

    after_init();
    return true;
  }

  void SrhJointVariablePidPositionController::starting(const ros::Time &time)
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

  bool SrhJointVariablePidPositionController::setGains(sr_robot_msgs::SetPidGains::Request &req,
                                            sr_robot_msgs::SetPidGains::Response &resp)
  {
    ROS_INFO_STREAM("Setting new PID parameters. P:" << req.p << " / I:" << req.i <<
                    " / D:" << req.d << " / IClamp:" << req.i_clamp << ", max force: " <<
                    req.max_force << ", friction deadband: " << req.friction_deadband <<
                    " pos deadband: " << req.deadband);

    p_init_ = req.p;
    i_init_ = req.i;
    d_init_ = req.d;
    i_clamp_ = req.i_clamp;

    max_force_demand = req.max_force;
    friction_deadband = req.friction_deadband;
    position_deadband_ = req.deadband;

    // Setting the new parameters in the parameter server
    node_.setParam("pid/p", req.p);
    node_.setParam("pid/i", req.i);
    node_.setParam("pid/d", req.d);
    node_.setParam("pid/i_clamp", req.i_clamp);
    node_.setParam("pid/max_force", max_force_demand);
    node_.setParam("pid/position_deadband", position_deadband_);
    node_.setParam("pid/friction_deadband", friction_deadband);

    return true;
  }

  bool SrhJointVariablePidPositionController::resetGains(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
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

  void SrhJointVariablePidPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_position_->getGains(p, i, d, i_max, i_min);
  }

  void SrhJointVariablePidPositionController::updatePid(double error, double set_point)
  {
    double i_initial = i_init_;
    double d_initial = d_init_;

    double diff_set_point = fabs(set_point_old_ - set_point);
    double set_point_velocity = diff_set_point * frequency_;
    double diff_error = fabs(error_old_ - error);
    double error_velocity = diff_error * frequency_;

    double exp_error = exp(fabs(error));
    double exp_log_error_velocity = exp(log10(error_velocity));

    double p = p_init_ * (exp_error + exp_log_error_velocity + set_point_velocity);
    double i = i_init_ * (exp_error + exp_log_error_velocity + set_point_velocity);
    double d = d_init_ * (exp_error + exp_log_error_velocity + set_point_velocity);

    if (set_point_velocity > smoothing_velocity_min_ && set_point_velocity < smoothing_velocity_max_)
    {
        p = smoothing_factor_p_ * p;
        i = smoothing_factor_i_ * i;
        d = smoothing_factor_d_ * d;
    }

    pid_controller_position_->setGains(p, i, d, i_clamp_, -i_clamp_, 1);

    set_point_old_ = set_point;
    error_old_ = error;
  }

  void SrhJointVariablePidPositionController::update(const ros::Time &time, const ros::Duration &period)
  {
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

    bool in_deadband = hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband_);

    // don't compute the error if we're in the deadband.
    if (in_deadband)
    {
      error_position = 0.0;
    }

    updatePid(error_position, command_);

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
        controller_state_publisher_->msg_.command = commanded_effort;

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

  void SrhJointVariablePidPositionController::read_parameters()
  {
    node_.param<double>("pid/max_force", max_force_demand, 1023.0);
    node_.param<double>("pid/position_deadband", position_deadband_, 0.015);
    node_.param<int>("pid/friction_deadband", friction_deadband, 5);
  }

  void SrhJointVariablePidPositionController::setCommandCB(const std_msgs::Float64ConstPtr &msg)
  {
    joint_state_->commanded_position_ = msg->data;
    if (has_j2)
    {
      joint_state_2->commanded_position_ = 0.0;
    }
  }

  void SrhJointVariablePidPositionController::resetJointState()
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
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */


