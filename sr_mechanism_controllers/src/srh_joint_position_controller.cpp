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
 * @file   srh_joint_position_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
 *
 * @brief  Follows a position target. The position demand is converted into a force
 * demand by a PID loop.
 *
 */

#include "sr_mechanism_controllers/srh_joint_position_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <string>
#include <sstream>
#include <algorithm>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"
#include <dynamic_reconfigure/server.h>
#include <sr_mechanism_controllers/TendonsConfig.h>

#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhJointPositionController, controller_interface::ControllerBase)

using std::min;
using std::max;

namespace controller
{

  SrhJointPositionController::SrhJointPositionController()
          : position_deadband(0.015)
  {
  }

  void SrhJointPositionController::dynamic_reconfigure_cb(sr_mechanism_controllers::TendonsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: j0-j3 smol,lorge: %d %d %d %d", config.j0_smol_num, config.j0_lorg_num, config.j3_smol_num, config.j3_lorg_num);
    this->j0_smol_num = config.j0_smol_num;
    this->j0_lorg_num = config.j0_lorg_num;
    this->j3_smol_num = config.j3_smol_num;
    this->j3_lorg_num = config.j3_lorg_num;
    this->bypass = config.bypass;
    this->correct = config.correct;
  }

  bool SrhJointPositionController::init(ros_ethercat_model::RobotStateInterface *robot, ros::NodeHandle &n)
  {
    ROS_ASSERT(robot);

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

    pid_controller_position_.reset(new control_toolbox::Pid());
    if (!pid_controller_position_->init(ros::NodeHandle(node_, "pid")))
    {
      return false;
    }

    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                                              (node_, "state", 1));

    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name_);

    dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<sr_mechanism_controllers::TendonsConfig>
                                              (node_));
    function_cb_ = boost::bind(&SrhJointPositionController::dynamic_reconfigure_cb, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(function_cb_);

   // double tau = 0.05;

   // pos_filter = sr_math_utils::filters::LowPassFilter(tau);

    // joint 0s e.g. FFJ0
    has_j2 = is_joint_0();
    if (has_j2)
    {
      get_joints_states_1_2();
      if (!joint_state_)
      {
        ROS_ERROR("SrhJointPositionController could not find the first joint relevant to \"%s\"\n",
                  joint_name_.c_str());
        return false;
      }
      if (!joint_state_2)
      {
        ROS_ERROR("SrhJointPositionController could not find the second joint relevant to \"%s\"\n",
                  joint_name_.c_str());
        return false;
      }
    }
    else
    {
      joint_state_ = robot_->getJointState(joint_name_);
      if (!joint_state_)
      {
        ROS_ERROR("SrhJointPositionController could not find joint named \"%s\"\n", joint_name_.c_str());
        return false;
      }
    }

    // get the min and max value for the current joint:
    get_min_max(robot_->robot_model_, joint_name_);

    friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhJointPositionController::setGains, this);
    serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhJointPositionController::resetGains, this);

    read_parameters();

    after_init();
    return true;
  }

  void SrhJointPositionController::starting(const ros::Time &time)
  {
    resetJointState();
    pid_controller_position_->reset();
    //setDefaults();
    if (has_j2)
      ROS_WARN_STREAM(
              "Reseting PID for joints " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
    else
      ROS_WARN_STREAM("Reseting PID for joint  " << joint_state_->joint_->name);

    bool did = pid_controller_position_->init(ros::NodeHandle(node_, "pid"));


  }

  bool SrhJointPositionController::setGains(sr_robot_msgs::SetPidGains::Request &req,
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
    //SrhJointPositionController::setDefaults();

    return true;
  }

  bool SrhJointPositionController::resetGains(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
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

  double SrhJointPositionController::map(double x, double in_min, double in_max, double out_min, double out_max)
  {
    if (x == 0)
      return 0.0;
    bool xs = false;
    if (x < 0)
      xs = true;
    x = abs(x);
    if ((x > 0) && ( x < in_max ))
    {
      x = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    } else {
      x = x - (in_max - out_max);
    }
    if (x < 2)
      x = 0.0;
    if (xs)
      x = x * -1.0;
    return x;
  }

  double SrhJointPositionController::corrects(double in_x)
  {
    bool negative = ( in_x < 0.0) ? true : false;
    double x = abs(in_x);
    double x2 = x*x;
    double x3 = x2*x;
    double corrected_val = 44.32656663379388 + 1.3212189376511323*in_x - 0.007179873277987306*x2 + 0.00002310811007344227*x3;
    if (negative)
      corrected_val = corrected_val * -1.0;
    return corrected_val;

  }

  double SrhJointPositionController::round(double d)
  {
    return floor(d + 0.5);
  }

  double SrhJointPositionController::interpolate(double input, double input_start, double input_end, double output_start, double output_end)
  {
    if (input > input_end){
        double output_offset = output_end - input_end;
        return input + output_offset;
    }
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    return output_start + SrhJointPositionController::round(slope * (input - input_start));
  }

  void SrhJointPositionController::setDefaults(void)
  {
    bool do_stuff = false;
    int smol_num;
    int lorg_num;
    std::string joint = joint_state_->joint_->name;
    if (boost::algorithm::istarts_with(joint, "rh_ff") || boost::algorithm::istarts_with(joint, "rh_mf") || boost::algorithm::istarts_with(joint, "rh_rf"))
    {
      if (has_j2)
      {
        if (boost::algorithm::iends_with(joint, "0"))
        {
          smol_num = 5;
          lorg_num = 30;
          do_stuff = true;
        }
      } else if (boost::algorithm::iends_with(joint, "3")){
        smol_num = 20;
        lorg_num = 80;
        do_stuff = true;
      } else {
        smol_num = 1;
        lorg_num = 4;
      }
    }
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    int_param.name = "smol_num";
    int_param.value = smol_num;
    conf.ints.push_back(int_param);

    int_param.name = "lorg_num";
    int_param.value = lorg_num;
    conf.ints.push_back(int_param);

    srv_req.config = conf;
    std::string ns = node_.getNamespace();
    if (do_stuff){
      ROS_WARN_STREAM("calling: " << ns << "/set_parameters... JOINT: " << joint);
      ros::service::call(ns + "/set_parameters", srv_req, srv_resp);
    } else {
      ROS_WARN_STREAM("NOT calling: " << ns << "/set_parameters... JOINT: " << joint);
    }

  }


  void SrhJointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_position_->getGains(p, i, d, i_max, i_min);
  }

  void SrhJointPositionController::update(const ros::Time &time, const ros::Duration &period)
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

    bool in_deadband = hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband);

    // don't compute the error if we're in the deadband.
    if (in_deadband)
    {
      error_position = 0.0;
    }

/*
    double timestamp = period.toSec();
    std::pair<double, double> pos_and_velocity = pos_filter.compute(error_position, timestamp);

    double error_dot = std::get<1>(pos_and_velocity);
*/
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

    //if (this->bypass == false)
    //  commanded_effort = SrhJointPositionController::map(commanded_effort, 0, this->in_max, 0, this->out_max);

    //if (this->correct == true)
    //  commanded_effort = SrhJointPositionController::corrects(commanded_effort);
std::string joint = joint_state_->joint_->name;
if (this->correct == true)
{
  if (has_j2)
  {
    if (boost::algorithm::istarts_with(joint, "rh_ff") || boost::algorithm::istarts_with(joint, "rh_mf") || boost::algorithm::istarts_with(joint, "rh_rf"))
    {

      if (commanded_effort > 0)
        commanded_effort = SrhJointPositionController::interpolate(commanded_effort, 0, this->j0_smol_num, 0, this->j0_lorg_num);

      if (commanded_effort < 0)
        commanded_effort = (-1.0)*SrhJointPositionController::interpolate((commanded_effort*-1.0), 0, this->j0_smol_num, 0, this->j0_lorg_num);

      if (loop_count_ % 500 == 0)
        std::cout << joint_state_->joint_->name << " == " << joint << ", starts with ff|mf|rf, DOES have J2\n";
    } else {
        std::cout << joint_state_->joint_->name << " == " << joint << ", DOESN'T start with ff|mf|rf, DOES have J2\n";
    }
  }
  if (boost::algorithm::iends_with(joint, "3"))
  {
    if (boost::algorithm::istarts_with(joint, "rh_ff") || boost::algorithm::istarts_with(joint, "rh_mf") || boost::algorithm::istarts_with(joint, "rh_rf"))
    {

      if (commanded_effort > 0)
        commanded_effort = SrhJointPositionController::interpolate(commanded_effort, 0, this->j3_smol_num, 0, this->j3_lorg_num);

      if (commanded_effort < 0)
        commanded_effort = (-1.0)*SrhJointPositionController::interpolate((commanded_effort*-1.0), 0, this->j3_smol_num, 0, this->j3_lorg_num);

      if (loop_count_ % 500 == 0)
        std::cout << joint_state_->joint_->name << " == " << joint << ", starts with ff|mf|rf, DOES NOT have J2, DOES END WITH 3\n";

    } else {
      if (loop_count_ % 500 == 0)
        std::cout << joint_state_->joint_->name << " == " << joint << ", DOESN'T start with ff|mf|rf, DOES END WITH 3\n";
    }
  } else {
      if (loop_count_ % 500 == 0)
        std::cout << joint_state_->joint_->name << " == " << joint << ", DOESN'T END WITH 3\n";
  }

}

    joint_state_->commanded_effort_ = commanded_effort;


   //if (loop_count_ % 200 == 0)
   //  std::cout << joint_state_->joint_->name << "+: " << this->in_max << ", -: " << this->out_max << "\n";

    //if (loop_count_ % 10 == 0)
    //{
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
    //}
    loop_count_++;
  }

  void SrhJointPositionController::read_parameters()
  {
    node_.param<double>("pid/max_force", max_force_demand, 1023.0);
    node_.param<double>("pid/position_deadband", position_deadband, 0.015);
    node_.param<int>("pid/friction_deadband", friction_deadband, 5);
  }

  void SrhJointPositionController::setCommandCB(const std_msgs::Float64ConstPtr &msg)
  {
    joint_state_->commanded_position_ = msg->data;
    if (has_j2)
    {
      joint_state_2->commanded_position_ = 0.0;
    }
  }

  void SrhJointPositionController::resetJointState()
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


