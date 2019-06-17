/**
 * @file   srh_joint_variable_pid_position_controller.cpp
 * @author Anastasios <anastasios@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
 *
 * Copyright 2019 Shadow Robot Company Ltd.
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
 *
 *
 *
 * @brief  Follows a position target. The position demand is converted into a force
 * demand by a PID loop.
 *
 * The code is base on srh_joint_position_controller.hpp
 *
 */


#ifndef _SRH_JOINT_POSITION_CONTROLLER_HPP_
#define _SRH_JOINT_POSITION_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>


namespace controller
{
class SrhJointVariablePidPositionController :
        public SrController
{
public:
  SrhJointVariablePidPositionController();

  bool init(ros_ethercat_model::RobotStateInterface *robot, ros::NodeHandle &n);

  virtual void starting(const ros::Time &time);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update(const ros::Time &time, const ros::Duration &period);

  virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  virtual bool resetGains(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  bool setGains(sr_robot_msgs::SetPidGains::Request &req, sr_robot_msgs::SetPidGains::Response &resp);

  virtual void updatePid(double, double);

private:
  /// Internal PID controller for the position loop.
  boost::scoped_ptr<control_toolbox::Pid> pid_controller_position_;

  /// the position deadband value used in the hysteresis_deadband
  double position_deadband;

  /// initial PID parameters
  double p_init;
  double i_init;
  double d_init;
  double i_clamp;
  double error_old;
  double set_point_old;
  double frequency;
  double smoothing_velocity_min;
  double smoothing_velocity_max;
  double smoothing_factor_p;
  double smoothing_factor_i;
  double smoothing_factor_d;

  /// We're using an hysteresis deadband.
  sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

  /// read all the controller settings from the parameter server
  void read_parameters();

  /// set the position target from a topic
  void setCommandCB(const std_msgs::Float64ConstPtr &msg);

  void resetJointState();
};
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
