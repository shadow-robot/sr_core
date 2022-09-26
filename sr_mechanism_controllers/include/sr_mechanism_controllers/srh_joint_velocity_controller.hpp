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
 * @file   srh_joint_velocity_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
 *
 * @brief  Follows a velocity target. The velocity demand is converted into a force
 * demand by a PID loop.
 *
 */


#ifndef _SRH_JOINT_VELOCITY_CONTROLLER_HPP_
#define _SRH_JOINT_VELOCITY_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_mechanism_controllers/sr_friction_compensation.hpp>


namespace controller
{
class SrhJointVelocityController :
        public SrController
{
public:
  SrhJointVelocityController();

  bool init(ros_ethercat_model::RobotStateInterface *robot, ros::NodeHandle &n);

  virtual void starting(const ros::Time &time);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update(const ros::Time &time, const ros::Duration &period);

  virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  virtual bool resetGains(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  bool setGains(sr_robot_msgs::SetPidGains::Request &req, sr_robot_msgs::SetPidGains::Response &resp);

  bool setTau(sr_robot_msgs::SetTau::Request &req, sr_robot_msgs::SetTau::Response &resp);

private:
  /// Internal PID controller for the velocity loop.
  boost::scoped_ptr<control_toolbox::Pid> pid_controller_velocity_;

  /// filter to smooth noisy joint date (ML Data collection)
  sr_math_utils::filters::LowPassFilter pos_filter;

  /// the velocity deadband value used in the hysteresis_deadband
  double velocity_deadband;

  double filter_tau = 0.05;

  /// We're using an hysteresis deadband.
  sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

  /// read all the controller settings from the parameter server
  void read_parameters();

  /// clamp the command to velocity limits
  double clamp_command(double cmd);

  /// set the velocity target from a topic
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
