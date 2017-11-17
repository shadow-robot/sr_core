/**
 * @file   srh_grasp_controller.hpp
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


#ifndef _SRH_GRASP_CONTROLLER_HPP_
#define _SRH_GRASP_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <std_msgs/Float64MultiArray.h>
#include "vector"

namespace controller
{
class SrhGraspController :
        public SrController
{
public:
  SrhGraspController();

  bool init(ros_ethercat_model::RobotStateInterface *robot, ros::NodeHandle &n);
  int i;

  virtual void starting(const ros::Time &time);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update(const ros::Time &time, const ros::Duration &period);

private:
  std::vector<std::vector<ros_ethercat_model::JointState *> > joints_;
  std::vector<control_toolbox::Pid> pids_;
  std::vector<double> mins_, maxs_, vel_mins_, vel_maxs_, eff_mins_, eff_maxs_;
  std::vector<double> position_command_;
  std::vector<double> max_force_demands_;
  std::vector<double> position_deadbands_;
  std::vector<int> friction_deadbands_;

  /// the position deadband value used in the hysteresis_deadband
  double position_deadband;
  
  boost::scoped_ptr<control_toolbox::Pid> pid_controller_position_;

  /// We're using an hysteresis deadband.
  sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

  /// read all the controller settings from the parameter server
  void read_parameters();

  void resetJointState();
  bool is_joint_0(const std::string &);
  void get_joints_states_1_2(const std::string &, std::vector<ros_ethercat_model::JointState*> &);
  void get_min_max(urdf::Model model, const std::vector<std::string> &);
  void setCommandCB(const std_msgs::Float64MultiArrayConstPtr &msg);
  
};
}  // namespace controller

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
