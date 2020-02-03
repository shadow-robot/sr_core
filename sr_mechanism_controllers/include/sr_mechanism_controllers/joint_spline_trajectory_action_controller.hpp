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
 * @file   joint_spline_trajectory_action_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Mar  4 12:57:46 2011
 *
 * @brief  Implement an actionlib server to execute a
 * control_msgs::JointTrajectoryAction. Follows the
 * given trajectory with the arm.
 *
 *
 */

#ifndef _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_
#define _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_

#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


namespace shadowrobot
{
class JointTrajectoryActionController
{
  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
public:
  JointTrajectoryActionController();

private:
  ros::NodeHandle nh, nh_tilde;
  ros::Subscriber command_sub;
  ros::Publisher sr_arm_target_pub;
  ros::Publisher sr_hand_target_pub;
  ros::Publisher desired_joint_state_pusblisher;
  std::vector<std::string> joint_names_;
  ros::ServiceClient joint_state_client;
  std::map<std::string, double> joint_state_map;

  std::vector<ros::Publisher> controller_publishers;   // This vector stores publishers to each joint controller.
  std::map<std::string, std::string> jointControllerMap;  // stores a map of controller name and associated joints
  std::map<std::string, unsigned int> jointPubIdxMap;  // stores a map of publisher associated with joint index
  std::map<std::string, unsigned int> joint_state_idx_map;  // store internal order of joints
  bool use_sendupdate;

  ros::Time last_time_;
  boost::shared_ptr<JTAS> action_server;

  // coef[0] + coef[1]*t + ... + coef[5]*t^5
  struct Spline
  {
    std::vector<double> coef;

    Spline() : coef(6, 0.0)
    {
    }
  };

  struct Segment
  {
    double start_time;
    double duration;
    std::vector<Spline> splines;
  };
  typedef std::vector<Segment> SpecifiedTrajectory;

  std::vector<double> q, qd, qdd;  // Preallocated in init

  // Samples, but handling time bounds.  When the time is past the end
  // of the spline duration, the position is the last valid position,
  // and the derivatives are all 0.
  static void sampleSplineWithTimeBounds(const std::vector<double> &coefficients, double duration, double time,
                                         double &position, double &velocity, double &acceleration);


  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  void updateJointState();

  bool getPosition(std::string joint_name, double &position);
};
}  // namespace shadowrobot

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif


