/**
 * @file   joint_trajectory_action_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Mar  4 12:57:46 2011
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
  typedef std::vector<trajectory_msgs::JointTrajectoryPoint> JointTrajectoryPointVec;
  typedef std::map<std::string, ros::Publisher> JointPubMap;
public:
  JointTrajectoryActionController();

private:
  ros::NodeHandle nh;
  JointPubMap joint_pub;
  boost::shared_ptr<JTAS> action_server;

  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
};
}  // namespace shadowrobot

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
