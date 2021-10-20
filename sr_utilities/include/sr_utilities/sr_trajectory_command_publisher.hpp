/*
* Copyright 2021 Shadow Robot Company Ltd.
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

#ifndef _SR_TRAJECTORY_COMMAND_PUBLISHER_HPP_
#define _SR_TRAJECTORY_COMMAND_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>

class SrTrajectoryCommandPublisher
{
public:
  void publish(trajectory_msgs::JointTrajectory joint_trajectory);
private:
  std::map<std::string, std::shared_ptr<ros::Publisher>> joint_to_publisher;
  std::shared_ptr<ros::Publisher> get_publisher_for_joint(std::string joint_name);
};

#endif  // _SR_TRAJECTORY_COMMAND_PUBLISHER_HPP_
