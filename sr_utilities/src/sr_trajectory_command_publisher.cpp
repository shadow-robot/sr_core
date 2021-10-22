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

#include <exception>
#include <memory>
#include <sr_utilities/sr_trajectory_command_publisher.hpp>
#include <string>
#include <unordered_set>
#include <utility>
#include <sr_utilities_common/wait_for_param.h>

SrTrajectoryCommandPublisher::SrTrajectoryCommandPublisher()
{
  setup_publishers();
}

SrTrajectoryCommandPublisher::~SrTrajectoryCommandPublisher()
{
}

void SrTrajectoryCommandPublisher::setup_publishers()
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue controller_list;
  wait_for_param(nh, "/move_group/controller_list");
  ros::param::get("/move_group/controller_list", controller_list);

  for (int i = 0; i < controller_list.size(); i++)
  {
    XmlRpc::XmlRpcValue controller = controller_list[i];
    std::string controller_name = controller["name"];
    XmlRpc::XmlRpcValue joints = controller["joints"];

    auto publisher_and_msg = std::make_shared<std::pair<ros::Publisher, trajectory_msgs::JointTrajectory>>(
        nh.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 1),
        trajectory_msgs::JointTrajectory());

    for (int k = 0; k < joints.size(); k++)
    {
      joint_to_publisher_and_msg_.insert(std::make_pair(joints[k], publisher_and_msg));
    }
  }
}

void SrTrajectoryCommandPublisher::publish(trajectory_msgs::JointTrajectory joint_trajectory)
{
  std::unordered_set<std::shared_ptr<std::pair<ros::Publisher,
    trajectory_msgs::JointTrajectory>>> publishers_and_msgs;

  for (int i = 0; i < joint_trajectory.joint_names.size(); i++)
  {
    std::string joint_name = joint_trajectory.joint_names[i];
    std::shared_ptr<std::pair<ros::Publisher, trajectory_msgs::JointTrajectory>>
      publisher_and_msg = joint_to_publisher_and_msg_.at(joint_name);

    if (publishers_and_msgs.insert(publisher_and_msg).second)
    {
      // New element was added to the set, reset cached message to the initial state
      publisher_and_msg->second.header.stamp = joint_trajectory.header.stamp;
      publisher_and_msg->second.joint_names.clear();
      publisher_and_msg->second.points = joint_trajectory.points;
      for (int k = 0; k < publisher_and_msg->second.points.size(); k++)
      {
        publisher_and_msg->second.points[k].positions.clear();
      }
    }
    publisher_and_msg->second.joint_names.push_back(joint_name);

    for (int k = 0; k < joint_trajectory.points.size(); k++)
    {
      publisher_and_msg->second.points[k].positions.push_back(joint_trajectory.points[k].positions[i]);
    }
  }

  for (auto& publisher_and_msg : publishers_and_msgs)
  {
    publisher_and_msg->first.publish(publisher_and_msg->second);
  }
}
