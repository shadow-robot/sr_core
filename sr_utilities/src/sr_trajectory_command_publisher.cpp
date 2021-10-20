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

void SrTrajectoryCommandPublisher::publish(trajectory_msgs::JointTrajectory joint_trajectory)
{
  std::unordered_set<std::shared_ptr<std::pair<ros::Publisher,
    trajectory_msgs::JointTrajectory>>> publishers_and_msgs;
  for (int i = 0; i < joint_trajectory.joint_names.size(); i++)
  {
    std::string joint_name = joint_trajectory.joint_names[i];
    std::shared_ptr<std::pair<ros::Publisher, trajectory_msgs::JointTrajectory>>
      publisher_and_msg = get_publisher_and_msg(joint_name);
    auto result = publishers_and_msgs.insert(publisher_and_msg);
    if (result.second)
    {
      publisher_and_msg->second.header.stamp = joint_trajectory.header.stamp;
      publisher_and_msg->second.joint_names.clear();
      publisher_and_msg->second.points[0].positions.clear();
      publisher_and_msg->second.points[0].time_from_start = joint_trajectory.points[0].time_from_start;
    }
    publisher_and_msg->second.joint_names.push_back(joint_name);
    publisher_and_msg->second.points[0].positions.push_back(joint_trajectory.points[0].positions[i]);
  }
  for (auto& publisher_and_msg : publishers_and_msgs)
  {
    publisher_and_msg->first.publish(publisher_and_msg->second);
  }
}

std::shared_ptr<std::pair<ros::Publisher, trajectory_msgs::JointTrajectory>>
  SrTrajectoryCommandPublisher::get_publisher_and_msg(std::string joint_name)
{
  auto it = joint_to_publisher_and_msg_.find(joint_name);
  if (it != joint_to_publisher_and_msg_.end())
  {
    return it->second;
  }
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue controller_list;
  ros::param::get("/move_group/controller_list", controller_list);
  for (int i = 0; i < controller_list.size(); i++)
  {
    XmlRpc::XmlRpcValue controller = controller_list[i];
    std::string controller_name = controller["name"];
    XmlRpc::XmlRpcValue joints = controller["joints"];
    for (int j = 0; j < joints.size(); j++)
    {
      if (joints[j] == joint_name)
      {
        std::shared_ptr<std::pair<ros::Publisher, trajectory_msgs::JointTrajectory>> publisher_and_msg =
          std::make_shared<std::pair<ros::Publisher, trajectory_msgs::JointTrajectory>>(
            nh.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 1),
            trajectory_msgs::JointTrajectory());
        publisher_and_msg->second.points.resize(1);  // Our trajectory controllers use single array of points
        for (int k = 0; k < joints.size(); k++)
        {
          joint_to_publisher_and_msg_.insert(std::make_pair(joints[k], publisher_and_msg));
        }
        return publisher_and_msg;
      }
    }
  }
  throw std::runtime_error("There is no trajectory controller specified in /move_group/controller_list for joint " +
    joint_name);
}
