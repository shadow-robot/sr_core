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

#include <memory>
#include <sr_utilities/sr_trajectory_command_publisher.hpp>
#include <string>
#include <unordered_map>
#include <utility>

void SrTrajectoryCommandPublisher::publish(trajectory_msgs::JointTrajectory joint_trajectory)
{
  if (!initialised)
  {
    initialise();
  }
  std::unordered_map<std::shared_ptr<ros::Publisher>,
    std::shared_ptr<trajectory_msgs::JointTrajectory>> publisher_to_msg;
  for (int i = 0; i < joint_trajectory.joint_names.size(); i++)
  {
    std::string joint_name = joint_trajectory.joint_names[i];
    double position = joint_trajectory.points[0].positions[i];
    std::shared_ptr<ros::Publisher> publisher = joint_to_publisher[joint_name];
    auto it = publisher_to_msg.find(publisher);
    std::shared_ptr<trajectory_msgs::JointTrajectory> msg;
    if (it == publisher_to_msg.end())
    {
      msg = std::make_shared<trajectory_msgs::JointTrajectory>();
      msg->header.stamp = joint_trajectory.header.stamp;
      msg->points.resize(1);
      msg->points[0].time_from_start = joint_trajectory.points[0].time_from_start;
      publisher_to_msg.insert(std::make_pair(publisher, msg));
    }
    else
    {
      msg = it->second;
    }
    msg->joint_names.push_back(joint_name);
    msg->points[0].positions.push_back(position);
  }
  for (auto& entry : publisher_to_msg)
  {
    entry.first->publish(*entry.second);
  }
}

void SrTrajectoryCommandPublisher::SrTrajectoryCommandPublisher::initialise()
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue controller_list;
  ros::param::get("/move_group/controller_list", controller_list);
  for (int i = 0; i < controller_list.size(); i++)
  {
    XmlRpc::XmlRpcValue controller = controller_list[i];
    std::string controller_name = controller["name"];
    std::shared_ptr<ros::Publisher> publisher = std::make_shared<ros::Publisher>(
      nh.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 1));
    XmlRpc::XmlRpcValue joints = controller["joints"];
    for (int j = 0; j < joints.size(); j++)
    {
      std::string joint_name = joints[j];
      joint_to_publisher.insert(std::make_pair(joint_name, publisher));
    }
  }
  initialised = true;
}
