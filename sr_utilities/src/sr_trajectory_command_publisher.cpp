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
#include <algorithm>
#include <memory>
#include <sr_utilities/sr_trajectory_command_publisher.hpp>
#include <string>
#include <unordered_set>
#include <utility>
#include <sr_utilities_common/wait_for_param.h>

SrTrajectoryCommandPublisher::SrTrajectoryCommandPublisher(const std::vector<std::string>& expected_joints)
{
  setup_publishers(expected_joints);
}

SrTrajectoryCommandPublisher::~SrTrajectoryCommandPublisher()
{
}

void SrTrajectoryCommandPublisher::setup_publishers(const std::vector<std::string>& expected_joints)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue controller_list;
  wait_for_param(nh, "/move_group/controller_list");
  ros::param::get("/move_group/controller_list", controller_list);

  for (int i = 0; i < controller_list.size(); i++)
  {
    XmlRpc::XmlRpcValue controller = controller_list[i];
    std::string controller_name = controller["name"];
    auto joints = xmlrpcvalue_to_vector(controller["joints"]);

    if (expected_joints.size() > 0)
    {
      bool create_publisher = false;
      for (const auto& expected_joint : expected_joints)
      {
        if (std::find(joints.begin(), joints.end(), expected_joint) != joints.end())
        {
            create_publisher = true;
            break;
        }
      }
      if (!create_publisher) continue;
    }

    auto publisher_and_msg = std::make_shared<std::pair<ros::Publisher, trajectory_msgs::JointTrajectory>>(
        nh.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 1),
        trajectory_msgs::JointTrajectory());

    for (int k = 0; k < joints.size(); k++)
    {
      if (expected_joints.size() > 0 && !(std::find(expected_joints.begin(),
                                                    expected_joints.end(),
                                                    static_cast<std::string>(joints[k])) != expected_joints.end()))
      {
        continue;
      }
      joint_to_publisher_and_msg_.insert(std::make_pair(joints[k], publisher_and_msg));
    }
  }

  check_for_unsupported_joints(expected_joints);
}

void SrTrajectoryCommandPublisher::check_for_unsupported_joints(const std::vector<std::string>& expected_joints)
{
  std::vector<std::string> unsupported_joints;
  for (const auto& expected_joint : expected_joints)
  {
    if (joint_to_publisher_and_msg_.find(expected_joint) == joint_to_publisher_and_msg_.end())
    {
      unsupported_joints.push_back(expected_joint);
      ROS_ERROR_STREAM("No trajectory controller specified for joint: " << expected_joint);
    }
  }
  if (unsupported_joints.size() > 0)
  {
    throw std::runtime_error("There is no trajectory controller specified"
                               " in /move_group/controller_list for some of expected joints. Terminating... ");
  }

}

std::vector<std::string> SrTrajectoryCommandPublisher::xmlrpcvalue_to_vector(const XmlRpc::XmlRpcValue& xmlrpcvalue)
{
  std::vector<std::string> result;
  for (int i = 0; i < xmlrpcvalue.size(); i++)
  {
    result.push_back(static_cast<std::string>(xmlrpcvalue[i]));
  }
  return result;
}

void SrTrajectoryCommandPublisher::publish(const trajectory_msgs::JointTrajectory& joint_trajectory)
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

  for (const auto& publisher_and_msg : publishers_and_msgs)
  {
    publisher_and_msg->first.publish(publisher_and_msg->second);
  }
}
