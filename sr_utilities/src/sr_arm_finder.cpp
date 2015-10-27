/*
* File:  sf_arm_finder.cpp
* Author: Andriy Petlovanyy <andriy@shadowrobot.com>
* Copyright 2015 Shadow Robot Company Ltd.
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
* @brief see README.md
*/

#include "sr_utilities/sr_arm_finder.hpp"
#include "sr_utilities/sr_hand_finder.hpp"
#include "urdf_parser/urdf_parser.h"
#include <map>
#include <set>
#include <vector>
#include <string>

namespace shadow_robot
{
SrArmFinder::SrArmFinder()
{
  if (ros::param::has("arm"))
  {
    ros::param::get("arm/mapping", arm_config_.mapping_);
    ros::param::get("arm/joint_prefix", arm_config_.joint_prefix_);

    if (ros::param::has("robot_description"))
    {
      auto default_hand_joints_vector = SrHandFinder::get_default_joints();
      const std::set<std::string> hand_default_joints(default_hand_joints_vector.begin(),
                                                      default_hand_joints_vector.end());
      std::string robot_description;
      ros::param::get("robot_description", robot_description);
      const auto hand_urdf = urdf::parseURDF(robot_description);

      for (const auto &joint : hand_urdf->joints_)
      {
        const std::string joint_name = joint.first;
        if ((urdf::Joint::FIXED != joint.second->type) &&
            (hand_default_joints.end() == hand_default_joints.find(joint_name)) &&
            std::none_of(hand_default_joints.begin(), hand_default_joints.end(),
                         [&joint_name](const std::string &item)
                         {
                           return (joint_name.size() - item.size()) == joint_name.find_last_of(item);
                         }))  // NOLINT(whitespace/braces)
        {
          for (const auto &hand : arm_config_.mapping_)
          {
            const auto hand_serial = hand.first;
            const auto hand_id = hand.second;

            if ((arm_config_.joint_prefix_.count(hand_serial) > 0) &&
                (0 == joint_name.find(arm_config_.joint_prefix_[hand_serial])))
             {
              joints_[hand_id].push_back(joint_name);
            }
          }
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("No robot_description found on parameter server. Assuming that there is no arm.");
    }
  }
  else
  {
    ROS_ERROR_STREAM("No arm is detected");
  }
}

ArmConfig SrArmFinder::get_arm_parameters()
{
  return arm_config_;
}

std::map<std::string, std::vector<std::string> > SrArmFinder::get_joints()
{
  return joints_;
}

} /* namespace shadow_robot */
