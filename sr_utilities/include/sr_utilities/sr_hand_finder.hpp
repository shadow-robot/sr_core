/*
* Copyright 2015 Shadow Robot Company Ltd.
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

/*
* File:  sf_hand_finder.hpp
* Author: Vahid Aminzadeh <vahid@shadowrobot.com>
* @brief see README.md
*/

#pragma once

#include "ros/ros.h"
#include <map>
#include <string>
#include <vector>

namespace shadow_robot
{

struct HandConfig
{
  std::map<std::string, std::string> mapping_;
  std::map<std::string, std::string> joint_prefix_;
};

class SrHandFinder
{
public:
  static const std::vector<std::string> get_default_joints();

  std::map<std::string, std::vector<std::string> > get_joints();

  HandConfig get_hand_parameters();

  SrHandFinder();

  virtual ~SrHandFinder()
  {
  }

private:
  static const std::vector<std::string> joint_names_;
  ros::NodeHandle node_handle_;
  HandConfig hand_config_;
  std::map<std::string, std::vector<std::string> > joints_;
  std::map<std::string, std::string> calibration_path_;

  void generate_joints_with_prefix();
};

} /* namespace shadow_robot */
