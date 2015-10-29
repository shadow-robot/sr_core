/*
* File:  sf_arm_finder.h
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

#ifndef SR_CORE_SR_ARM_FINDER_H
#define SR_CORE_SR_ARM_FINDER_H

#include "ros/ros.h"
#include <map>
#include <string>
#include <vector>

namespace shadow_robot
{

struct ArmConfig
{
  std::map<std::string, std::string> mapping_;
  std::map<std::string, std::string> joint_prefix_;
};

class SrArmFinder
{
public:
  SrArmFinder();

  virtual ~SrArmFinder()
  {
  }

  ArmConfig get_arm_parameters();

  std::map<std::string, std::vector<std::string> > get_joints();

private:
  ArmConfig arm_config_;
  std::map<std::string, std::vector<std::string> > joints_;
};

} /* namespace shadow_robot */

#endif  // SR_CORE_SR_ARM_FINDER_H
