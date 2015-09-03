/**
 * @file virtual_shadowhand_library.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 10 Nov 2010
 *
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
 * @brief 
 *
 *
 */

#include "sr_hand/hand/virtual_shadowhand_library.h"
#include <ros/ros.h>
#include <map>
#include <string>
#include <vector>

namespace shadowrobot
{

  VirtualShadowhandLibrary::VirtualShadowhandLibrary() :
          SRArticulatedRobot()
  {
    joints_map_mutex.lock();

    JointData tmpData;
    JointData tmpDataZero;
    JointControllerData tmpController;
    tmpDataZero.isJointZero = 1;
    tmpDataZero.max = 180.0;

    joints_map["FFJ0"] = tmpDataZero;
    joints_map["FFJ1"] = tmpData;
    joints_map["FFJ2"] = tmpData;
    joints_map["FFJ3"] = tmpData;
    tmpData.min = -20.0;
    tmpData.max = 20.0;
    joints_map["FFJ4"] = tmpData;

    joints_map["MFJ0"] = tmpDataZero;
    tmpData.min = 0.0;
    tmpData.max = 90.0;
    joints_map["MFJ1"] = tmpData;
    joints_map["MFJ2"] = tmpData;
    joints_map["MFJ3"] = tmpData;
    tmpData.min = -20.0;
    tmpData.max = 20.0;
    joints_map["MFJ4"] = tmpData;

    joints_map["RFJ0"] = tmpDataZero;
    tmpData.min = 0.0;
    tmpData.max = 90.0;
    joints_map["RFJ1"] = tmpData;
    joints_map["RFJ2"] = tmpData;
    joints_map["RFJ3"] = tmpData;
    tmpData.min = -20.0;
    tmpData.max = 20.0;
    joints_map["RFJ4"] = tmpData;

    joints_map["LFJ0"] = tmpDataZero;
    tmpData.min = 0.0;
    tmpData.max = 90.0;
    joints_map["LFJ1"] = tmpData;
    joints_map["LFJ2"] = tmpData;
    joints_map["LFJ3"] = tmpData;
    tmpData.min = -20.0;
    tmpData.max = 20.0;
    joints_map["LFJ4"] = tmpData;
    tmpData.min = 0.0;
    tmpData.max = 45.0;
    joints_map["LFJ5"] = tmpData;

    tmpData.min = 0.0;
    tmpData.max = 90.0;
    joints_map["THJ1"] = tmpData;
    tmpData.min = -40.0;
    tmpData.max = 40.0;
    joints_map["THJ2"] = tmpData;
    tmpData.min = -15.0;
    tmpData.max = 15.0;
    joints_map["THJ3"] = tmpData;
    tmpData.min = 0.0;
    tmpData.max = 75.0;
    joints_map["THJ4"] = tmpData;
    tmpData.min = -60.0;
    tmpData.max = 60.0;
    joints_map["THJ5"] = tmpData;

    tmpData.min = -30.0;
    tmpData.max = 45.0;
    joints_map["WRJ1"] = tmpData;
    tmpData.min = -30.0;
    tmpData.max = 10.0;
    joints_map["WRJ2"] = tmpData;

    joints_map_mutex.unlock();
  }

  int16_t VirtualShadowhandLibrary::sendupdate(std::string joint_name, double target)
  {
    return (int16_t) 0;
  }

  JointData VirtualShadowhandLibrary::getJointData(std::string joint_name)
  {
    JointData tmp;
    return tmp;
  }

  std::map<std::string, JointData> VirtualShadowhandLibrary::getAllJointsData()
  {
    joints_map_mutex.lock();
    JointsMap tmp_map = JointsMap(joints_map);
    joints_map_mutex.unlock();
    return tmp_map;
  }

  int16_t VirtualShadowhandLibrary::setContrl(std::string contrlr_name, JointControllerData ctrlr_data)
  {
    return (int16_t) 0;
  }

  JointControllerData VirtualShadowhandLibrary::getContrl(std::string ctrlr_name)
  {
    JointControllerData tmp;
    return tmp;
  }

  int16_t VirtualShadowhandLibrary::setConfig(std::vector<std::string> myConfig)
  {
    return (int16_t) 0;
  }

  void VirtualShadowhandLibrary::getConfig(std::string joint_name)
  {
  }

  std::vector<DiagnosticData> VirtualShadowhandLibrary::getDiagnostics()
  {
    std::vector<DiagnosticData> tmp;
    return tmp;
  }
}  // namespace shadowrobot
