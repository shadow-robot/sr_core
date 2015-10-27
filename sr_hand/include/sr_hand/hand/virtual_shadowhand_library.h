/**
 * @file virtual_shadowhand_library.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 10 Nov 2010
*
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
 * @brief 
 *
 *
 */

#ifndef SR_HAND_HAND_VIRTUAL_SHADOWHAND_LIBRARY_H
#define SR_HAND_HAND_VIRTUAL_SHADOWHAND_LIBRARY_H

#include "sr_hand/hand/sr_articulated_robot.h"
#include <string>
#include <vector>

namespace shadowrobot
{

class VirtualShadowhandLibrary :
        public virtual SRArticulatedRobot
{
public:
  VirtualShadowhandLibrary();

  virtual ~VirtualShadowhandLibrary()
  {
  };

  virtual int16_t sendupdate(std::string joint_name, double target);

  virtual JointData getJointData(std::string joint_name);

  virtual JointsMap getAllJointsData();

  virtual int16_t setContrl(std::string contrlr_name, JointControllerData ctrlr_data);

  virtual JointControllerData getContrl(std::string ctrlr_name);

  virtual int16_t setConfig(std::vector<std::string> myConfig);

  virtual void getConfig(std::string joint_name);

  virtual std::vector<DiagnosticData> getDiagnostics();
};  // end class

}  // namespace shadowrobot
#endif  // SR_HAND_HAND_VIRTUAL_SHADOWHAND_LIBRARY_H
