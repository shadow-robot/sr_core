/*
* Copyright 2010 Shadow Robot Company Ltd.
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

/**
 * @file   real_arm.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue Jun 29 14:56:10 2010
 *
 * @brief The real arm is a ROS interface to Shadow Robot muscle arm.
 *
 */

#ifndef SR_HAND_HAND_REAL_ARM_H
#define SR_HAND_HAND_REAL_ARM_H

#include <robot/config.h>
#include <string>
#include <vector>

#include "sr_hand/hand/sr_articulated_robot.h"

namespace shadowrobot
{

class RealArm :
        public SRArticulatedRobot
{
public:
  /**
   * Initializes the necessary mappings with a static list of names.
   */
  RealArm();

  virtual ~RealArm();

  // virtual classes defined in SRArticulatedRobot
  /**
   * This function will set the target of the object to the given angle and send it to
   * the robot.
   *
   *
   * @param joint_name The Joint in joints_map you wish to send the target to.
   * @param target The target in degree
   * @return 0 if success ; -1 if error
   */
  virtual int16_t sendupdate(std::string joint_name, double target);

  /**
   * Reads the data from the robot.
   *
   * @param joint_name The name of the joint, as specified in joints_map.
   * @return The information regarding this joint.
   */
  virtual JointData getJointData(std::string joint_name);

  virtual JointsMap getAllJointsData();

  virtual int16_t setContrl(std::string contrlr_name, JointControllerData ctrlr_data);

  virtual JointControllerData getContrl(std::string ctrlr_name);

  virtual int16_t setConfig(std::vector <std::string> myConfig);

  virtual void getConfig(std::string joint_name);

  /**
   * Get diagnostic information from the robot.
   * @return A vector containing all the diagnostics for the hand (motor information, etc...)
   */
  virtual std::vector <DiagnosticData> getDiagnostics();

protected:
  /**
   * Initialise a mapping for the joints.
   */
  void initializeMap();
};

}  // namespace shadowrobot

#endif  // SR_HAND_HAND_REAL_ARM_H
