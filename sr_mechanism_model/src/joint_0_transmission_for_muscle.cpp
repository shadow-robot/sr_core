/*
* Copyright 2011 Shadow Robot Company Ltd.
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
 * @file   joint_0_transmission_for_muscle.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * @brief This is the implementation of the transmission for the joint 0s.
 * We need a specific transmission which takes into account that 2 joints
 * are actuated with only one actuator.
 *
 *
 */

#include "sr_mechanism_model/joint_0_transmission_for_muscle.hpp"
#include <string>

using std::string;
using sr_actuator::SrMuscleActuator;
using ros_ethercat_model::RobotState;
using ros_ethercat_model::Transmission;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::J0TransmissionForMuscle, Transmission)

namespace sr_mechanism_model
{

  bool J0TransmissionForMuscle::initXml(tinyxml2::XMLElement *elt, RobotState *robot)
  {
    if (!SimpleTransmissionForMuscle::initXml(elt, robot))
    {
      return false;
    }

    string joint2_name = joint_->joint_->name;
    joint2_name[joint2_name.size() - 1] = '2';
    joint2_ = robot->getJointState(joint2_name);

    return true;
  }

  void J0TransmissionForMuscle::propagatePosition()
  {
    // the size is either 2 or 0 when the joint hasn't been updated yet
    // (joint 0 is composed of the 2 calibrated values: joint 1 and joint 2)
    SrMuscleActuator *act = static_cast<SrMuscleActuator *>(actuator_);
    size_t size = act->muscle_state_.calibrated_sensor_values_.size();
    if (size == 0)
    {
      ROS_DEBUG_STREAM("READING pos " << act->state_.position_
                       << " J1 " << act->muscle_state_.calibrated_sensor_values_[0]
                       << " J2 " << act->muscle_state_.calibrated_sensor_values_[1]);

      joint_->position_ = act->muscle_state_.calibrated_sensor_values_[0];
      joint2_->position_ = act->muscle_state_.calibrated_sensor_values_[1];

      joint_->velocity_ = act->state_.velocity_ / 2.0;
      joint2_->velocity_ = act->state_.velocity_ / 2.0;

      // We don't want to define a modified version of JointState, as that would imply using a modified version
      // of robot_state.hpp, controller manager, ethercat_hardware and ros_etherCAT main loop
      // So we will encode the two uint16_t that contain the data from the muscle pressure sensors
      // into the double effort_. (We don't have any measured effort in the muscle hand anyway).
      // Then in the joint controller we will decode that back into uint16_t.
      joint_->effort_ = (static_cast<double>((act->muscle_state_.pressure_[1]) * 0x10000))
                        + static_cast<double>(act->muscle_state_.pressure_[0]);
      joint2_->effort_ = (static_cast<double>((act->muscle_state_.pressure_[1]) * 0x10000))
                         + static_cast<double>(act->muscle_state_.pressure_[0]);
    }
  }

}  // namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
