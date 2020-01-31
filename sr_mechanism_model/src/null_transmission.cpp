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

/*
 * Author: Stuart Glaser
 *
 * modified by Ugo Cupcic
 */

#include "sr_mechanism_model/null_transmission.hpp"
#include <string>

using std::string;
using ros_ethercat_model::RobotState;
using ros_ethercat_model::Transmission;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::NullTransmission, Transmission)

namespace sr_mechanism_model
{

  bool NullTransmission::initXml(tinyxml2::XMLElement *elt, RobotState *robot)
  {
    if (!Transmission::initXml(elt, robot))
    {
      return false;
    }

    // reading the joint name
    tinyxml2::XMLElement *jel = elt->FirstChildElement("joint");
    if (!jel || !jel->Attribute("name"))
    {
      ROS_ERROR_STREAM("Joint name not specified in transmission " << name_);
      return false;
    }

    tinyxml2::XMLElement *ael = elt->FirstChildElement("actuator");
    if (!ael || !ael->Attribute("name"))
    {
      ROS_ERROR_STREAM("Transmission " << name_ << " has no actuator in configuration");
      return false;
    }

    joint_ = robot->getJointState(jel->Attribute("name"));
    actuator_ = new ros_ethercat_model::Actuator();
    actuator_->name_ = ael->Attribute("name");
    actuator_->command_.enable_ = true;

    return true;
  }

  void NullTransmission::propagatePosition()
  {
//  SrMotorActuator *act = static_cast<SrMotorActuator*>(actuator_);
//  joint_->position_ = act->state_.position_;
//  joint_->velocity_ = act->state_.velocity_;
//  joint_->effort_ = act->state_.last_measured_effort_;
  }

  void NullTransmission::propagateEffort()
  {
//  SrMotorActuator *act = static_cast<SrMotorActuator*>(actuator_);
//  act->command_.enable_ = true;
//  act->command_.effort_ = joint_->commanded_effort_;
  }

}  // namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
