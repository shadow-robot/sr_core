/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
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
