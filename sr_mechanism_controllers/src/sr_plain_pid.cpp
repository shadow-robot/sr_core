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
  Author: Melonee Wise
  Contributors: Dave Coleman, Jonathan Bohren, Bob Holmberg, Wim Meeussen
  Desc: Implements a standard proportional-integral-derivative controller
*/

#include <sr_mechanism_controllers/sr_plain_pid.hpp>
#include <tinyxml.h>
#include <string>
#include <tuple>

#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/minmax.hpp>

namespace controller
{

PlainPid::PlainPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  setGains(p, i, d, i_max, i_min, antiwindup);

  reset();
}

PlainPid::~PlainPid()
{
}

void PlainPid::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  setGains(p, i, d, i_max, i_min, antiwindup);

  reset();
}

bool PlainPid::initParam(const std::string& prefix, const bool quiet)
{
  ros::NodeHandle nh(prefix);
  return init(nh, quiet);
}

bool PlainPid::init(const ros::NodeHandle &node, const bool quiet)
{
  ros::NodeHandle nh(node);

  Gains gains;

  // Load PID gains from parameter server
  if (!nh.getParam("p", gains.p_gain_))
  {
    if (!quiet)
    {
      ROS_ERROR("No p gain specified for pid.  Namespace: %s", nh.getNamespace().c_str());
    }
    return false;
  }
  // Only the P gain is required, the I and D gains are optional and default to 0:
  nh.param("i", gains.i_gain_, 0.0);
  nh.param("d", gains.d_gain_, 0.0);

  // Load integral clamp from param server or default to 0
  double i_clamp;
  nh.param("i_clamp", i_clamp, 0.0);
  gains.i_max_ = std::abs(i_clamp);
  gains.i_min_ = -std::abs(i_clamp);
  if (nh.hasParam("i_clamp_min"))
  {
    nh.param("i_clamp_min", gains.i_min_, gains.i_min_);  // use i_clamp_min parameter, otherwise keep -i_clamp
    gains.i_min_ = -std::abs(gains.i_min_);  // make sure the value is <= 0
  }
  if (nh.hasParam("i_clamp_max"))
  {
    nh.param("i_clamp_max", gains.i_max_, gains.i_max_);  // use i_clamp_max parameter, otherwise keep i_clamp
    gains.i_max_ = std::abs(gains.i_max_);  // make sure the value is >= 0
  }
  nh.param("antiwindup", gains.antiwindup_, false);

  setGains(gains.p_gain_, gains.i_gain_, gains.d_gain_, gains.i_max_, gains.i_min_, gains.antiwindup_);

  reset();

  return true;
}

bool PlainPid::initXml(TiXmlElement *config)
{
  double i_clamp;
  i_clamp = config->Attribute("iClamp") ? atof(config->Attribute("iClamp")) : 0.0;

  setGains(
    config->Attribute("p") ? atof(config->Attribute("p")) : 0.0,
    config->Attribute("i") ? atof(config->Attribute("i")) : 0.0,
    config->Attribute("d") ? atof(config->Attribute("d")) : 0.0,
    std::abs(i_clamp),
    -std::abs(i_clamp),
    config->Attribute("antiwindup") ? atof(config->Attribute("antiwindup")) : false);

  reset();

  return true;
}

void PlainPid::reset()
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cmd_ = 0.0;
}

void PlainPid::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool antiwindup;
  getGains(p, i, d, i_max, i_min, antiwindup);
}

void PlainPid::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  p     = pid_gains.p_gain_;
  i     = pid_gains.i_gain_;
  d     = pid_gains.d_gain_;
  i_max = pid_gains.i_max_;
  i_min = pid_gains.i_min_;
  antiwindup = pid_gains.antiwindup_;
}

void PlainPid::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  pid_gains.p_gain_ = p;
  pid_gains.i_gain_ = i;
  pid_gains.d_gain_ = d;
  pid_gains.i_max_ = i_max;
  pid_gains.i_min_ = i_min;
  pid_gains.antiwindup_ = antiwindup;
}

double PlainPid::computeCommand(double error, ros::Duration dt)
{
  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
    return 0.0;

  double error_dot = d_error_;

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
    error_dot = (error - p_error_last_) / dt.toSec();
    p_error_last_ = error;
  }

  return computeCommand(error, error_dot, dt);
}

double PlainPid::updatePid(double error, ros::Duration dt)
{
  return -computeCommand(error, dt);
}

double PlainPid::computeCommand(double error, double error_dot, ros::Duration dt)
{
  double p_term, d_term, i_term;
  p_error_ = error;  // this is error = target - state
  d_error_ = error_dot;

  if (dt == ros::Duration(0.0) ||
  std::isnan(error) ||
  std::isinf(error) ||
  std::isnan(error_dot) ||
  std::isinf(error_dot))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = pid_gains.p_gain_ * p_error_;

  // Calculate the integral of the position error
  i_error_ += dt.toSec() * p_error_;

  if (pid_gains.antiwindup_ && pid_gains.i_gain_ != 0)
  {
    // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
    boost::tuple<double, double> bounds = boost::minmax<double>(pid_gains.i_min_ / pid_gains.i_gain_,
                                                                pid_gains.i_max_ / pid_gains.i_gain_);
    i_error_ = boost::algorithm::clamp(i_error_, bounds.get<0>(), bounds.get<1>());
  }

  // Calculate integral contribution to command
  i_term = pid_gains.i_gain_ * i_error_;

  if (!pid_gains.antiwindup_)
  {
    // Limit i_term so that the limit is meaningful in the output
    i_term = boost::algorithm::clamp(i_term, pid_gains.i_min_, pid_gains.i_max_);
  }

  // Calculate derivative contribution to command
  d_term = pid_gains.d_gain_ * d_error_;

  // Compute the command
  cmd_ = p_term + i_term + d_term;

  return cmd_;
}

double PlainPid::updatePid(double error, double error_dot, ros::Duration dt)
{
  return -computeCommand(error, error_dot, dt);
}

void PlainPid::setCurrentCmd(double cmd)
{
  cmd_ = cmd;
}

double PlainPid::getCurrentCmd()
{
  return cmd_;
}

void PlainPid::getCurrentPIDErrors(double *pe, double *ie, double *de)
{
  *pe = p_error_;
  *ie = i_error_;
  *de = d_error_;
}

void PlainPid::printValues()
{
  ROS_INFO_STREAM_NAMED("pid", "Current Values of PID Class:\n"
    << "  P Gain: " << pid_gains.p_gain_ << "\n"
    << "  I Gain: " << pid_gains.i_gain_ << "\n"
    << "  D Gain: " << pid_gains.d_gain_ << "\n"
    << "  I_Max:  " << pid_gains.i_max_  << "\n"
    << "  I_Min:  " << pid_gains.i_min_  << "\n"
    << "  Antiwindup:  " << pid_gains.antiwindup_  << "\n"
    << "  P_Error_Last: " << p_error_last_  << "\n"
    << "  P_Error:      " << p_error_  << "\n"
    << "  I_Error:       " << i_error_  << "\n"
    << "  D_Error:      " << d_error_  << "\n"
    << "  Command:      " << cmd_);
}
}  // namespace controller
