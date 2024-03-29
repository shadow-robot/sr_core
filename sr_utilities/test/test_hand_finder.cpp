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
 * File:  test_hand_finder.cpp
 * Author: Vahid Aminzadeh <vahid@shadowrobot.com>
 * Copyright:
 *
 * @brief see README.md
 */
#include <gtest/gtest.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <utility>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include "sr_utilities/sr_hand_finder.hpp"
#include <boost/foreach.hpp>

using std::vector;
using std::map;
using std::string;
using std::pair;

void ASSERT_MAP_HAS_VALUE(const map<string, string> &map, const string &value)
{
  bool found = false;
  pair<string, string> item;
  BOOST_FOREACH(item, map)
  {
    if (value == item.second)
    {
      found = true;
    }
  }

  ASSERT_TRUE(found);
}

TEST(SrHandFinder, hand_absent_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  shadow_robot::SrHandFinder hand_finder;
  map<string, vector<string> > hand_joints(hand_finder.get_joints());
  ASSERT_EQ(hand_joints.size(), 0);
}

TEST(SrHandFinder, one_hand_no_robot_description_finder_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  ros::param::set("hand/mapping/1", "right");
  ros::param::set("hand/joint_prefix/1", "rh_");

  const string joint_names[] =
  {
    "FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
    "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"
  };

  shadow_robot::SrHandFinder hand_finder;
  // hand parameters
  ASSERT_GT(hand_finder.get_hand_parameters().mapping_.size(), 0);
  ASSERT_GT(hand_finder.get_hand_parameters().joint_prefix_.size(), 0);
  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_["1"], "right");
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_["1"], "rh_");

  // hand joints
  ASSERT_EQ(hand_finder.get_joints().size(), 1);
  ASSERT_GT(hand_finder.get_joints().count("right"), 0);
  const vector<string> rh_joints = hand_finder.get_joints().at("right");
  ASSERT_EQ(rh_joints.size(), 24);
  for (size_t i = 0; i < rh_joints.size(); ++i)
  {
    ROS_DEBUG_STREAM(rh_joints[i]);
    ASSERT_STREQ(rh_joints[i].c_str(), ("rh_" + joint_names[i]).c_str());
  }
}

TEST(SrHandFinder, one_hand_no_mapping_no_robot_description_finder_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  ros::param::set("hand/mapping/1", "");
  ros::param::set("hand/joint_prefix/1", "rh_");

  const string joint_names[] =
  {
    "FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
    "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"
  };

  shadow_robot::SrHandFinder hand_finder;
  // hand parameters
  ASSERT_GT(hand_finder.get_hand_parameters().mapping_.size(), 0);
  ASSERT_GT(hand_finder.get_hand_parameters().joint_prefix_.size(), 0);
  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_["1"], "1");
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_["1"], "rh_");

  // hand joints
  ASSERT_EQ(hand_finder.get_joints().size(), 1);
  ASSERT_GT(hand_finder.get_joints().count("1"), 0);
  const vector<string> rh_joints = hand_finder.get_joints().at("1");
  ASSERT_EQ(rh_joints.size(), 24);
  for (size_t i = 0; i < rh_joints.size(); ++i)
  {
    ROS_DEBUG_STREAM(rh_joints[i]);
    ASSERT_STREQ(rh_joints[i].c_str(), ("rh_" + joint_names[i]).c_str());
  }
}

TEST(SrHandFinder, one_hand_no_prefix_no_robot_description_finder_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  ros::param::set("hand/mapping/1", "rh");
  ros::param::set("hand/joint_prefix/1", "");

  const string joint_names[] =
  {
    "FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
    "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"
  };

  shadow_robot::SrHandFinder hand_finder;
  // hand parameters
  ASSERT_GT(hand_finder.get_hand_parameters().mapping_.size(), 0);
  ASSERT_GT(hand_finder.get_hand_parameters().joint_prefix_.size(), 0);
  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_["1"], "rh");
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_["1"], "");

  // hand joints
  ASSERT_EQ(hand_finder.get_joints().size(), 1);
  ASSERT_GT(hand_finder.get_joints().count("rh"), 0);
  const vector<string> rh_joints = hand_finder.get_joints().at("rh");
  ASSERT_EQ(rh_joints.size(), 24);
  for (size_t i = 0; i < rh_joints.size(); ++i)
  {
    ROS_DEBUG_STREAM(rh_joints[i]);
    ASSERT_STREQ(rh_joints[i].c_str(), (joint_names[i]).c_str());
  }
}

TEST(SrHandFinder, one_hand_no_mapping_no_prefix_no_robot_description_finder_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  ros::param::set("hand/mapping/1", "");
  ros::param::set("hand/joint_prefix/1", "");

  const string joint_names[] =
  {
    "FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
    "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"
  };

  shadow_robot::SrHandFinder hand_finder;
  // hand parameters
  ASSERT_GT(hand_finder.get_hand_parameters().mapping_.size(), 0);
  ASSERT_GT(hand_finder.get_hand_parameters().joint_prefix_.size(), 0);
  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_["1"], "1");
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_["1"], "");

  // hand joints
  ASSERT_EQ(hand_finder.get_joints().size(), 1);
  ASSERT_GT(hand_finder.get_joints().count("1"), 0);
  const vector<string> rh_joints = hand_finder.get_joints().at("1");
  ASSERT_EQ(rh_joints.size(), 24);
  for (size_t i = 0; i < rh_joints.size(); ++i)
  {
    ROS_DEBUG_STREAM(rh_joints[i]);
    ASSERT_STREQ(rh_joints[i].c_str(), (joint_names[i]).c_str());
  }
}

TEST(SrHandFinder, one_hand_robot_description_exists_finger_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  string right_hand_description;
  ros::param::get("right_hand_description", right_hand_description);
  ros::param::set("robot_description", right_hand_description);

  ros::param::set("hand/mapping/1", "right");
  ros::param::set("hand/joint_prefix/1", "rh_");

  const string joint_names[] =
  {
    "FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
    "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"
  };

  shadow_robot::SrHandFinder hand_finder;
  // hand parameters
  ASSERT_GT(hand_finder.get_hand_parameters().mapping_.size(), 0);
  ASSERT_GT(hand_finder.get_hand_parameters().joint_prefix_.size(), 0);
  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_["1"], "right");
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_["1"], "rh_");

  // hand joints
  ASSERT_EQ(hand_finder.get_joints().size(), 1);
  ASSERT_GT(hand_finder.get_joints().count("right"), 0);
  const vector<string> rh_joints = hand_finder.get_joints().at("right");
  ASSERT_EQ(rh_joints.size(), 1);  // only RFJ4 is there
  ASSERT_EQ(std::find(rh_joints.begin(), rh_joints.end(), "rh_FFJ3"), rh_joints.end());
  ASSERT_NE(std::find(rh_joints.begin(), rh_joints.end(), "rh_RFJ4"), rh_joints.end());
}

TEST(SrHandFinder, two_hand_robot_description_exists_finder_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  ros::param::set("hand/mapping/1", "right");
  ros::param::set("hand/joint_prefix/1", "rh_");
  ros::param::set("hand/mapping/2", "left");
  ros::param::set("hand/joint_prefix/2", "lh_");

  string two_hands_description;
  ros::param::get("two_hands_description", two_hands_description);
  ros::param::set("robot_description", two_hands_description);

  const string joint_names[] =
  {
    "FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
    "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"
  };

  unsigned idx = 0;
  const string dir[] =  {"left", "right"};

  shadow_robot::SrHandFinder hand_finder;

  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_.size(), 2);
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_.size(), 2);
  ASSERT_EQ(hand_finder.get_joints().size(), 2);

  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().mapping_, "right");
  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().mapping_, "left");

  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().joint_prefix_, "rh_");
  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().joint_prefix_, "lh_");

  ASSERT_GT(hand_finder.get_joints().count("right"), 0);
  const vector<string> rh_joints = hand_finder.get_joints().at("right");
  ASSERT_EQ(std::find(rh_joints.begin(), rh_joints.end(), "rh_FFJ3"), rh_joints.end());
  ASSERT_NE(std::find(rh_joints.begin(), rh_joints.end(), "rh_RFJ4"), rh_joints.end());

  ASSERT_GT(hand_finder.get_joints().count("left"), 0);
  const vector<string> lh_joints = hand_finder.get_joints().at("left");
  ASSERT_EQ(std::find(lh_joints.begin(), lh_joints.end(), "lh_FFJ1"), lh_joints.end());
  ASSERT_NE(std::find(lh_joints.begin(), lh_joints.end(), "lh_LFJ4"), lh_joints.end());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_hand_finder");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

