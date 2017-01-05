/**
 * @file   hand_commander_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
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
 *
 */

// We are testing this
#include "sr_hand/hand_commander.hpp"

#include <utility>
#include <string>
#include <vector>

// ROS
#include "ros/ros.h"

// Gtest
#include <gtest/gtest.h>

using shadowrobot::HandCommander;

TEST(HandCommander, constructor)
{
  HandCommander handcmd = HandCommander();
  EXPECT_TRUE(true);
}

TEST(HandCommander, topic_state)
{
  HandCommander handcmd = HandCommander();

  // get_controller_state_topic returns fully resolved topics
  ros::NodeHandle nh;

  std::string topic = handcmd.get_controller_state_topic("rh_FFJ3");
  EXPECT_EQ(nh.resolveName("sh_rh_ffj3_position_controller/state"), topic);

  topic = handcmd.get_controller_state_topic("rh_FFJ0");
  EXPECT_EQ(nh.resolveName("sh_rh_ffj0_position_controller/state"), topic);

  topic = handcmd.get_controller_state_topic("unknown joint");
  EXPECT_EQ("", topic);
}

TEST(HandCommander, min_max)
{
  HandCommander handcmd = HandCommander();

  std::pair<double, double> min_max = handcmd.get_min_max("rh_FFJ3");
  EXPECT_DOUBLE_EQ(min_max.first, 0.0);
  EXPECT_DOUBLE_EQ(min_max.second, 1.57079632679);

  // also works for lower case
  min_max = handcmd.get_min_max("rh_FFJ3");
  EXPECT_DOUBLE_EQ(min_max.first, 0.0);
  EXPECT_DOUBLE_EQ(min_max.second, 1.57079632679);

  // j0 should be 0, 180
  min_max = handcmd.get_min_max("rh_FFJ0");
  EXPECT_DOUBLE_EQ(min_max.first, 0.0);
  EXPECT_DOUBLE_EQ(min_max.second, 3.14159265358);

  // returns 0, 0 if joint not found
  min_max = handcmd.get_min_max("unknown joint");
  EXPECT_DOUBLE_EQ(min_max.first, 0.0);
  EXPECT_DOUBLE_EQ(min_max.second, 0.0);

  // Check that we can get the min and max for each joint
  std::vector<std::string> all_joints = handcmd.get_all_joints();
  for (size_t i = 0; i < all_joints.size(); ++i)
  {
    min_max = handcmd.get_min_max(all_joints[i]);
    EXPECT_TRUE(min_max.first != min_max.second);  // min = max if joint not found
  }
}

TEST(HandCommander, all_joints)
{
  HandCommander handcmd = HandCommander();

  EXPECT_EQ(handcmd.get_all_joints().size(), 20);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // sleep until gazebo is ready
  sleep(300.0);

  ros::init(argc, argv, "hand_commander_test");
  ros::NodeHandle nh;  // init the node


  return RUN_ALL_TESTS();
}
