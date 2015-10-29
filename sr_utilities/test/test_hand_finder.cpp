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
  map<string, string> calibration_path(hand_finder.get_calibration_path());
  ASSERT_EQ(calibration_path.size(), 0);
  shadow_robot::HandControllerTuning controller_tuning(hand_finder.get_hand_controller_tuning());
  ASSERT_EQ(controller_tuning.friction_compensation_.size(), 0);
  ASSERT_EQ(controller_tuning.host_control_.size(), 0);
  ASSERT_EQ(controller_tuning.motor_control_.size(), 0);
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

  ros::param::set("hand/mapping/1", "rh");
  ros::param::set("hand/joint_prefix/1", "rh_");

  shadow_robot::SrHandFinder hand_finder;

  ASSERT_GT(hand_finder.get_hand_parameters().mapping_.size(), 0);
  ASSERT_GT(hand_finder.get_hand_parameters().joint_prefix_.size(), 0);
  ASSERT_GT(hand_finder.get_joints().size(), 0);
  ASSERT_GT(hand_finder.get_calibration_path().size(), 0);

  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_["1"], "rh");
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_["1"], "rh_");

  ASSERT_EQ(hand_finder.get_joints().size(), 1);
  ASSERT_GT(hand_finder.get_joints().count("rh"), 0);

  const vector<string> rh_joints = hand_finder.get_joints().at("rh");
  ASSERT_NE(std::find(rh_joints.begin(), rh_joints.end(), "rh_FFJ3"), rh_joints.end());
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

  ros::param::set("hand/mapping/1", "rh");
  ros::param::set("hand/joint_prefix/1", "rh_");
  ros::param::set("hand/mapping/2", "lh");
  ros::param::set("hand/joint_prefix/2", "lh_");

  string two_hands_description;
  ros::param::get("two_hands_description", two_hands_description);
  ros::param::set("robot_description", two_hands_description);

  shadow_robot::SrHandFinder hand_finder;

  ASSERT_EQ(hand_finder.get_hand_parameters().mapping_.size(), 2);
  ASSERT_EQ(hand_finder.get_hand_parameters().joint_prefix_.size(), 2);
  ASSERT_EQ(hand_finder.get_joints().size(), 2);
  ASSERT_GT(hand_finder.get_calibration_path().size(), 0);

  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().mapping_, "rh");
  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().mapping_, "lh");

  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().joint_prefix_, "rh_");
  ASSERT_MAP_HAS_VALUE(hand_finder.get_hand_parameters().joint_prefix_, "lh_");

  ASSERT_GT(hand_finder.get_joints().count("rh"), 0);
  const vector<string> rh_joints = hand_finder.get_joints().at("rh");
  ASSERT_EQ(std::find(rh_joints.begin(), rh_joints.end(), "rh_FFJ3"), rh_joints.end());
  ASSERT_NE(std::find(rh_joints.begin(), rh_joints.end(), "rh_RFJ4"), rh_joints.end());

  ASSERT_GT(hand_finder.get_joints().count("lh"), 0);
  const vector<string> lh_joints = hand_finder.get_joints().at("lh");
  ASSERT_EQ(std::find(lh_joints.begin(), lh_joints.end(), "lh_FFJ1"), lh_joints.end());
  ASSERT_NE(std::find(lh_joints.begin(), lh_joints.end(), "lh_LFJ4"), lh_joints.end());
}

TEST(SrHandFinder, hand_present_test)
{
  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  ros::NodeHandle nh;
  nh.setParam("hand/mapping/1", "rh");
  nh.setParam("hand/joint_prefix/1", "rh_");
  const string joint_names[] = {"FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
                                "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
                                "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"};
  shadow_robot::SrHandFinder hand_finder;
  map<string, vector<string> > hand_joints(hand_finder.get_joints());
  for (map<string, vector<string> >::const_iterator iter = hand_joints.begin(); iter != hand_joints.end(); ++iter)
  {
    for (size_t i = 0; i != iter->second.size(); ++i)
    {
      ROS_DEBUG_STREAM(iter->second[i]);
      ASSERT_STREQ(iter->second[i].c_str(), ("rh_" + joint_names[i]).c_str());
    }
  }
  string ethercat_path = ros::package::getPath("sr_ethercat_hand_config");
  map<string, string> calibration_path(hand_finder.get_calibration_path());
  for (map<string, string>::const_iterator iter = calibration_path.begin(); iter != calibration_path.end(); ++iter)
  {
    ROS_INFO_STREAM(iter->second);
    ASSERT_STREQ(iter->second.c_str(), (ethercat_path + "/calibrations/rh/" + "calibration.yaml").c_str());
  }
  shadow_robot::HandControllerTuning controller_tuning(hand_finder.get_hand_controller_tuning());
  for (map<string, string>::const_iterator iter = controller_tuning.friction_compensation_.begin();
       iter != controller_tuning.friction_compensation_.end(); ++iter)
  {
    ASSERT_STREQ(iter->second.c_str(), (ethercat_path + "/controls/friction_compensation.yaml").c_str());
    ROS_DEBUG_STREAM(iter->second);
  }
  for (map<string, string>::const_iterator iter = controller_tuning.motor_control_.begin();
       iter != controller_tuning.motor_control_.end(); ++iter)
  {
    ASSERT_STREQ(iter->second.c_str(),
                 (ethercat_path + "/controls/motors/rh/motor_board_effort_controllers.yaml").c_str());
    ROS_DEBUG_STREAM(iter->second);
  }
  for (map<string, vector<string> >::const_iterator iter = controller_tuning.host_control_.begin();
       iter != controller_tuning.host_control_.end(); ++iter)
  {
    string host_array[] = {"sr_edc_calibration_controllers.yaml",
                           "sr_edc_joint_velocity_controllers_PWM.yaml",
                           "sr_edc_effort_controllers_PWM.yaml",
                           "sr_edc_joint_velocity_controllers.yaml",
                           "sr_edc_effort_controllers.yaml",
                           "sr_edc_mixed_position_velocity_joint_controllers_PWM.yaml",
                           "sr_edc_joint_position_controllers_PWM.yaml",
                           "sr_edc_mixed_position_velocity_joint_controllers.yaml",
                           "sr_edc_joint_position_controllers.yaml"};
    vector<string> host_controller_files(host_array, host_array + 9);
    ASSERT_EQ(host_controller_files.size(), iter->second.size());
    for (size_t i = 0; i != iter->second.size(); ++i)
    {
      ASSERT_STREQ(iter->second[i].c_str(), (ethercat_path + "/controls/host/rh/" + host_controller_files[i]).c_str());
      ROS_DEBUG_STREAM(iter->second[i]);
    }
  }
  ASSERT_TRUE(true);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_hand_finder");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

