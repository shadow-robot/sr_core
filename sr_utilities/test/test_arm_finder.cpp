/*
 * File:  test_arm_finder.cpp
 * Author: Andriy Petlovanyy <andriy@shadowrobot.com>
 * Copyright:
 *
 * @brief see README.md
 */
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "sr_utilities/sr_arm_finder.hpp"
#include <utility>
#include <string>

TEST(SrArmFinder, no_hand_no_arm_finder_test)
{
  if (ros::param::has("arm"))
  {
    ros::param::del("arm");
  }

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

  shadow_robot::SrArmFinder arm_finder;

  ASSERT_EQ(arm_finder.get_arm_parameters().mapping_.size(), 0);
  ASSERT_EQ(arm_finder.get_arm_parameters().joint_prefix_.size(), 0);
  ASSERT_EQ(arm_finder.get_joints().size(), 0);
}

TEST(SrArmFinder, one_hand_no_arm_finder_test)
{
  if (ros::param::has("arm"))
  {
    ros::param::del("arm");
  }

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

  shadow_robot::SrArmFinder arm_finder;

  ASSERT_EQ(arm_finder.get_arm_parameters().mapping_.size(), 0);
  ASSERT_EQ(arm_finder.get_arm_parameters().joint_prefix_.size(), 0);
  ASSERT_EQ(arm_finder.get_joints().size(), 0);
}

TEST(SrArmFinder, no_hand_one_arm_finder_test)
{
  if (ros::param::has("arm"))
  {
    ros::param::del("arm");
  }

  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  std::string no_hand_one_arm_description;
  ros::param::get("no_hand_one_arm_description", no_hand_one_arm_description);
  ros::param::set("robot_description", no_hand_one_arm_description);

  ros::param::set("arm/mapping/1", "la");
  ros::param::set("arm/joint_prefix/1", "la_");

  shadow_robot::SrArmFinder arm_finder;

  ASSERT_EQ(arm_finder.get_arm_parameters().mapping_.size(), 1);
  ASSERT_EQ(arm_finder.get_arm_parameters().joint_prefix_.size(), 1);
  ASSERT_EQ(arm_finder.get_joints().size(), 1);

  auto mapping = arm_finder.get_arm_parameters().mapping_;
  ASSERT_NE(mapping.end(), std::find_if(mapping.begin(), mapping.end(),
                                        [](const std::pair<std::string, std::string> &item)
                                        {
                                          return item.second == "la";
                                        }));  // NOLINT(whitespace/braces)

  auto joint_prefixes = arm_finder.get_arm_parameters().joint_prefix_;
  ASSERT_NE(joint_prefixes.end(), std::find_if(joint_prefixes.begin(), joint_prefixes.end(),
                                               [](const std::pair<std::string, std::string> &item)
                                               {
                                                 return item.second == "la_";
                                               }));  // NOLINT(whitespace/braces)

  ASSERT_GT(arm_finder.get_joints().count("la"), 0);
}

TEST(SrArmFinder, one_hand_two_arms_finder_test)
{
  if (ros::param::has("arm"))
  {
    ros::param::del("arm");
  }

  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  std::string right_hand_two_arms;
  ros::param::get("right_hand_two_arms", right_hand_two_arms);
  ros::param::set("robot_description", right_hand_two_arms);

  ros::param::set("hand/mapping/1", "rh");
  ros::param::set("hand/joint_prefix/1", "rh_");

  ros::param::set("arm/mapping/1", "la");
  ros::param::set("arm/joint_prefix/1", "la_");

  ros::param::set("arm/mapping/2", "ra");
  ros::param::set("arm/joint_prefix/2", "ra_");

  shadow_robot::SrArmFinder arm_finder;

  ASSERT_EQ(arm_finder.get_arm_parameters().mapping_.size(), 2);
  ASSERT_EQ(arm_finder.get_arm_parameters().joint_prefix_.size(), 2);
  ASSERT_EQ(arm_finder.get_joints().size(), 2);

  auto mapping = arm_finder.get_arm_parameters().mapping_;
  ASSERT_NE(mapping.end(), std::find_if(mapping.begin(), mapping.end(),
                                        [](const std::pair<std::string, std::string> &item)
                                        {
                                          return item.second == "ra";
                                        }));  // NOLINT(whitespace/braces)

  auto joint_prefixes = arm_finder.get_arm_parameters().joint_prefix_;
  ASSERT_NE(joint_prefixes.end(), std::find_if(joint_prefixes.begin(), joint_prefixes.end(),
                                               [](const std::pair<std::string, std::string> &item)
                                               {
                                                 return item.second == "ra_";
                                               }));  // NOLINT(whitespace/braces)

  ASSERT_EQ(arm_finder.get_joints().count("ra"), 1);
  ASSERT_EQ(arm_finder.get_joints()["ra"].size(), 1);

  ASSERT_NE(mapping.end(), std::find_if(mapping.begin(), mapping.end(),
                                        [](const std::pair<std::string, std::string> &item)
                                        {
                                          return item.second == "la";
                                        }));  // NOLINT(whitespace/braces)
  ASSERT_NE(joint_prefixes.end(), std::find_if(joint_prefixes.begin(), joint_prefixes.end(),
                                               [](const std::pair<std::string, std::string> &item)
                                               {
                                                 return item.second == "la_";
                                               }));  // NOLINT(whitespace/braces)
  ASSERT_EQ(arm_finder.get_joints().count("la"), 1);
  ASSERT_EQ(arm_finder.get_joints()["la"].size(), 1);
}

TEST(SrArmFinder, two_hands_one_arm_finder_test)
{
  if (ros::param::has("arm"))
  {
    ros::param::del("arm");
  }

  if (ros::param::has("hand"))
  {
    ros::param::del("hand");
  }

  if (ros::param::has("robot_description"))
  {
    ros::param::del("robot_description");
  }

  std::string two_hands_left_arm;
  ros::param::get("two_hands_left_arm", two_hands_left_arm);
  ros::param::set("robot_description", two_hands_left_arm);

  ros::param::set("hand/mapping/1", "rh");
  ros::param::set("hand/joint_prefix/1", "rh_");

  ros::param::set("hand/mapping/2", "lh");
  ros::param::set("hand/joint_prefix/2", "lh_");

  ros::param::set("arm/mapping/1", "la");
  ros::param::set("arm/joint_prefix/1", "la_");

  shadow_robot::SrArmFinder arm_finder;

  ASSERT_EQ(arm_finder.get_arm_parameters().mapping_.size(), 1);
  ASSERT_EQ(arm_finder.get_arm_parameters().joint_prefix_.size(), 1);
  ASSERT_EQ(arm_finder.get_joints().size(), 1);

  auto mapping = arm_finder.get_arm_parameters().mapping_;
  ASSERT_NE(mapping.end(), std::find_if(mapping.begin(), mapping.end(),
                                        [](const std::pair<std::string, std::string> &item)
                                        {
                                          return item.second == "la";
                                        }));  // NOLINT(whitespace/braces)

  auto joint_prefixes = arm_finder.get_arm_parameters().joint_prefix_;
  ASSERT_NE(joint_prefixes.end(), std::find_if(joint_prefixes.begin(), joint_prefixes.end(),
                                               [](const std::pair<std::string, std::string> &item)
                                               {
                                                 return item.second == "la_";
                                               }));  // NOLINT(whitespace/braces)
  ASSERT_EQ(arm_finder.get_joints().count("la"), 1);
  ASSERT_EQ(arm_finder.get_joints()["la"].size(), 1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_arm_finder");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

