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

TEST(SrArmFinder, failure_test)
{
  ASSERT_TRUE(false);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_arm_finder");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

