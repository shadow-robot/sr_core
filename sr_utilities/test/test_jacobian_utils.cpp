#include <gtest/gtest.h>
#include "ros/ros.h"
#include "sr_utilities/jacobian_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TEST(SrJacobianUtils, test_get_torque_given_wrench)
{
  shadow_robot::SrJacobianUtils ju("robot_description", "rh_first_finger");
  geometry_msgs::WrenchStamped desired_wrench_;
  ASSERT_TRUE(ros::param::has("robot_description"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_jacobian_utils");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}