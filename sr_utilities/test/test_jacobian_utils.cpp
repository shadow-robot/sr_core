#include <gtest/gtest.h>
#include "ros/ros.h"
#include "sr_utilities/jacobian_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void wait_for_param(std::string param_name, double timeout_in_secs=30.0)
{
  ros::NodeHandle nh;
  ros::Time start_time = ros::Time::now();
  while (ros::Time::now().toSec() - start_time.toSec() < timeout_in_secs)
  if (nh.hasParam(param_name))
  {
    return;
  }
}

TEST(SrJacobianUtils, test_get_torque_given_wrench)
{
  wait_for_param("robot_description");
  wait_for_param("robot_description_semantic");
  shadow_robot::SrJacobianUtils ju("robot_description", "rh_first_finger");
  geometry_msgs::WrenchStamped desired_wrench;
  desired_wrench.wrench.force.x = 10;
  desired_wrench.wrench.force.y = 0;
  desired_wrench.wrench.force.z = 0;
  desired_wrench.wrench.torque.x = 0;
  desired_wrench.wrench.torque.y = 0;
  desired_wrench.wrench.torque.z = 0;
  desired_wrench.header.frame_id = "rh_fftip";
  Eigen::VectorXd needed_torques = ju.get_torques_given_wrench(desired_wrench);
  ASSERT_NEAR(needed_torques[0], -0.96, 0.001);
  ASSERT_NEAR(needed_torques[1], 0, 0.001);
  ASSERT_NEAR(needed_torques[2], 0, 0.001);
  ASSERT_NEAR(needed_torques[3], 0, 0.001);
}

TEST(SrJacobianUtils, test_get_jacobian)
{
  wait_for_param("robot_description");
  wait_for_param("robot_description_semantic");
  shadow_robot::SrJacobianUtils ju("robot_description", "rh_first_finger");
  Eigen::MatrixXd known_jacobian(6,4);
  known_jacobian << -0.095993, 0, 0, 0,
                    0, -0.0959993, -0.0509995, -0.0259998,
                    0, 0, 0, 0,
                    0, 0.999935, 0.999935, 0.999935,
                    -1, 0, 0, 0,
                    0, -0.0114267, -0.0114267, -0.0114267;

  Eigen::MatrixXd tested_jacobian(6,4);
  tested_jacobian = ju.get_jacobian();

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      ASSERT_NEAR(known_jacobian(i, j), tested_jacobian(i, j), 0.1);
    }
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_jacobian_utils");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}