/*
* Copyright 2020 Shadow Robot Company Ltd.
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
  const int torques_vector_dimension = 4;
  const std::vector<double> expected_torques{-0.96, 0, 0, 0};
  geometry_msgs::WrenchStamped desired_wrench;
  desired_wrench.wrench.force.x = 10;
  desired_wrench.wrench.force.y = 0;
  desired_wrench.wrench.force.z = 0;
  desired_wrench.wrench.torque.x = 0;
  desired_wrench.wrench.torque.y = 0;
  desired_wrench.wrench.torque.z = 0;
  desired_wrench.header.frame_id = "rh_fftip";

  wait_for_param("robot_description");
  wait_for_param("robot_description_semantic");
  
  shadow_robot::SrJacobianUtils ju("robot_description", "rh_first_finger");
  Eigen::VectorXd calculated_torques = ju.get_torques_given_wrench(desired_wrench);

  for (int i=0; i<torques_vector_dimension; i++)
  {
    ASSERT_NEAR(calculated_torques[i], expected_torques[i], 0.001);
  }
}

TEST(SrJacobianUtils, test_get_jacobian)
{
  const std::vector<int> jacobian_dimensions{6, 4};
  Eigen::MatrixXd expected_jacobian(jacobian_dimensions[0], jacobian_dimensions[1]);
  expected_jacobian << -0.095993, 0, 0, 0,
                    0, -0.0959993, -0.0509995, -0.0259998,
                    0, 0, 0, 0,
                    0, 0.999935, 0.999935, 0.999935,
                    -1, 0, 0, 0,
                    0, -0.0114267, -0.0114267, -0.0114267;

  Eigen::MatrixXd tested_jacobian(jacobian_dimensions[0], jacobian_dimensions[1]);

  wait_for_param("robot_description");
  wait_for_param("robot_description_semantic");

  shadow_robot::SrJacobianUtils ju("robot_description", "rh_first_finger");
  tested_jacobian = ju.get_jacobian();

  for (int i = 0; i < jacobian_dimensions[0]; i++)
  {
    for (int j = 0; j < jacobian_dimensions[1]; j++)
    {
      ASSERT_NEAR(expected_jacobian(i, j), tested_jacobian(i, j), 0.1);
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