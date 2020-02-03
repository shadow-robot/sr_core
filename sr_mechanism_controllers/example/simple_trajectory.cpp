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

/**
 * @file   simple_trajectory.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * @brief http://ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
 *
 *
 */


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class ShadowTrajectory
{
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient *traj_client_;

public:
  // ! Initialize the action client and wait for action server to come up
  ShadowTrajectory()
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  // ! Clean up the action client
  ~ShadowTrajectory()
  {
    delete traj_client_;
  }

  // ! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  // ! Wait for currently running trajectory to finish
  void waitTrajectory()
  {
    while (!getState().isDone() && ros::ok())
    {
      usleep(50000);
    }
  }

  // ! Generates a simple trajectory to move two fingers on the hand.
  /*! Note that this trajectory contains three waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal fingerWiggleTrajectory()
  {
    // our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("ffj0");
    goal.trajectory.joint_names.push_back("lfj0");

    // Set number of waypoints in this goal trajectory
    goal.trajectory.points.resize(3);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(2);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    // Points also have velocities, but the shadow action server doesn't use them
    // goal.trajectory.points[ind].velocities.resize(2);
    // goal.trajectory.points[ind].velocities[0] = 0.0;
    // goal.trajectory.points[ind].velocities[0] = 0.0;
    // To be reached 1.0 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // 2nd trajectory point
    ind += 1;
    goal.trajectory.points[ind].positions.resize(2);
    goal.trajectory.points[ind].positions[0] = 1.0;
    goal.trajectory.points[ind].positions[1] = 1.0;
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    // 3rd trajectory point
    ind += 1;
    goal.trajectory.points[ind].positions.resize(2);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].time_from_start = ros::Duration(10.0);

    return goal;
  }

  // ! Generates a simple trajectory to move the arm
  control_msgs::FollowJointTrajectoryGoal armWaveTrajectory()
  {
    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("er");
    goal.trajectory.joint_names.push_back("es");

    goal.trajectory.points.resize(3);

    int ind = 0;
    goal.trajectory.points[ind].positions.resize(2);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    ind += 1;
    goal.trajectory.points[ind].positions.resize(2);
    goal.trajectory.points[ind].positions[0] = 1.0;
    goal.trajectory.points[ind].positions[1] = 1.0;
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    ind += 1;
    goal.trajectory.points[ind].positions.resize(2);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].time_from_start = ros::Duration(10.0);

    return goal;
  }

  // ! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
};

int main(int argc, char **argv)
{
  // Init the ROS node
  ros::init(argc, argv, "shadow_trajectory_driver");

  ShadowTrajectory sj;
  sj.startTrajectory(sj.fingerWiggleTrajectory());
  sj.waitTrajectory();

  sj.startTrajectory(sj.armWaveTrajectory());
  sj.waitTrajectory();
}

// vim: sw=2:ts=2
