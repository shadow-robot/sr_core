/*
* Copyright 2010 Shadow Robot Company Ltd.
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
 * @file   valves.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Feb 9 13:56:10 2010
 *
 * @brief This is a ROS Interface used to directly access the valves 
 * on Shadow Robot's muscle robots.
 *
 */

#ifndef SR_HAND_HAND_VALVES_H
#define SR_HAND_HAND_VALVES_H

#include <robot/config.h>
#include <robot/robot.h>
#include <robot/hand.h>
#include <robot/hand_protocol.h>
#include <ros/ros.h>
#include <vector>

#include <std_msgs/Float64.h>

namespace shadowrobot
{
class Valves
{
public:
  /**
   * Initializes the necessary mappings with a static list of names.
   */
  Valves();

  /// destructor
  ~Valves();

  void publish();

private:
  void valve_command(const std_msgs::Float64ConstPtr &msg, int index_valve);

  std::vector<struct sensor> valves_sensors;

  std::vector <ros::Publisher> valves_publishers;
  std::vector <ros::Subscriber> valves_subscribers;

  ros::NodeHandle n_tilde;
  ros::Rate publish_rate;

  void init_subs_and_pubs(int index_joint);
};

}  // namespace shadowrobot

#endif  // SR_HAND_HAND_VALVES_H

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

