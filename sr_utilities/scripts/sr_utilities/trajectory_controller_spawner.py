#!/usr/bin/env python
#
# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import rospy
import yaml
import rospkg
from sr_utilities.hand_finder import HandFinder

class TrajectoryControllerSpawner(object):
    def __init__(self):
        self.hand_finder = HandFinder()
        self.joints = self.hand_finder.get_hand_joints()
        ros_pack = rospkg.RosPack()
        sr_robot_launch_path = ros_pack.get_path('sr_robot_launch')
        hand_mapping = self.hand_finder.get_hand_parameters().mapping
        self.yaml_file_path = {}
        for hand in hand_mapping:
            self.yaml_file_path[hand_mapping[hand]] = (
                sr_robot_launch_path + "/config/" + hand_mapping[hand] + "_trajectory_controller.yaml")

    def generate_parameters(self):
        for hand in self.yaml_file_path:
            with open(self.yaml_file_path[hand], 'r') as yaml_file:
                yaml_content = yaml.load(yaml_file)
            if hand + "_trajectory_controller" not in yaml_content.keys():
                rospy.logerr("there are errors openning trajectory controller yaml file")
            else:
                hand_trajectory = yaml_content[hand + "_trajectory_controller"]
                hand_trajectory['joints'] = self.joints[hand]
                for joint_name in hand_trajectory['constraints'].keys():
                    if (joint_name not in hand_trajectory['joints'] and
                            joint_name != 'goal_time' and joint_name != 'stopped_velocity_tolerance'):
                        del hand_trajectory['constraints'][joint_name]

                param_prefix = hand + "_trajectory_controller/"
                rospy.set_param(param_prefix + 'allow_partial_joints_goal',
                                hand_trajectory['allow_partial_joints_goal'])
                rospy.set_param(param_prefix + 'joints', hand_trajectory['joints'])
                rospy.set_param(param_prefix + 'stop_trajectory_duration', hand_trajectory['stop_trajectory_duration'])
                rospy.set_param(param_prefix + 'type', hand_trajectory['type'])
                constrain_prefix = param_prefix + 'constraints/'
                for constraint in hand_trajectory['constraints']:
                    if constraint == 'goal_time' or constraint == 'stopped_velocity_tolerance':
                        rospy.set_param(constrain_prefix + constraint, hand_trajectory['constraints'][constraint])
                    else:
                        rospy.set_param(constrain_prefix + constraint + '/goal',
                                        hand_trajectory['constraints'][constraint]['goal'])
                        rospy.set_param(constrain_prefix + constraint + '/trajectory',
                                        hand_trajectory['constraints'][constraint]['trajectory'])


if __name__ == "__main__":
    rospy.init_node("generate_trajectory_controller_parameters")
    trajectory_spawner = TrajectoryControllerSpawner()
    trajectory_spawner.generate_parameters()
