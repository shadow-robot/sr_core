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
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, LoadController
from sr_utilities.hand_finder import HandFinder


class TrajectoryControllerSpawner(object):
    def __init__(self):
        self.hand_finder = HandFinder()
        self.joints = self.hand_finder.get_hand_joints()
        ros_pack = rospkg.RosPack()
        sr_robot_launch_path = ros_pack.get_path('sr_robot_launch')
        self.hand_mapping = self.hand_finder.get_hand_parameters().mapping
        self.yaml_file_path = {}
        for hand in self.hand_mapping:
            self.yaml_file_path[self.hand_mapping[hand]] = (
                sr_robot_launch_path + "/config/" + self.hand_mapping[hand] + "_trajectory_controller.yaml")

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

    def set_controller(self):
        for hand_serial in self.hand_mapping:
            hand_prefix = self.hand_mapping[hand_serial]
            success = True
            list_controllers = rospy.ServiceProxy(
                'controller_manager/list_controllers', ListControllers)
            try:
                running_controllers = list_controllers()
            except rospy.ServiceException:
                success = False
                rospy.logerr("Failed to load trajectory controller")
            if success:
                controllers_to_start = []
                already_running = False
                for controller_state in running_controllers.controller:
                    if controller_state.name == hand_prefix + '_trajectory_controller':
                        already_running = True
                if not already_running:
                    controllers_to_start.append(hand_prefix + '_trajectory_controller')

        for load_control in controllers_to_start:
            try:
                load_controllers = rospy.ServiceProxy(
                    'controller_manager/load_controller',
                    LoadController)
                loaded_controllers = load_controllers(load_control)
            except rospy.ServiceException:
                success = False
            if not loaded_controllers.ok:
                success = False

        switch_controllers = rospy.ServiceProxy(
            'controller_manager/switch_controller', SwitchController)
        try:
            switched_controllers = switch_controllers(
                controllers_to_start, None,
                SwitchController._request_class.BEST_EFFORT)
        except rospy.ServiceException:
            success = False

        if not switched_controllers.ok:
            success = False

        if not success:
            rospy.logerr(
                "Failed to launch trajectory controller!")


if __name__ == "__main__":
    rospy.init_node("generate_trajectory_controller_parameters")
    trajectory_spawner = TrajectoryControllerSpawner()
    trajectory_spawner.generate_parameters()
    trajectory_spawner.set_controller()
