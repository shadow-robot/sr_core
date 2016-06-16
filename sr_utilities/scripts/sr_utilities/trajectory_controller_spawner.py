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
from std_msgs.msg import Bool


class TrajectoryControllerSpawner(object):
    def __init__(self, trajectory, service_timeout):
        self.trajectory = trajectory
        self.service_timeout = service_timeout
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
                rospy.logerr("there are errors opening trajectory controller yaml file")
            else:
                hand_trajectory = yaml_content[hand + "_trajectory_controller"]
                if rospy.has_param('~exclude_wrist') and rospy.get_param('~exclude_wrist'):
                    hand_trajectory['joints'] = [s for s in self.joints[hand] if "WR" not in s]
                else:
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

    @staticmethod
    def check_joint(joint, controllers_to_start, controller_names):
        if joint[3:5].lower() == 'th' or joint[3:5].lower() == 'wr' or (joint[6] != '1' and joint[6] != '2'):
            joint_controller = 'sh_' + joint.lower() + "_position_controller"
        else:
            joint = joint[:6] + '0'
            joint_controller = 'sh_' + joint.lower() + "_position_controller"

        if joint_controller not in controller_names and joint_controller not in controllers_to_start:
            controllers_to_start.append(joint_controller)

    def set_controller(self):
        controllers_to_start = []
        for hand_serial in self.hand_mapping:
            hand_prefix = self.hand_mapping[hand_serial]
            success = True
            try:
                rospy.wait_for_service('controller_manager/list_controllers', self.service_timeout)
                list_controllers = rospy.ServiceProxy(
                    'controller_manager/list_controllers', ListControllers)

                running_controllers = list_controllers()
            except rospy.ServiceException:
                success = False
                rospy.logerr("Failed to load trajectory controller")
            if success:
                already_running = False
                controller_names = []
                for controller_state in running_controllers.controller:
                    controller_names.append(controller_state.name)
                    if controller_state.name == hand_prefix + '_trajectory_controller':
                        already_running = True
                if self.trajectory and not already_running:
                    controllers_to_start.append(hand_prefix + '_trajectory_controller')
                for joint in self.joints[hand_prefix]:
                    TrajectoryControllerSpawner.check_joint(joint, controllers_to_start, controller_names)

        for load_control in controllers_to_start:
            try:
                rospy.wait_for_service('controller_manager/load_controller', self.service_timeout)
                load_controllers = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
                loaded_controllers = load_controllers(load_control)
            except rospy.ServiceException:
                success = False
            if not loaded_controllers.ok:
                success = False

        try:
            rospy.wait_for_service('controller_manager/switch_controller', self.service_timeout)
            switch_controllers = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
            switched_controllers = switch_controllers(controllers_to_start, None,
                                                      SwitchController._request_class.BEST_EFFORT)
        except rospy.ServiceException:
            success = False

        if not switched_controllers.ok:
            success = False

        if not success:
            rospy.logerr("Failed to launch trajectory controller!")

    @staticmethod
    def wait_for_topic(topic_name, timeout):
        if not topic_name:
            return True

        # This has to be a list since Python has a peculiar mechanism to determine
        # whether a variable is local to a function or not:
        # if the variable is assigned in the body of the function, then it is
        # assumed to be local. Modifying a mutable object (like a list)
        # works around this debatable "design choice".
        wait_for_topic_result = [None]

        def wait_for_topic_cb(msg):
            wait_for_topic_result[0] = msg
            rospy.logdebug("Heard from wait-for topic: %s" % str(msg.data))
        rospy.Subscriber(topic_name, Bool, wait_for_topic_cb)
        started_waiting = rospy.Time.now().to_sec()

        # We might not have received any time messages yet
        warned_about_not_hearing_anything = False
        while not wait_for_topic_result[0]:
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                return False
            if not warned_about_not_hearing_anything:
                if rospy.Time.now().to_sec() - started_waiting > timeout:
                    warned_about_not_hearing_anything = True
                    rospy.logwarn("Controller Spawner hasn't heard anything from its \"wait for\" topic (%s)" %
                                  topic_name)
        while not wait_for_topic_result[0].data:
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                return False
        return True


if __name__ == "__main__":
    rospy.init_node("generate_trajectory_controller_parameters")
    wait_for_topic = rospy.get_param("~wait_for", "")
    hand_trajectory = rospy.get_param("~hand_trajectory", False)
    timeout = rospy.get_param("~timeout", 30.0)
    service_timeout = rospy.get_param("~service_timeout", 60.0)

    trajectory_spawner = TrajectoryControllerSpawner(trajectory=hand_trajectory, service_timeout=service_timeout)
    if trajectory_spawner.wait_for_topic(wait_for_topic, timeout):
        trajectory_spawner.generate_parameters()
        trajectory_spawner.set_controller()
