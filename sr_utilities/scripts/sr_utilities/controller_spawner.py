#!/usr/bin/env python3

# Copyright 2020, 2022, 2023 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from __future__ import absolute_import
import os
import re
import sys
import rospkg
import rospy
import yaml
from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController, SwitchControllerRequest)
from sr_utilities.hand_finder import HandFinder


class ControllerSpawner:
    JOINT_NAMES = ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
                   "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", "rh_THJ1",
                   "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5", "rh_WRJ1", "rh_WRJ2",
                   "lh_FFJ1", "lh_FFJ2", "lh_FFJ3", "lh_FFJ4", "lh_LFJ1", "lh_LFJ2", "lh_LFJ3", "lh_LFJ4", "lh_LFJ5",
                   "lh_MFJ1", "lh_MFJ2", "lh_MFJ3", "lh_MFJ4", "lh_RFJ1", "lh_RFJ2", "lh_RFJ3", "lh_RFJ4", "lh_THJ1",
                   "lh_THJ2", "lh_THJ3", "lh_THJ4", "lh_THJ5", "lh_WRJ1", "lh_WRJ2"]

    def __init__(self, config_file_path, service_timeout, excluded_joints=None):
        if excluded_joints is None:
            excluded_joints = []
        self._config_file_path = config_file_path
        self._service_timeout = service_timeout
        hand_finder = HandFinder()
        self._config = {}
        self._controller_groups = {}
        self._all_controllers = []
        self._hand_mapping = hand_finder.get_hand_parameters().mapping
        self._joints = hand_finder.get_hand_joints()
        self._nonpresent_joints = list(ControllerSpawner.JOINT_NAMES)
        for side in self._joints:
            for joint in self._joints[side]:
                if joint in self._nonpresent_joints:
                    self._nonpresent_joints.remove(joint)
        self._excluded_joints = excluded_joints
        self._excluded_joints = list(set(self._excluded_joints) | set(self._nonpresent_joints))
        rospy.logwarn("Excluded joints:")
        rospy.logwarn(self._excluded_joints)

    def load_config(self):
        try:
            with open(self._config_file_path, 'r', encoding="utf-8") as config_yaml:
                self._config = yaml.safe_load(config_yaml)
        except EnvironmentError as error:
            rospy.logerr(f"Failed to load controller spawner configuration from '{self._config_file_path}'")
            rospy.logerr(error)
            return False
        if not self.load_controller_configs():
            return False
        if not self.parse_controllers():
            return False
        return True

    def load_controller_configs(self):
        success = True
        if "controller_configs" in list(self._config.keys()):
            for side in self._config["controller_configs"]:
                if side not in self._joints:
                    continue
                side_config = self._config["controller_configs"][side]
                rospy.logwarn(side_config)
                for controller in side_config:
                    try:
                        resolved_config_path = self.resolve_path(side_config[controller],
                                                                 local_path=os.path.dirname(self._config_file_path))
                        with open(resolved_config_path, encoding="utf-8") as controller_config_yaml:
                            controller_config = yaml.safe_load(controller_config_yaml)
                            ControllerSpawner.remove_joints(controller_config, self._excluded_joints)
                            for key in controller_config:
                                rospy.set_param(key, controller_config[key])
                    except EnvironmentError as error:
                        rospy.logerr(f"Failed to load {controller} controller configuration from "
                                     f"'{self._config_file_path}'")
                        rospy.logerr(error)
                        rospy.logerr(f"This path is defined in {self._config_file_path}")
                        success = False
        return success

    def resolve_path(self, path, joint_name=None, local_path=None):
        path = self.resolve_string(path, joint_name=joint_name)
        matches = re.findall(r'%rospack_find_(.+)%', path)
        if matches:
            package_name = matches[0]
            ros_pack_ = rospkg.RosPack()
            try:
                package_path = ros_pack_.get_path(package_name)
                path = re.sub(r'%rospack_find_(.+)%', package_path, path)
            except rospkg.common.ResourceNotFound as error:
                rospy.logerr(f"Package '{package_name}' in controller spawner config doesn't exist.")
                rospy.logerr(error)
        if path.startswith('/'):
            return path
        if local_path is None:
            return path
        return f"{local_path}/{path}"

    @staticmethod
    def resolve_string(string, joint_name=None):
        if joint_name is not None:
            string = re.sub(r'%joint_name%', joint_name, string)
        return string

    def parse_controllers(self):
        if "controller_groups" not in list(self._config.keys()):
            rospy.logwarn(f"No controller groups specified in controller spawner config ({self._config_file_path})")
            return False
        for controller_group_name in self._config["controller_groups"]:  # pylint: disable=R1702
            controller_group_ = []
            for side in self._config["controller_groups"][controller_group_name]:
                side_controllers = self._config["controller_groups"][controller_group_name][side]
                if side not in self._joints:
                    continue
                necessary_if_joint_present = []
                if "necessary_if_joint_present" in side_controllers:
                    necessary_if_joint_present = side_controllers["necessary_if_joint_present"]
                for joint_name in self._joints[side]:
                    if "common" in side_controllers:
                        for controller_raw in side_controllers["common"]:
                            controller = self.resolve_string(controller_raw, joint_name=joint_name.lower())
                            if (joint_name not in self._excluded_joints) or (controller in necessary_if_joint_present):
                                if controller not in controller_group_:
                                    controller_group_.append(controller)
                            if controller not in self._all_controllers:
                                self._all_controllers.append(controller)
                    if joint_name in side_controllers:
                        for controller_raw in side_controllers[joint_name]:
                            controller = self.resolve_string(controller_raw, joint_name=joint_name.lower())
                            if (joint_name not in self._excluded_joints) or (controller in necessary_if_joint_present):
                                if controller not in controller_group_:
                                    controller_group_.append(controller)
                            if controller not in self._all_controllers:
                                self._all_controllers.append(controller)
                    elif "default" in side_controllers:
                        for controller_raw in side_controllers["default"]:
                            controller = self.resolve_string(controller_raw, joint_name=joint_name.lower())
                            if (joint_name not in self._excluded_joints) or (controller in necessary_if_joint_present):
                                if controller not in controller_group_:
                                    controller_group_.append(controller)
                            if controller not in self._all_controllers:
                                self._all_controllers.append(controller)
            if controller_group_:
                self._controller_groups[controller_group_name] = controller_group_
        ControllerSpawner.remove_joints(self._controller_groups, self._nonpresent_joints)
        rospy.set_param("controller_groups", self._controller_groups)
        return True

    def switch_controllers(self, controller_group_name):
        if controller_group_name not in self._controller_groups:
            rospy.logerr(f"There is no controller group named '{controller_group_name}'")
            return False
        desired_controllers = self._controller_groups[controller_group_name]
        try:
            rospy.wait_for_service('controller_manager/list_controllers', self._service_timeout)
            list_controllers = rospy.ServiceProxy(
                'controller_manager/list_controllers', ListControllers)
            current_controllers = list_controllers().controller
        except rospy.ServiceException:
            success = False
            rospy.logerr("Failed to get currently running controllers.")
            return False
        loaded_controllers = [controller.name for controller in current_controllers]
        running_controllers = [controller.name for controller in current_controllers if controller.state == "running"]
        controllers_to_stop = [controller for controller in running_controllers if
                               controller in self._all_controllers and controller not in desired_controllers]
        controllers_to_load = [controller for controller in desired_controllers if controller not in loaded_controllers]
        controllers_to_start = [controller for controller in desired_controllers if
                                controller not in running_controllers]
        success = True

        # Load any desired controllers that were not already loaded
        for controller in controllers_to_load:
            try:
                rospy.wait_for_service('controller_manager/load_controller', self._service_timeout)
                load_controllers = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
                load_success = load_controllers(controller).ok
                if not load_success:
                    rospy.logerr(f"Failed to load controller '{controller}'")
                    success = False
            except rospy.ServiceException as error:
                rospy.logerr(f"Failed to load controller '{controller}'")
                rospy.logerr(error)
                success = False

        # Start any desired controllers that were not already running
        try:
            rospy.wait_for_service('controller_manager/switch_controller', self._service_timeout)
            switch_controllers = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
            start_success = switch_controllers(controllers_to_start, controllers_to_stop,
                                               SwitchControllerRequest.BEST_EFFORT, False, 0).ok
            if not start_success:
                rospy.logerr(f"Failed to stop controllers {controllers_to_stop} and/or "
                             f"start controllers {controllers_to_start}")
                success = False
        except rospy.ServiceException as error:
            rospy.logerr(f"Failed to stop controllers {controllers_to_stop} and/or "
                         f"start controllers {controllers_to_start}")
            rospy.logerr(error)
            success = False
        if success:
            rospy.loginfo("Controllers spawned successfully.")
        else:
            rospy.logerr("There was an error spawning controllers")
        return success

    @staticmethod
    def remove_joints(config, joints=None):
        if joints is None:
            joints = []
        joints_lower = [joint.lower() for joint in joints]
        for key in list(config.keys()):
            if key.lower() in joints_lower:
                del config[key]
                continue
            if isinstance(config[key], dict):
                ControllerSpawner.remove_joints(config[key], joints)
            if isinstance(config[key], list):
                for joint in joints_lower:
                    match_indices = [i for i, item in enumerate(config[key]) if joint in item.lower()]
                    for index in sorted(match_indices, reverse=True):
                        del config[key][index]


if __name__ == "__main__":
    rospy.init_node("sr_controller_spawner")
    ros_pack = rospkg.RosPack()
    sr_robot_launch_path = ros_pack.get_path('sr_robot_launch')
    wait_for_topic = rospy.get_param("~wait_for", "")
    controller_group = rospy.get_param("~controller_group", "trajectory")
    config_file_path_param = rospy.get_param("~config_file_path",
                                             f"{sr_robot_launch_path}/config/controller_spawner.yaml")
    service_timeout_param = rospy.get_param("~service_timeout", 60.0)
    excluded_joints_param = rospy.get_param("~excluded_joints", [])
    controller_spawner = ControllerSpawner(config_file_path_param, service_timeout_param, excluded_joints_param)
    if not controller_spawner.load_config():
        rospy.logerr("Failed to load controller spawner config.")
        sys.exit(1)
    if rospy.has_param("~wait_for"):
        rospy.loginfo("Shadow controller spawner is waiting for topic '{}'...".format(rospy.get_param("~wait_for")))
        rospy.wait_for_message(rospy.get_param("~wait_for"), rospy.AnyMsg)
    controller_spawner.switch_controllers(controller_group)
