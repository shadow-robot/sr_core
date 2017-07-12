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
from urdf_parser_py.urdf import URDF
from hand_finder import HandJoints


class ArmConfig(object):

    def __init__(self, mapping, joint_prefix):
        """
        Initializes arm configuration
        """
        self.mapping = mapping
        self.joint_prefix = joint_prefix


class ArmFinder(object):
    """
    The ArmFinder is a utility library for detecting arms running on
    the system. The idea is to make it easier to write generic code,
     using this library to handle prefixes, joint prefixes etc...
    """

    def __init__(self):
        """
        Parses the parameter server to extract the necessary information.
        """
        if not rospy.has_param("arm"):
            rospy.logerr("No arm param defined.")
            arm_parameters = {'joint_prefix': {}, 'mapping': {}}
        else:
            arm_parameters = rospy.get_param("arm")

        # TODO(@anyone): This parameter is never set. This script needs to be modified to find available
        # arms from robot_description and present them appropriately. (sr_core issue #74)

        self.arm_config = ArmConfig(arm_parameters["mapping"], arm_parameters["joint_prefix"])
        self.arm_joints = {}

        if rospy.has_param('robot_description'):

            robot_description = rospy.get_param('robot_description')
            robot_urdf = URDF.from_xml_string(robot_description)

            hand_default_joints = HandJoints.get_default_joints()
            possible_arm_joints = []

            for joint in robot_urdf.joints:
                if joint.type != 'fixed' and joint.name not in hand_default_joints:
                    match_suffix = False
                    for hand_default_joint_name in hand_default_joints:
                        if joint.name.endswith('_' + hand_default_joint_name):
                            match_suffix = True
                            break

                    if not match_suffix:
                        possible_arm_joints.append(joint.name)

            for arm_id, arm_mapping in self.arm_config.mapping.iteritems():
                self.arm_joints[arm_mapping] = []
                arm_joint_prefix = self.arm_config.joint_prefix[arm_id]
                for joint_name in possible_arm_joints:
                    if joint_name.startswith(arm_joint_prefix):
                        self.arm_joints[arm_mapping].append(joint_name)
        else:
            rospy.logwarn("No robot_description found on parameter server. Assuming that there is no arm.")

    def get_arm_joints(self):
        return self.arm_joints

    def get_arm_parameters(self):
        return self.arm_config
