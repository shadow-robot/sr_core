#!/usr/bin/env python3

# Copyright 2014 Shadow Robot Company Ltd.
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
import rospy
from urdf_parser_py.urdf import URDF


class HandConfig:

    def __init__(self, mapping, joint_prefix):
        """

        """
        # handle possibly empty mapping
        self.mapping = {}
        for serial_id in mapping:
            if mapping[serial_id] == '':
                self.mapping[serial_id] = str(serial_id)
            else:
                self.mapping[serial_id] = mapping[serial_id]
        self.joint_prefix = joint_prefix


class HandJoints:
    TIMEOUT_WAIT_FOR_PARAMS_IN_SECS = 60.0

    @classmethod
    def get_default_joints(cls):
        joints = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'MFJ1', 'MFJ2', 'MFJ3',
                  'MFJ4', 'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'LFJ1', 'LFJ2',
                  'LFJ3', 'LFJ4', 'LFJ5', 'THJ1', 'THJ2', 'THJ3', 'THJ4',
                  'THJ5', 'WRJ1', 'WRJ2']
        return joints

    def __init__(self, mapping, joint_prefix):
        """

        """
        self.joints = {}
        hand_joints = []
        joints = self.get_default_joints()
        start_time = rospy.get_time()
        while not rospy.has_param("/robot_description"):
            if rospy.get_time() - start_time > self.TIMEOUT_WAIT_FOR_PARAMS_IN_SECS:
                rospy.logwarn("No robot_description found on parameter server."
                              "Joint names are loaded for 5 finger hand")
                # concatenate all the joints with prefixes
                for hand in mapping:
                    hand_joints = []
                    if hand in joint_prefix:
                        for joint in joints:
                            hand_joints.append(joint_prefix[hand] + joint)
                    else:
                        rospy.logwarn("Cannot find serial " + hand +
                                      "in joint_prefix parameters")
                    self.joints[mapping[hand]] = hand_joints
                return

        robot_description = rospy.get_param('robot_description')

        # concatenate all the joints with prefixes
        for hand in mapping:
            if hand in joint_prefix:
                for joint in joints:
                    hand_joints.append(joint_prefix[hand] + joint)
            else:
                rospy.logwarn("Cannot find serial " + hand +
                              "in joint_prefix parameters")

        # add the prefixed joints to each hand but remove fixed joints
        hand_urdf = URDF.from_xml_string(robot_description)
        for hand in mapping:
            joints_tmp = []
            self.joints[mapping[hand]] = []
            for joint in hand_urdf.joints:
                if joint.type != 'fixed':
                    prefix = joint.name[:3]
                    # is there an empty prefix ?
                    if "" in list(joint_prefix.values()):
                        joints_tmp.append(joint.name)
                    elif prefix not in list(joint_prefix.values()):
                        rospy.logdebug("joint " + joint.name + " has invalid "
                                       "prefix:" + prefix)
                    elif prefix == joint_prefix[hand]:
                        joints_tmp.append(joint.name)
            for joint_unordered in hand_joints:
                if joint_unordered in joints_tmp:
                    self.joints[mapping[hand]].append(joint_unordered)


class HandFinder:
    """
    The HandFinder is a utility library for detecting Shadow Hands running on
    the system. The idea is to make it easier to write generic code,
     using this library to handle prefixes, joint prefixes etc...
    """
    TIMEOUT_WAIT_FOR_PARAMS_IN_SECS = 60.0

    def __init__(self):
        """
        Parses the parameter server to extract the necessary information.
        """
        self._hand_e = False
        self._hand_parameters = {'joint_prefix': {}, 'mapping': {}}
        self._hand_h = False
        self._hand_h_parameters = {}

        self.wait_for_hand_params(self.TIMEOUT_WAIT_FOR_PARAMS_IN_SECS)

        self.hand_config = HandConfig(self._hand_parameters["mapping"],
                                      self._hand_parameters["joint_prefix"])
        self.hand_joints = HandJoints(self.hand_config.mapping, self.hand_config.joint_prefix).joints

    def wait_for_hand_params(self, timeout_in_secs):
        start_time = rospy.get_time()
        while not rospy.has_param("/hand") and not rospy.has_param("/fh_hand"):
            if rospy.get_time() - start_time > timeout_in_secs:
                rospy.logerr("No hand is detected")
                break
        if rospy.has_param("/hand"):
            rospy.loginfo("Found hand E")
            self._hand_e = True
            self._hand_parameters = rospy.get_param("/hand")
        elif rospy.has_param("/fh_hand"):
            rospy.loginfo("Found hand H")
            self._hand_h = True
            self._hand_h_parameters = rospy.get_param("/fh_hand")

    def get_hand_joints(self):
        # TODO(@anyone): update HandJoints to work with Hand H. Didn't seem necessary yet, so left for now - dg
        if not self._hand_e:
            rospy.logerr("No Hand E present - can't get hand joints")
        return self.hand_joints

    def get_hand_parameters(self):
        if not self._hand_e:
            rospy.logerr("No Hand E present - can't get hand parameters")
        return self.hand_config

    def hand_e_available(self):
        return self._hand_e

    def hand_h_available(self):
        return self._hand_h

    def get_hand_e(self, number=0, serial=None):
        hand_parameters = self.get_hand_parameters()
        if serial is None:
            serial = sorted(hand_parameters.mapping.keys())[number]
        name = "right_hand" if hand_parameters.mapping[serial] == "rh" else "left_hand"
        prefix = hand_parameters.joint_prefix[serial]
        return name, prefix, serial

    def get_hand_h(self, number=0, name=None):
        if name is None:
            name = sorted(self._hand_h_parameters.keys())[number]
        prefix = self._hand_h_parameters[name]['controller_prefix']
        serial = self._hand_h_parameters[name]['palm']['serial_number']
        return "hand_h", prefix, serial
        # TODO(@anyone): replace "hand_h" with name once moveit config is auto-generated with correct movegroup name

    def get_available_prefix(self, number=0, serial=None, name=None):
        if self._hand_e:
            hand_parameters = self.get_hand_parameters()
            if serial is None:
                serial = sorted(hand_parameters.mapping.keys())[number]
            return hand_parameters.joint_prefix[serial]
        if self._hand_h:
            if name is None:
                name = sorted(self._hand_h_parameters.keys())[number]
            return self._hand_h_parameters[name]['controller_prefix']
        return None
