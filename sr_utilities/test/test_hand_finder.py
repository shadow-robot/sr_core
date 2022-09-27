#!/usr/bin/env python3

# Copyright 2011, 2022 Shadow Robot Company Ltd.
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
import unittest
import rospy
import rostest
from sr_utilities.hand_finder import HandFinder
joint_names = ["FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
               "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
               "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"]


class TestHandFinder(unittest.TestCase):
    def test_no_hand_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        hand_finder = HandFinder()

        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_joints()), 0, "correct joints without a hand")

    def test_one_hand_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "right")

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "right", "It should be only right mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "rh_", "It should be only rh_ prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        self.assertIn("right", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['right']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["right"], "FFJ3 joint should be in the joints list")

    def test_one_hand_no_mapping_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "")

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "1", "It should be the serial id as mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "rh_", "It should be only rh_ prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        print(hand_finder.get_hand_joints())
        self.assertIn("1", hand_finder.get_hand_joints(), "Serial should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']  # use serial
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should be in the joints list")

    def test_one_hand_no_prefix_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "")
        rospy.set_param("hand/mapping/1", "rh")

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "rh", "It should be only right mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "", "It should be only an empty prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['rh']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should be in the joints list")

    def test_one_hand_no_prefix_no_mapping_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "")
        rospy.set_param("hand/mapping/1", "")

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "1", "It should be the serial id as mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "", "It should be only an empty prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        self.assertIn("1", hand_finder.get_hand_joints(), "Serial should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should be in the joints list")

    def test_two_hand_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "right")
        rospy.set_param("hand/joint_prefix/2", "lh_")
        rospy.set_param("hand/mapping/2", "left")

        hand_finder = HandFinder()

        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 2, "It should be two mappings")
        self.assertIn("right", list(hand_finder.get_hand_parameters().mapping.values()), "It should be right mapping")
        self.assertIn("left", list(hand_finder.get_hand_parameters().mapping.values()), "It should be left mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 2, "It should be two joint_prefixes")

        self.assertIn("rh_", list(hand_finder.get_hand_parameters().joint_prefix.values()), "It should be rh_ prefix")
        self.assertIn("lh_", list(hand_finder.get_hand_parameters().joint_prefix.values()), "It should be rh_ prefix")

        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 2, "It should be two mappings")
        self.assertIn("right", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['right']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["right"], "FFJ3 joint should be in the joints list")
        self.assertIn("left", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['left']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("lh_FFJ1", hand_finder.get_hand_joints()["left"], "FFJ1 joint should be in the joints list")

    def test_no_hand_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("robot_description", rospy.get_param("two_hands_description"))

        hand_finder = HandFinder()

        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_joints()), 0, "correct joints without a hand")

    def test_one_hand_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "right")
        rospy.set_param("robot_description", rospy.get_param("right_hand_description"))

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "right", "It should be only right mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "rh_", "It should be only rh_ prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        self.assertIn("right", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['right']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("rh_FFJ3", hand_finder.get_hand_joints()["right"],
                         "FFJ3 joint should not be in the joints list")
        self.assertIn("rh_RFJ4", hand_finder.get_hand_joints()["right"], "RFJ4 joint should be in the joints list")

    def test_one_hand_no_mapping_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "")
        rospy.set_param("robot_description", rospy.get_param("right_hand_description"))

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "1", "It should be the serial id as mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "rh_", "It should be only rh_ prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        self.assertIn("1", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("rh_FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should not be in the joints list")
        self.assertIn("rh_RFJ4", hand_finder.get_hand_joints()["1"], "RFJ4 joint should be in the joints list")

    def test_one_hand_no_prefix_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "")
        rospy.set_param("hand/mapping/1", "rh")
        rospy.set_param("robot_description", rospy.get_param("right_hand_description_no_prefix"))

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "rh", "It should be only right mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "", "It should be only an empty prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['rh']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should not be in the joints list")
        self.assertIn("RFJ4", hand_finder.get_hand_joints()["rh"], "RFJ4 joint should be in the joints list")

    def test_one_hand_no_prefix_no_ns_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "")
        rospy.set_param("hand/mapping/1", "")
        rospy.set_param("robot_description", rospy.get_param("right_hand_description_no_prefix"))

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")
        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "1", "It should be the serial id as mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "", "It should be only an empty prefix")
        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 1, "It should be only one mapping")
        self.assertIn("1", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should not be in the joints list")
        self.assertIn("RFJ4", hand_finder.get_hand_joints()["1"], "RFJ4 joint should be in the joints list")

    def test_two_hand_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "right")
        rospy.set_param("hand/joint_prefix/2", "lh_")
        rospy.set_param("hand/mapping/2", "left")
        rospy.set_param("robot_description", rospy.get_param("two_hands_description"))

        hand_finder = HandFinder()
        # hand params
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 2, "It should be two mappings")
        self.assertIn("right", list(hand_finder.get_hand_parameters().mapping.values()), "It should be right mapping")
        self.assertIn("left", list(hand_finder.get_hand_parameters().mapping.values()), "It should be left mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 2, "It should be two joint_prefixes")

        self.assertIn("rh_", list(hand_finder.get_hand_parameters().joint_prefix.values()), "It should be rh_ prefix")
        self.assertIn("lh_", list(hand_finder.get_hand_parameters().joint_prefix.values()), "It should be rh_ prefix")

        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(list(hand_finder.get_hand_joints().keys())), 2, "It should be two mappings")
        self.assertIn("right", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['right']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("rh_FFJ3", hand_finder.get_hand_joints()["right"],
                         "rh_FFJ3 joint should not be in the joints list")
        self.assertIn("rh_RFJ4", hand_finder.get_hand_joints()["right"], "rh_RFJ4 joint should be in the joints list")
        self.assertIn("left", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['left']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("lh_FFJ3", hand_finder.get_hand_joints()["left"],
                         "lh_FFJ3 joint should not be in the joints list")
        self.assertIn("lh_LFJ4", hand_finder.get_hand_joints()["left"], "lh_LFJ4 joint should be in the joints list")


if __name__ == "__main__":
    rospy.init_node("test_hand_finder")
    rostest.rosrun("sr_utilities", "test_hand_finder", TestHandFinder)
