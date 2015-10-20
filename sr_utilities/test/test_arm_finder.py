#!/usr/bin/env python

import rospy
import rostest
import unittest
from sr_utilities.arm_finder import ArmFinder


class TestArmFinder(unittest.TestCase):

    def test_no_arm_finder(self):
        if rospy.has_param("arm"):
            rospy.delete_param("arm")

        arm_finder = ArmFinder()

        self.assertEqual(len(arm_finder.get_arm_parameters().joint_prefix),
                         0, "correct parameters without an arm")
        self.assertEqual(len(arm_finder.get_arm_parameters().mapping),
                         0, "correct parameters without an arm")
        self.assertEqual(len(arm_finder.get_arm_joints()), 0,
                         "correct joints without an arm")

    def test_one_arm_finder(self):
        rospy.set_param("arm/joint_prefix/1", "ra_")
        rospy.set_param("arm/mapping/1", "ra")

        arm_finder = ArmFinder()
        self.assertIsNotNone(arm_finder.get_arm_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(arm_finder.get_arm_joints(),
                             "Joints extracted.")

    def test_two_arms_finder(self):
        rospy.set_param("arm/joint_prefix/1", "ra_")
        rospy.set_param("arm/mapping/1", "ra")
        rospy.set_param("arm/joint_prefix/2", "la_")
        rospy.set_param("arm/mapping/2", "la")

        arm_finder = ArmFinder()
        self.assertIsNotNone(arm_finder.get_arm_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(arm_finder.get_arm_joints(),
                             "Joints extracted.")

if __name__ == "__main__":
    rospy.init_node("test_arm_finder")
    rostest.rosrun("sr_utilities", "test_arm_finder", TestArmFinder)
