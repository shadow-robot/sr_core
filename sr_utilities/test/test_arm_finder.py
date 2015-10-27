#!/usr/bin/env python

import rospy
import rostest
import unittest
from sr_utilities.arm_finder import ArmFinder


class TestArmFinder(unittest.TestCase):

    def test_no_hand_no_arm_finder(self):
        if rospy.has_param("arm"):
            rospy.delete_param("arm")

        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        arm_finder = ArmFinder()
        self.assertIsNotNone(arm_finder.get_arm_parameters(), "Parameters extracted.")
        self.assertIsNotNone(arm_finder.get_arm_joints(), "Joints extracted.")

        self.assertEqual(len(arm_finder.get_arm_parameters().mapping), 0, "It should be zero mapping")
        self.assertEqual(len(arm_finder.get_arm_parameters().joint_prefix), 0, "It should be zero joint_prefix")
        self.assertEqual(len(arm_finder.get_arm_joints().keys()), 0, "It should be zero arm joints")

    def test_one_hand_no_arm_finder(self):
        if rospy.has_param("arm"):
            rospy.delete_param("arm")

        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        arm_finder = ArmFinder()
        self.assertIsNotNone(arm_finder.get_arm_parameters(), "Parameters extracted.")
        self.assertIsNotNone(arm_finder.get_arm_joints(), "Joints extracted.")

        self.assertEqual(len(arm_finder.get_arm_parameters().mapping), 0, "It should be zero mapping")
        self.assertEqual(len(arm_finder.get_arm_parameters().joint_prefix), 0, "It should be zero joint_prefix")
        self.assertEqual(len(arm_finder.get_arm_joints().keys()), 0, "It should be zero arm joints")

    def test_no_hand_one_arm_finder(self):
        if rospy.has_param("arm"):
            rospy.delete_param("arm")

        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("robot_description", rospy.get_param("no_hand_one_arm_description"))

        rospy.set_param("arm/joint_prefix/1", "la_")
        rospy.set_param("arm/mapping/1", "la")

        arm_finder = ArmFinder()

        self.assertIsNotNone(arm_finder.get_arm_parameters(), "Parameters extracted.")
        self.assertIsNotNone(arm_finder.get_arm_joints(), "Joints extracted.")

        self.assertEqual(len(arm_finder.get_arm_parameters().mapping), 1, "It should be one mapping")
        self.assertEqual(len(arm_finder.get_arm_parameters().joint_prefix), 1, "It should be one joint_prefix")
        self.assertEqual(len(arm_finder.get_arm_joints().keys()), 1, "It should be one arm joints")

        self.assertIn("la", arm_finder.get_arm_parameters().mapping.values(), "It should be la mapping")
        self.assertIn("la_", arm_finder.get_arm_parameters().joint_prefix.values(), "It should be la_ prefix")
        self.assertIn("la", arm_finder.get_arm_joints(), "Maping should be in the joints result")

    def test_one_hand_two_arms_finder(self):
        if rospy.has_param("arm"):
            rospy.delete_param("arm")

        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("robot_description", rospy.get_param("right_hand_two_arms"))

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")

        rospy.set_param("arm/joint_prefix/1", "la_")
        rospy.set_param("arm/mapping/1", "la")

        rospy.set_param("arm/joint_prefix/2", "ra_")
        rospy.set_param("arm/mapping/2", "ra")

        arm_finder = ArmFinder()

        self.assertIsNotNone(arm_finder.get_arm_parameters(), "Parameters extracted.")
        self.assertIsNotNone(arm_finder.get_arm_joints(), "Joints extracted.")

        self.assertEqual(len(arm_finder.get_arm_parameters().mapping), 2, "It should be two mappings")
        self.assertEqual(len(arm_finder.get_arm_parameters().joint_prefix), 2, "It should be two joint_prefixes")
        self.assertEqual(len(arm_finder.get_arm_joints().keys()), 2, "It should be two arm joints mappings")

        self.assertIn("ra", arm_finder.get_arm_parameters().mapping.values(), "It should be ra mapping")
        self.assertIn("ra_", arm_finder.get_arm_parameters().joint_prefix.values(), "It should be ra_ prefix")
        self.assertIn("ra", arm_finder.get_arm_joints(), "Maping should be in the joints result")
        self.assertEqual(len(arm_finder.get_arm_joints()["ra"]), 1, "It should be one arm joint for mapping")

        self.assertIn("la", arm_finder.get_arm_parameters().mapping.values(), "It should be la mapping")
        self.assertIn("la_", arm_finder.get_arm_parameters().joint_prefix.values(), "It should be la_ prefix")
        self.assertIn("la", arm_finder.get_arm_joints(), "Maping should be in the joints result")
        self.assertEqual(len(arm_finder.get_arm_joints()["la"]), 1, "It should be one arm joint for mapping")

    def test_two_hands_one_arm_finder(self):
        if rospy.has_param("arm"):
            rospy.delete_param("arm")

        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("robot_description", rospy.get_param("two_hands_left_arm"))

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")

        rospy.set_param("hand/joint_prefix/2", "lh_")
        rospy.set_param("hand/mapping/2", "lh")

        rospy.set_param("arm/joint_prefix/1", "la_")
        rospy.set_param("arm/mapping/1", "la")

        arm_finder = ArmFinder()

        self.assertEqual(len(arm_finder.get_arm_parameters().mapping), 1, "It should be one mapping")
        self.assertEqual(len(arm_finder.get_arm_parameters().joint_prefix), 1, "It should be one joint_prefix")
        self.assertEqual(len(arm_finder.get_arm_joints().keys()), 1, "It should be one arm joints")

        self.assertIn("la", arm_finder.get_arm_parameters().mapping.values(), "It should be la mapping")
        self.assertIn("la_", arm_finder.get_arm_parameters().joint_prefix.values(), "It should be la_ prefix")
        self.assertIn("la", arm_finder.get_arm_joints(), "Maping should be in the joints result")

if __name__ == "__main__":
    rospy.init_node("test_arm_finder")
    rostest.rosrun("sr_utilities", "test_arm_finder", TestArmFinder)
