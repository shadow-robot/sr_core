#!/usr/bin/env python

import rospy
import rostest
import unittest
from sr_utilities.hand_finder import HandFinder


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
        self.assertEqual(len(hand_finder.get_calibration_path()), 0, "correct calibration path without a hand")
        self.assertEqual(len(hand_finder.get_hand_control_tuning(). friction_compensation), 0,
                         "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning(). host_control), 0, "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning(). motor_control), 0,
                         "correct tuning without a hands")

    def test_one_hand_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")

        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")

        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "rh", "It should be only rh mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "rh_", "It should be only rh_ prefix")

        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Maping should be in the joints result")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should be in the joints list")

    def test_two_hand_no_robot_description_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")
        rospy.set_param("hand/joint_prefix/2", "lh_")
        rospy.set_param("hand/mapping/2", "lh")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")

        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 2, "It should be two mappings")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 2, "It should be two joint_prefixes")
        self.assertIn("rh", hand_finder.get_hand_parameters().mapping.values(), "It should be rh mapping")
        self.assertIn("lh", hand_finder.get_hand_parameters().mapping.values(), "It should be lh mapping")
        self.assertIn("rh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")
        self.assertIn("lh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")

        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 2, "It should be two mappings")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Maping should be in the joints result")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should be in the joints list")
        self.assertIn("lh", hand_finder.get_hand_joints(), "Maping should be in the joints result")
        self.assertIn("lh_FFJ1", hand_finder.get_hand_joints()["lh"], "FFJ3 joint should be in the joints list")

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
        self.assertEqual(len(hand_finder.get_calibration_path()), 0, "correct calibration path without a hand")
        self.assertEqual(len(hand_finder.get_hand_control_tuning(). friction_compensation), 0,
                         "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning(). host_control), 0, "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning(). motor_control), 0,
                         "correct tuning without a hands")

    def test_one_hand_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("robot_description", rospy.get_param("right_hand_description"))

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")

        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 1, "It should be only one mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 1, "It should be only one joint_prefix")

        self.assertEqual(hand_finder.get_hand_parameters().mapping['1'], "rh", "It should be only rh mapping")
        self.assertEqual(hand_finder.get_hand_parameters().joint_prefix['1'], "rh_", "It should be only rh_ prefix")

        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Maping should be in the joints result")
        self.assertNotIn("rh_FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should not be in the joints list")
        self.assertIn("rh_RFJ4", hand_finder.get_hand_joints()["rh"], "RFJ4 joint should be in the joints list")

    def test_two_hand_robot_description_exists_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        if rospy.has_param("robot_description"):
            rospy.delete_param("robot_description")

        rospy.set_param("robot_description", rospy.get_param("two_hands_description"))

        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")
        rospy.set_param("hand/joint_prefix/2", "lh_")
        rospy.set_param("hand/mapping/2", "lh")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(), "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")

        self.assertEqual(len(hand_finder.get_hand_parameters().mapping), 2, "It should be two mappings")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 2, "It should be two joint_prefixes")
        self.assertIn("rh", hand_finder.get_hand_parameters().mapping.values(), "It should be rh mapping")
        self.assertIn("lh", hand_finder.get_hand_parameters().mapping.values(), "It should be lh mapping")
        self.assertIn("rh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")
        self.assertIn("lh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")

        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 2, "It should be two mappings")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Maping should be in the joints result")
        self.assertNotIn("rh_FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should not be in the joints list")
        self.assertIn("rh_RFJ4", hand_finder.get_hand_joints()["rh"], "RFJ4 joint should be in the joints list")
        self.assertIn("lh", hand_finder.get_hand_joints(), "Maping should be in the joints result")
        self.assertNotIn("lh_FFJ1", hand_finder.get_hand_joints()["lh"], "FFJ3 joint should not be in the joints list")
        self.assertIn("lh_LFJ4", hand_finder.get_hand_joints()["lh"], "LFJ4 joint should be in the joints list")

if __name__ == "__main__":
    rospy.init_node("test_hand_finder")
    rostest.rosrun("sr_utilities", "test_hand_finder", TestHandFinder)
