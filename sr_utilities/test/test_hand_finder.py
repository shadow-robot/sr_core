#!/usr/bin/env python

import rospy
import rospkg
import rostest
import unittest
from sr_utilities.hand_finder import HandFinder
joint_names = ["FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
               "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
               "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"]
controller_params = ["sr_edc_calibration_controllers.yaml",
                     "sr_edc_joint_velocity_controllers_PWM.yaml",
                     "sr_edc_effort_controllers_PWM.yaml",
                     "sr_edc_joint_velocity_controllers.yaml",
                     "sr_edc_effort_controllers.yaml",
                     "sr_edc_mixed_position_velocity_joint_controllers_PWM.yaml",
                     "sr_edc_joint_position_controllers_PWM.yaml",
                     "sr_edc_mixed_position_velocity_joint_controllers.yaml",
                     "sr_edc_joint_position_controllers.yaml"]


class TestHandFinder(unittest.TestCase):
    rospack = rospkg.RosPack()
    ethercat_path = rospack.get_path('sr_ethercat_hand_config')

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("right", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['right']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["right"], "FFJ3 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['right']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/right/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['right']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['right']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/right/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['right']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/right/" + controller_param,
                             "incorrect controller config file")

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        print hand_finder.get_hand_joints()
        self.assertIn("1", hand_finder.get_hand_joints(), "Serial should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']  # use serial
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/1/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/1/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/1/" + controller_param,
                             "incorrect controller config file")

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['rh']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['rh']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/rh/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['rh']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['rh']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/rh/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['rh']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/rh/" + controller_param,
                             "incorrect controller config file")

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("1", hand_finder.get_hand_joints(), "Serial should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/1/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/1/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/1/" + controller_param,
                             "incorrect controller config file")

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
        self.assertIn("right", hand_finder.get_hand_parameters().mapping.values(), "It should be right mapping")
        self.assertIn("left", hand_finder.get_hand_parameters().mapping.values(), "It should be left mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 2, "It should be two joint_prefixes")

        self.assertIn("rh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")
        self.assertIn("lh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")

        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 2, "It should be two mappings")
        self.assertIn("right", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['right']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("rh_FFJ3", hand_finder.get_hand_joints()["right"], "FFJ3 joint should be in the joints list")
        self.assertIn("left", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['left']
        self.assertEqual(len(joints), 24, "Joint number should be 24")
        self.assertIn("lh_FFJ1", hand_finder.get_hand_joints()["left"], "FFJ1 joint should be in the joints list")

        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['right']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/right/" + "calibration.yaml",
                         "incorrect calibration file")
        calibration_path = hand_finder.get_calibration_path()['left']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/left/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['right']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['right']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/right/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['right']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/right/" + controller_param,
                             "incorrect controller config file")

        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['left']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['left']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/left/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['left']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/left/" + controller_param,
                             "incorrect controller config file")

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("right", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['right']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("rh_FFJ3", hand_finder.get_hand_joints()["right"],
                         "FFJ3 joint should not be in the joints list")
        self.assertIn("rh_RFJ4", hand_finder.get_hand_joints()["right"], "RFJ4 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['right']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/right/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['right']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['right']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/right/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['right']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/right/" + controller_param,
                             "incorrect controller config file")

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("1", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("rh_FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should not be in the joints list")
        self.assertIn("rh_RFJ4", hand_finder.get_hand_joints()["1"], "RFJ4 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/1/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/1/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/1/" + controller_param,
                             "incorrect controller config file")

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("rh", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['rh']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("FFJ3", hand_finder.get_hand_joints()["rh"], "FFJ3 joint should not be in the joints list")
        self.assertIn("RFJ4", hand_finder.get_hand_joints()["rh"], "RFJ4 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['rh']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/rh/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['rh']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['rh']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/rh/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['rh']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/rh/" + controller_param,
                             "incorrect controller config file")

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
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 1, "It should be only one mapping")
        self.assertIn("1", hand_finder.get_hand_joints(), "Mapping should be in the joints result")
        joints = hand_finder.get_hand_joints()['1']
        self.assertEqual(len(joints), 1, "Joint number should be 1")
        self.assertNotIn("FFJ3", hand_finder.get_hand_joints()["1"], "FFJ3 joint should not be in the joints list")
        self.assertIn("RFJ4", hand_finder.get_hand_joints()["1"], "RFJ4 joint should be in the joints list")
        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['1']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/1/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['1']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['1']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/1/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['1']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/1/" + controller_param,
                             "incorrect controller config file")

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
        self.assertIn("right", hand_finder.get_hand_parameters().mapping.values(), "It should be right mapping")
        self.assertIn("left", hand_finder.get_hand_parameters().mapping.values(), "It should be left mapping")
        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix), 2, "It should be two joint_prefixes")

        self.assertIn("rh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")
        self.assertIn("lh_", hand_finder.get_hand_parameters().joint_prefix.values(), "It should be rh_ prefix")

        # hand joints
        self.assertIsNotNone(hand_finder.get_hand_joints(), "Joints extracted.")
        self.assertEqual(len(hand_finder.get_hand_joints().keys()), 2, "It should be two mappings")
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

        # calibration
        self.assertIsNotNone(hand_finder.get_calibration_path(), "Calibration extracted.")
        calibration_path = hand_finder.get_calibration_path()['right']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/right/" + "calibration.yaml",
                         "incorrect calibration file")
        calibration_path = hand_finder.get_calibration_path()['left']
        self.assertEqual(calibration_path, self.ethercat_path + "/calibrations/left/" + "calibration.yaml",
                         "incorrect calibration file")
        # tuning
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(), "Control tuning parameters extracted.")
        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['right']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['right']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/right/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['right']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/right/" + controller_param,
                             "incorrect controller config file")

        ctrl_tun_friction_comp_path = hand_finder.get_hand_control_tuning().friction_compensation['left']
        self.assertEqual(ctrl_tun_friction_comp_path, self.ethercat_path + "/controls/friction_compensation.yaml",
                         "incorrect friction compensation file")
        ctrl_tun_motors_path = hand_finder.get_hand_control_tuning().motor_control['left']
        self.assertEqual(ctrl_tun_motors_path, self.ethercat_path +
                         "/controls/motors/left/motor_board_effort_controllers.yaml",
                         "incorrect motor config file")
        ctrl_tun_host_control_paths = hand_finder.get_hand_control_tuning().host_control['left']
        self.assertEqual(len(ctrl_tun_host_control_paths), len(controller_params),
                         "incorrect number of host controllers param")
        for controller_path, controller_param in zip(ctrl_tun_host_control_paths, controller_params):
            self.assertEqual(controller_path, self.ethercat_path + "/controls/host/left/" + controller_param,
                             "incorrect controller config file")

if __name__ == "__main__":
    rospy.init_node("test_hand_finder")
    rostest.rosrun("sr_utilities", "test_hand_finder", TestHandFinder)
