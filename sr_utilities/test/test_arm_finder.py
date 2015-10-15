#!/usr/bin/env python

import rospy
import rostest
import unittest
from sr_utilities.arm_finder import ArmFinder


class TestArmFinder(unittest.TestCase):

    def test_failure(self):
        self.fail("Simple failure")

if __name__ == "__main__":
    rospy.init_node("test_arm_finder")
    rostest.rosrun("sr_utilities", "test_arm_finder", TestArmFinder)
