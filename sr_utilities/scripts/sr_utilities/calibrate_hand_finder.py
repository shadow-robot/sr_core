#!/usr/bin/env python3

# Copyright 2014, 2020, 2022-2023 Shadow Robot Company Ltd.
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
import sys
import rospy
import rosservice
from sr_utilities.hand_finder import HandFinder
from std_msgs.msg import Bool
from std_srvs.srv import Empty


CALIBRATE_TIMEOUT = 120.0


class CalibrateHand:
    """
    This class resets all the motor boards of the Shadow Hand E present in the system.
    On reset, the motor board firmware causes a jiggle and zeroes the tendon strain gauges.
    Once the hands have been successfully reset, this class publishes True on the topic /calibrated
    """

    def __init__(self, timeout=CALIBRATE_TIMEOUT):
        rospy.wait_for_service("controller_manager/load_controller", timeout=timeout)
        self.pub_calibrated = rospy.Publisher('calibrated', Bool, queue_size=1, latch=True)

    @staticmethod
    def generate_reset_services_list():
        reset_service_list = []
        generated_reset_service_list = []
        service_list = []

        # We first read the list of available motor reset services in this namespace
        # this will allow us to avoid having to know the name of the robot driver node
        namespace = rospy.get_namespace()
        while not reset_service_list:
            rospy.sleep(0.5)
            service_list = rosservice.get_service_list(namespace=namespace)
            reset_service_list = [srv for srv in service_list if '/reset_motor_' in srv]
            if not reset_service_list:
                rospy.loginfo("Waiting for motor reset services")

        # We generate a list of the services that we expect to find
        # and merely throw a warning if some of them didn't turn up in the available service list
        hand_finder = HandFinder()
        joints = hand_finder.get_hand_joints()
        mapping = hand_finder.get_hand_parameters().mapping
        for hand in mapping:
            prefix = mapping[hand]
            for joint in joints[prefix]:
                if not (joint[3:5].lower() == 'th' or joint[3:5].lower() == 'wr' or
                        (joint[6] != '1' and joint[6] != '2')):
                    joint = joint[:6] + '0'

                joint_reset_service = joint[:2] + '/reset_motor_' + joint[3:].upper()

                if joint_reset_service not in generated_reset_service_list:
                    generated_reset_service_list.append(joint_reset_service)
        for gen_srv in generated_reset_service_list:
            gen_srv_match_list = [srv for srv in reset_service_list if gen_srv in srv]
            if not gen_srv_match_list:
                rospy.logwarn("Expected service not found: %s", gen_srv)

        return reset_service_list

    @staticmethod
    def calibrate(services):
        success = True
        for srv in services:
            rospy.wait_for_service(srv, timeout=4.0)
            try:
                reset_motor = rospy.ServiceProxy(srv, Empty)
                reset_motor()
            except rospy.ServiceException as exc:
                success = False
                rospy.logerr("Motor reset failed: %s Exception: %s", srv, str(exc))
        return success


def main():
    if rospy.is_shutdown():
        return

    rospy.init_node('calibration', anonymous=True)
    # By default should be enabled, except when shutting system down
    ros_node_shutdown = rospy.get_param("~node_shutdown", False)
    ros_node_timeout = rospy.get_param("~node_timeout", CALIBRATE_TIMEOUT)
    calibrate_class = CalibrateHand(ros_node_timeout)
    services = calibrate_class.generate_reset_services_list()
    calibrate_class.pub_calibrated.publish(False)
    if not calibrate_class.calibrate(services):
        sys.exit(3)
    calibrate_class.pub_calibrated.publish(True)

    print("Hand calibration complete")
    if not ros_node_shutdown:  # Used to allow us to make the hand jiggle when closing everything.
        rospy.spin()


if __name__ == '__main__':
    main()
