#!/usr/bin/env python
#
# Copyright 2014 Shadow Robot Company Ltd.
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
import sys
import rospy
from ros_ethercat_model.calibrate_class import Calibrate
from sr_utilities.hand_finder import HandFinder


class CalibrateWithHand(Calibrate):

    @staticmethod
    def generate_controllers():
        controllers = []
        hand_finder = HandFinder()
        joints = hand_finder.get_hand_joints()
        mapping = hand_finder.get_hand_parameters().mapping
        for hand in mapping:
            prefix = mapping[hand]
            for joint in joints[prefix]:
                if joint[3:5].lower() == 'th' or joint[3:5].lower() == 'wr' or (joint[6] != '1' and joint[6] != '2'):
                    joint_controller = 'cal_sh_' + joint.lower()
                else:
                    joint = joint[:6] + '0'
                    joint_controller = 'cal_sh_' + joint.lower()
                if joint_controller not in controllers:
                    controllers.append(joint_controller)
        return controllers


def main():
    if rospy.is_shutdown():
        return

    rospy.init_node('calibration', anonymous=True)

    calibrate_class = CalibrateWithHand()
    controllers = calibrate_class.generate_controllers()

    calibrate_class.pub_calibrated.publish(False)
    # Don't calibrate the IMU unless ordered to by user
    cal_imu = rospy.get_param('calibrate_imu', False)

    if cal_imu:
        imustatus = calibrate_class.calibrate_imu()
    else:
        imustatus = True

    if not calibrate_class.calibrate(controllers):
        sys.exit(3)

    calibrate_class.pub_calibrated.publish(True)

    if not imustatus:
        print("Mechanism calibration complete, but IMU calibration failed")
    else:
        print("Calibration complete")

    rospy.spin()

if __name__ == '__main__':
    main()
