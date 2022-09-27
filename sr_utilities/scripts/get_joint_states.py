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
import _thread
import rospy
from sensor_msgs.msg import JointState
from sr_utilities.srv import getJointState

RATE = 100


class GetJointState:
    def __init__(self):
        rospy.init_node('get_joint_state_service', anonymous=True)

        self.subs_1 = rospy.Subscriber("/joint_states", JointState, self.callback1)
        self.serv = rospy.Service('/getJointState', getJointState, self.get_joint_state_cb)

        self.joint_state_msg = JointState()

        self.mutex = _thread.allocate_lock()

        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            rate.sleep()

    def callback1(self, data):
        self.joint_state_msg = data

    def get_joint_state_cb(self, req):
        res = self.joint_state_msg
        return res


if __name__ == '__main__':
    service = GetJointState()
