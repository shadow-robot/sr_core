#!/usr/bin/env python

# Copyright 2011 Shadow Robot Company Ltd.
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

import rospy
from sensor_msgs.msg import JointState
from sr_utilities.srv import getJointState
import thread

RATE = 100


class GetJointState(object):
    def __init__(self):
        rospy.init_node('get_joint_state_service', anonymous=True)

        self.subs_1 = rospy.Subscriber("/joint_states", JointState, self.callback1)
        self.serv = rospy.Service('/getJointState', getJointState, self.getJointStateCB)

        self.joint_state_msg = JointState()

        self.mutex = thread.allocate_lock()

        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            r.sleep()

    def callback1(self, data):
        self.joint_state_msg = data

    def getJointStateCB(self, req):
        res = self.joint_state_msg
        return res

if __name__ == '__main__':
    service = GetJointState()
