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


class MergeMessages:
    def __init__(self):
        rospy.init_node('arm_and_hand_joint_state_merger', anonymous=True)

        self.msg_1_received = False
        self.msg_2_received = False

        self.subs_1 = rospy.Subscriber("/sh/joint_states", JointState, self.callback1)
        self.subs_2 = rospy.Subscriber("/sa/joint_states", JointState, self.callback2)
        self.serv = rospy.Service('/getJointState', getJointState, self.get_joint_state_cb)

        self.pub = rospy.Publisher("/joint_states", JointState)

        self.joint_state_msg = JointState()
        self.joint_state_msg_1 = JointState()
        self.joint_state_msg_2 = JointState()

        self.mutex = _thread.allocate_lock()

        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def callback1(self, data):
        self.msg_1_received = True
        self.joint_state_msg_1 = data

    def callback2(self, data):
        self.msg_2_received = True
        self.joint_state_msg_2 = data

    def publish(self):
        if self.msg_1_received and self.msg_2_received:
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_msg.name = self.joint_state_msg_1.name + self.joint_state_msg_2.name
            self.joint_state_msg.position = self.joint_state_msg_1.position + self.joint_state_msg_2.position
            self.joint_state_msg.effort = self.joint_state_msg_1.effort + self.joint_state_msg_2.effort
            self.joint_state_msg.velocity = self.joint_state_msg_1.velocity + self.joint_state_msg_2.velocity

            self.pub.publish(self.joint_state_msg)
            self.msg_1_received = False
            self.msg_2_received = False

    def get_joint_state_cb(self, req):
        res = self.joint_state_msg
        return res


if __name__ == '__main__':
    merger = MergeMessages()
