#!/usr/bin/env python3

# Copyright 2011, 2022, 2023 Shadow Robot Company Ltd.
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


class MergeMessages:
    def __init__(self):
        rospy.init_node('joint_state_merger', anonymous=True)
        self.subs_1 = rospy.Subscriber("/srh/position/joint_states", JointState, self.callback1)
        self.subs_2 = rospy.Subscriber("/sr_arm/position/joint_states", JointState, self.callback2)

        self.pub = rospy.Publisher("/joint_states", JointState)

        self.msg_1_received = False
        self.msg_2_received = False

        self.joint_state_msg = JointState()

        self.mutex = _thread.allocate_lock()

        rospy.spin()

    def callback1(self, data):
        self.mutex.acquire()
        if self.msg_1_received:
            self.mutex.release()
            return

        self.msg_1_received = True

        self.joint_state_msg.header.stamp = rospy.Time.now()

        tmp = self.joint_state_msg.name
        tmp += data.name
        self.joint_state_msg.name = tmp

        tmp = self.joint_state_msg.position
        tmp += data.position
        self.joint_state_msg.position = tmp

        tmp = self.joint_state_msg.velocity
        tmp += data.velocity
        self.joint_state_msg.velocity = tmp

        tmp = self.joint_state_msg.effort
        tmp += data.effort
        self.joint_state_msg.effort = tmp

        if self.msg_2_received:
            self.pub.publish(self.joint_state_msg)
            self.msg_1_received = False
            self.msg_2_received = False
            self.joint_state_msg = JointState()

        self.mutex.release()

    def callback2(self, data):
        self.mutex.acquire()
        if self.msg_2_received:
            self.mutex.release()
            return

        self.msg_2_received = True

        self.joint_state_msg.header.stamp = rospy.Time.now()

        tmp = self.joint_state_msg.name
        tmp += data.name
        self.joint_state_msg.name = tmp

        tmp = self.joint_state_msg.position
        tmp += data.position
        self.joint_state_msg.position = tmp

        tmp = self.joint_state_msg.velocity
        tmp += data.velocity
        self.joint_state_msg.velocity = tmp

        tmp = self.joint_state_msg.effort
        tmp += data.effort
        self.joint_state_msg.effort = tmp

        if self.msg_1_received:
            self.pub.publish(self.joint_state_msg)
            self.msg_1_received = False
            self.msg_2_received = False
            self.joint_state_msg = JointState()

        self.mutex.release()


if __name__ == '__main__':
    merger = MergeMessages()
