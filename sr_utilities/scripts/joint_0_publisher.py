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


class Joint0Publisher:
    def __init__(self):
        rospy.init_node('joint_0_publisher', anonymous=True)
        rospy.wait_for_message("joint_states", JointState)
        self.sub = rospy.Subscriber("joint_states", JointState, self.callback)
        self.pub = rospy.Publisher("joint_0s/joint_states", JointState, queue_size=1)
        self.joint_state_msg = JointState()

        rospy.spin()

    def callback(self, data):
        self.joint_state_msg.header.stamp = rospy.Time.now()

        self.joint_state_msg.name = []
        self.joint_state_msg.position = []
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []

        fj0 = [0.0, 0.0, 0.0]
        for name, position, velocity, effort in zip(data.name,
                                                    data.position,
                                                    data.velocity,
                                                    data.effort):
            if "FJ1" in name and len(name) == 4:
                fj0 = [position, velocity, effort]

            if "FJ2" in name and len(name) == 4:
                fj0 = [fj0[0] + position, fj0[1] + velocity, fj0[2] + effort]
                self.joint_state_msg.name.append(name[0]+"FJ0")
                self.joint_state_msg.position.append(fj0[0])
                self.joint_state_msg.velocity.append(fj0[1])
                self.joint_state_msg.effort.append(fj0[2])

        self.pub.publish(self.joint_state_msg)

if __name__ == '__main__':
    j0_pub = Joint0Publisher()
